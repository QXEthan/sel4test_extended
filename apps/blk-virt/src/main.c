/*
 * Copyright 2024 Ethan Xu
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdio.h>

#include <sel4/sel4.h>
#include <vka/capops.h>
#include <allocman/allocman.h>
#include <allocman/vka.h>
#include <allocman/bootstrap.h>
#include <sel4utils/thread.h>
#include <serial_server/parent.h>
#include <serial_server/client.h>

#include <queue.h>
#include <ialloc.h>
#include <storage_info.h>
#include <msdos_mbr.h>
#include <simple-default/simple-default.h>

typedef struct region_resource {
    void *vaddr;
    uint64_t size;
} region_resource_t;

typedef struct blk_connection_resource {
    region_resource_t storage_info;
    region_resource_t req_queue;
    region_resource_t resp_queue;
    uint16_t num_buffers;
    uint8_t id;
} blk_connection_resource_t;

typedef struct device_region_resource {
    region_resource_t region;
    uintptr_t io_addr;
} device_region_resource_t;


typedef struct blk_virt_config_client {
    blk_connection_resource_t conn;
    device_region_resource_t data;
    uint32_t partition;
} blk_virt_config_client_t;

typedef struct blk_virt_boot_info {
    blk_virt_config_client_t client;
    uintptr_t driver_storage_info_vaddr;
    uintptr_t driver_request_shmem_vaddr;
    uintptr_t driver_request_paddr;
    uintptr_t driver_response_shmem_vaddr;
    uintptr_t driver_response_paddr;
    uintptr_t driver_data_vaddr;
    uintptr_t driver_data_paddr;

    uintptr_t client_storage_info_vaddr;
    uintptr_t client_request_shmem_vaddr;
    uintptr_t client_request_paddr;
    uintptr_t client_response_shmem_vaddr;
    uintptr_t client_response_paddr;
    uintptr_t client_data_vaddr;
    uintptr_t client_data_paddr;

    // notification
    seL4_CPtr driver_nt;
    seL4_CPtr client_nt;

    seL4_CPtr virt_nt;


} blk_virt_boot_info_t;

blk_queue_handle_t blk_v2d_queue;

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE
#define DRIVER_MAX_NUM_BUFFERS 1024

/* Request info to be bookkept from client */
typedef struct reqbk {
    uint32_t cli_id;
    uint32_t cli_req_id;
    uintptr_t vaddr;
    uint16_t count;
    blk_req_code_t code;
} reqbk_t;
static reqbk_t reqsbk[DRIVER_MAX_NUM_BUFFERS];

#define MICROKIT_MAX_CHANNELS 62
#define SDDF_BLK_MAX_CLIENTS (MICROKIT_MAX_CHANNELS - 1)
/* Client specific info */
typedef struct client {
    blk_queue_handle_t queue_h;
    uint32_t start_sector;
    uint32_t sectors;
} client_t;
client_t client;

// virt.c
#define DRIVER_MAX_NUM_BUFFERS 1024
/* Index allocator for driver request id */
static ialloc_t ialloc;
static uint32_t ialloc_idxlist[DRIVER_MAX_NUM_BUFFERS];


#define BOOTINFO_VADDR 0x30000000
#define BADGE_C2V 0x22
#define BADGE_D2V 0x44


#define LINE_INDEX(a) (a >> CONFIG_L1_CACHE_LINE_SIZE_BITS)

static inline void *sddf_memcpy(void *dest, const void *src, size_t n)
{
    unsigned char *to = dest;
    const unsigned char *from = src;
    while (n-- > 0) {
        *to++ = *from++;
    }
    return dest;
}

/*
 * Cleans from start to end. This is not inclusive.
 * If end is on a cache line boundary, the cache line starting at end
 * will not be cleanend.
 *
 * On ARM, this operation ultimately performs the 'dc cvac' instruction.
 * On RISC-V, this is a no-op.
 */
void cache_clean(unsigned long start, unsigned long end)
{
#ifdef CONFIG_ARCH_AARCH64
    unsigned long vaddr;
    unsigned long index;

    assert(start != end);

    /* If the end address is not on a cache line boundary, we want to perform
     * the cache operation on that cache line as well. */
    unsigned long end_rounded = ROUND_UP(end, 1 << CONFIG_L1_CACHE_LINE_SIZE_BITS);

    for (index = LINE_INDEX(start); index < LINE_INDEX(end_rounded); index++) {
        vaddr = index << CONFIG_L1_CACHE_LINE_SIZE_BITS;
        asm volatile("dc cvac, %0" : : "r"(vaddr));
        asm volatile("dmb sy" ::: "memory");
    }
#endif
}

static void request_mbr(blk_virt_boot_info_t *boot_info)
{
    int err = 0;
    uintptr_t mbr_paddr = boot_info->driver_data_paddr;
    uintptr_t mbr_vaddr = boot_info->driver_data_vaddr;

    uint32_t mbr_req_id = 0;
    err = ialloc_alloc(&ialloc, &mbr_req_id);
    assert(!err);
    reqsbk[mbr_req_id] = (reqbk_t) { 0, 0, mbr_vaddr, 1, 0 };

    /* Virt-to-driver data region needs to be big enough to transfer MBR data */
    assert(boot_info->client.data.region.size >= BLK_TRANSFER_SIZE);

    err = blk_enqueue_req(&blk_v2d_queue, BLK_REQ_READ, mbr_paddr, 0, 1, mbr_req_id);
    assert(!err);
    ZF_LOGI("SSSSSSSSSSSSSSS    Virt signal to driver    SSSSSSSSSSSSSSSSSSSSSS\n");
    seL4_Signal(boot_info->driver_nt);
    // microkit_deferred_notify(config.driver.conn.id);
}

/* MS-DOS Master boot record */
struct msdos_mbr msdos_mbr;

static bool handle_mbr_reply()
{
    int err = 0;
    if (blk_queue_empty_resp(&blk_v2d_queue)) {
        ZF_LOGI(
            "Notified by driver but queue is empty, expecting a response to a BLK_REQ_READ request into sector 0\n");
        return false;
    }

    blk_resp_status_t drv_status;
    uint16_t drv_success_count;
    uint32_t drv_resp_id;
    err = blk_dequeue_resp(&blk_v2d_queue, &drv_status, &drv_success_count, &drv_resp_id);
    assert(!err);

    reqbk_t mbr_bk = reqsbk[drv_resp_id];
    err = ialloc_free(&ialloc, drv_resp_id);
    assert(!err);

    if (drv_status != BLK_RESP_OK) {
        ZF_LOGE("Failed to read sector 0 from driver\n");
        return false;
    }

    /* TODO: This is a raw seL4 system call because Microkit does not (currently)
     * include a corresponding libmicrokit API. */
#ifdef CONFIG_ARCH_ARM
    seL4_ARM_VSpace_Invalidate_Data(3, mbr_bk.vaddr, mbr_bk.vaddr + (BLK_TRANSFER_SIZE * mbr_bk.count));
#endif
    sddf_memcpy(&msdos_mbr, (void *)mbr_bk.vaddr, sizeof(struct msdos_mbr));

    return true;
}

static void partitions_dump()
{
    ZF_LOGI("the following partitions exist:\n");
    for (int i = 0; i < MSDOS_MBR_MAX_PRIMARY_PARTITIONS; i++) {
        ZF_LOGI("partition %d: type: 0x%hhx", i, msdos_mbr.partitions[i].type);
        if (msdos_mbr.partitions[i].type == MSDOS_MBR_PARTITION_TYPE_EMPTY) {
            ZF_LOGI(" (empty)\n");
        } else {
            ZF_LOGI("\n");
        }
    }
}


static void partitions_init(blk_virt_boot_info_t* bootinfo)
{
    if (msdos_mbr.signature != MSDOS_MBR_SIGNATURE) {
        ZF_LOGE("Invalid MBR signature\n");
        return;
    }
    ZF_LOGE("In partitions init\n");
    /* Validate partition and assign to client */
    blk_virt_config_client_t *c_client = &bootinfo->client;
    size_t client_partition = c_client->partition;

    if (client_partition >= MSDOS_MBR_MAX_PRIMARY_PARTITIONS
        || msdos_mbr.partitions[client_partition].type == MSDOS_MBR_PARTITION_TYPE_EMPTY) {
        /* Partition does not exist */
        ZF_LOGE("Invalid client partition mapping for client, partition: %zu, partition does not exist\n", client_partition);
        partitions_dump();
        return;
    }

    if (msdos_mbr.partitions[client_partition].lba_start % (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE) != 0) {
        /* Partition start sector is not aligned to sDDF transfer size */
        ZF_LOGE("Partition %d start sector %d not aligned to sDDF transfer size\n", (int)client_partition,
                            msdos_mbr.partitions[client_partition].lba_start);
        return;
    }
    ZF_LOGE("We have a valid partition now\n");
    /* We have a valid partition now. */
    client.start_sector = msdos_mbr.partitions[client_partition].lba_start;
    client.sectors = msdos_mbr.partitions[client_partition].sectors;
    ZF_LOGE("Client set done.\n");
    ZF_LOGE("Test c_client->conn.storage_info.vaddr: %d.\n", c_client->conn.storage_info.vaddr);
    blk_storage_info_t *client_storage_info = c_client->conn.storage_info.vaddr;
    blk_storage_info_t *driver_storage_info = bootinfo->driver_storage_info_vaddr;
    ZF_LOGE("Assign storage info done.\n");
    ZF_LOGE("Test driver_storage_info->sector_size: %d.\n", driver_storage_info->sector_size);
    ZF_LOGE("Test client_storage_info->sector_size: %d.\n", client_storage_info->sector_size);
    client_storage_info->sector_size = driver_storage_info->sector_size;
    ZF_LOGE("Assign storage info done1.\n");
    client_storage_info->capacity = client.sectors / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
    ZF_LOGE("Assign storage info done2.\n");
    client_storage_info->read_only = false;
    ZF_LOGE("!!!!!!!     After Client storage info init     !!!!!!!\n");
    __atomic_store_n(&client_storage_info->ready, true, __ATOMIC_RELEASE);
}

static void handle_driver(blk_virt_boot_info_t* bootinfo)
{
    bool client_notify = false;
    blk_resp_status_t drv_status = 0;
    uint16_t drv_success_count = 0;
    uint32_t drv_resp_id = 0;

    int err = 0;
    while (!blk_queue_empty_resp(&blk_v2d_queue)) {
        err = blk_dequeue_resp(&blk_v2d_queue, &drv_status, &drv_success_count, &drv_resp_id);
        assert(!err);

        reqbk_t reqbk = reqsbk[drv_resp_id];
        err = ialloc_free(&ialloc, drv_resp_id);
        assert(!err);

        switch (reqbk.code) {
        case BLK_REQ_READ:
            if (drv_status == BLK_RESP_OK) {
                /* Invalidate cache */
                /* TODO: This is a raw seL4 system call because Microkit does not (currently)
                    * include a corresponding libmicrokit API. */
    #ifdef CONFIG_ARCH_ARM
                seL4_ARM_VSpace_Invalidate_Data(3, reqbk.vaddr, reqbk.vaddr + (BLK_TRANSFER_SIZE * reqbk.count));
    #endif
            }
            break;
        case BLK_REQ_WRITE:
        case BLK_REQ_FLUSH:
        case BLK_REQ_BARRIER:
            break;
        default:
            /* This should never happen as we will have sanitized request codes before they are bookkept */
            ZF_LOGE("bookkept client %d request code %d is invalid, this should never happen\n", reqbk.cli_id,
                             reqbk.code);
            assert(false);
        }

        blk_queue_handle_t h = client.queue_h;

        /* Response queue should never be full since number of inflight requests (ialloc size)
         * should always be less than or equal to resp queue capacity.
         */
        err = blk_enqueue_resp(&h, drv_status, drv_success_count, reqbk.cli_req_id);
        assert(!err);
        client_notify = true;
    }

    /* Notify corresponding client if a response was enqueued */
    if (client_notify) {
        // microkit_notify(config.clients[i].conn.id);
        ZF_LOGI("Virt signal to client\n");
        seL4_Signal(bootinfo->client_nt);
    }
}

static bool handle_client(blk_virt_boot_info_t* bootinfo)
{
    int err = 0;
    blk_queue_handle_t h = client.queue_h;
    uintptr_t cli_data_base_paddr = bootinfo->client.data.io_addr;
    uintptr_t cli_data_base_vaddr = bootinfo->client.data.region.vaddr;
    uint64_t cli_data_region_size = bootinfo->client.data.region.size;;
    ZF_LOGI("In handle client, paddr: %lu, vaddr: %lu, region_size: %lu", cli_data_base_paddr, cli_data_base_vaddr, cli_data_region_size);

    blk_req_code_t cli_code = 0;
    uintptr_t cli_offset = 0;
    uint64_t cli_block_number = 0;
    uint16_t cli_count = 0;
    uint32_t cli_req_id = 0;

    bool driver_notify = false;
    bool client_notify = false;
    /*
     * In addition to checking the client actually has a request, we check that the
     * we can enqueue the request into the driver as well as that our index state tracking
     * is not full. We check the index allocator as there can be more in-flight requests
     * than currently in the driver queue.
     */

    while (!blk_queue_empty_req(&h) && !blk_queue_full_req(&blk_v2d_queue) && !ialloc_full(&ialloc)) {
        err = blk_dequeue_req(&h, &cli_code, &cli_offset, &cli_block_number, &cli_count, &cli_req_id);
        assert(!err);

        uint64_t drv_block_number = 0;
        drv_block_number = cli_block_number + (client.start_sector / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE));

        blk_resp_status_t resp_status = BLK_RESP_ERR_UNSPEC;
        switch (cli_code) {
        case BLK_REQ_READ:
        case BLK_REQ_WRITE: {
            unsigned long client_sectors = client.sectors / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
            unsigned long client_start_sector = client.start_sector / (BLK_TRANSFER_SIZE / MSDOS_MBR_SECTOR_SIZE);
            if (drv_block_number < client_start_sector || drv_block_number + cli_count > client_start_sector + client_sectors) {
                /* Requested block number out of bounds */
                ZF_LOGE("client request for block %lu is out of bounds\n", cli_block_number);
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            if ((cli_offset + BLK_TRANSFER_SIZE * cli_count) > cli_data_region_size) {
                /* Requested offset is out of bounds from client data region */
                ZF_LOGE("client request offset 0x%lx is invalid\n", cli_offset);
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            if (cli_count == 0) {
                ZF_LOGE("client requested zero blocks\n");
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }

            if ((cli_data_base_vaddr + cli_offset) % BLK_TRANSFER_SIZE != 0) {
                ZF_LOGE(
                    "client requested dma address is not aligned to page size (same as blk transfer size)\n");
                resp_status = BLK_RESP_ERR_INVALID_PARAM;
                goto req_fail;
            }
            
            break;
        }
        case BLK_REQ_FLUSH:
        case BLK_REQ_BARRIER:
            break;
        default:
            /* Invalid request code given */
            ZF_LOGE("client gave an invalid request code %d\n", cli_code);
            resp_status = BLK_RESP_ERR_INVALID_PARAM;
            goto req_fail;
        }

        if (cli_code == BLK_REQ_WRITE) {
            cache_clean(cli_data_base_vaddr + cli_offset,
                        cli_data_base_vaddr + cli_offset + (BLK_TRANSFER_SIZE * cli_count));
        }

        /* Bookkeep client request and generate driver req id */
        uint32_t drv_req_id = 0;
        err = ialloc_alloc(&ialloc, &drv_req_id);
        assert(!err);
        reqsbk[drv_req_id] = (reqbk_t) { 0, cli_req_id, cli_data_base_vaddr + cli_offset, cli_count, cli_code };

        err = blk_enqueue_req(&blk_v2d_queue, cli_code, cli_data_base_paddr + cli_offset, drv_block_number, cli_count,
                              drv_req_id);

        assert(!err);
        driver_notify = true;
        continue;

    req_fail:
        /* Response queue should never be full since number of inflight requests (ialloc size)
         * should always be less than or equal to resp queue capacity.
         */
        err = blk_enqueue_resp(&h, resp_status, 0, cli_req_id);
        assert(!err);
        client_notify = true;
    }

    if (client_notify) {
        ZF_LOGI("Virt signal to client\n");
        seL4_Signal(bootinfo->client_nt);
        // microkit_notify(config.clients[cli_id].conn.id);
    }

    return driver_notify;
}

static void handle_clients(blk_virt_boot_info_t* bootinfo)
{
    ZF_LOGI("In handle_clients\n");
    bool driver_notify = false;
    if (handle_client(bootinfo)) {
        driver_notify = true;
    }

    if (driver_notify) {
        ZF_LOGI("SSSSSSSSSSSSSSS    Virt signal to driver    SSSSSSSSSSSSSSSSSSSSSS\n");
        seL4_Signal(bootinfo->driver_nt);
        // microkit_notify(config.driver.conn.id);
    }
}

/* The virtualiser is not initialised until we can read the MBR and populate the block device configuration. */
bool initialised = false;

void notified_virt(blk_virt_boot_info_t* bootinfo)
{
    ZF_LOGI("In Virt Notified\n");
    int count = 0;
    while (1) {
        ZF_LOGI("In While count: %d\n", count);
        seL4_Word badge;
        seL4_Wait(bootinfo->virt_nt, &badge);
        ZF_LOGI("After wait badge: %lx, count: %d\n", badge, count);
        if (initialised == false) {
            ZF_LOGI("Initialised == false, before handle_mbr_reply\n");
            bool success = handle_mbr_reply();
            ZF_LOGI("Initialised == false, after handle_mbr_reply, success: %d\n", success);
            if (success) {
                partitions_init(bootinfo);
                initialised = true;
            };
            continue;
        }
        if (badge & BADGE_D2V) {
            handle_driver(bootinfo);
            handle_clients(bootinfo); 
        }
        if (badge & BADGE_C2V) {
            handle_clients(bootinfo); 
        }
        count ++;
    }
}

int main(int argc, char *argv[]) {
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");

    blk_virt_boot_info_t *boot_info = (blk_virt_boot_info_t *)BOOTINFO_VADDR;


    ZF_LOGI("After init bootinfo, %p \n", boot_info);
    blk_storage_info_t *driver_storage_info = boot_info->driver_storage_info_vaddr;
    ZF_LOGI("Before while \n");
    while (!blk_storage_is_ready(driver_storage_info));
    ZF_LOGI("After while \n");
    /* Initialise client queues */
    blk_req_queue_t *curr_req = boot_info->client.conn.req_queue.vaddr;
    blk_resp_queue_t *curr_resp = boot_info->client.conn.resp_queue.vaddr;
    uint32_t queue_capacity = VIRTQ_NUM_REQUESTS;
    blk_queue_init(&client.queue_h, curr_req, curr_resp, queue_capacity);
    /* Initialise driver queue */
    blk_queue_init(&blk_v2d_queue, boot_info->driver_request_shmem_vaddr, boot_info->driver_response_shmem_vaddr, VIRTQ_NUM_REQUESTS);
    ZF_LOGI("After queue init \n");
    /* Initialise index allocator */
    ialloc_init(&ialloc, ialloc_idxlist, DRIVER_MAX_NUM_BUFFERS);

    request_mbr(boot_info);
    notified_virt(boot_info);
}