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
// #include <simple/simple.h>
#include <basic_data.h>
#include <virtio.h>
#include <virtio_queue.h>
#include <queue.h>
#include <ialloc.h>
#include <block.h>
#include <storage_info.h>
#include <msdos_mbr.h>
#include <simple-default/simple-default.h>
#include "../test.h"
#include "../helpers.h"

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE
#define SERSERV_TEST_ALLOCMAN_PREALLOCATED_MEMSIZE (64 * 1024)
#define SHMEM_DRIVER_KB 4096

#define ALIGN(x, align)   (((x) + (align) - 1) & ~((align) - 1))

#define VIRT_IRQ 0x1
#define VIRT_NOTIFICATION 0x2
#define BADGE_V2C 0x11
#define BADGE_C2V 0x22
#define BADGE_D2V 0x44
#define BADGE_V2D 0x88


volatile struct virtio_blk_config *virtio_config;

/*
* A mapping from virtIO header index in the descriptor virtq ring, to the sDDF ID given
* in the request. We need this mapping due to out of order operations.
*/
uint32_t virtio_header_to_id[QUEUE_SIZE];

uintptr_t virtio_headers_paddr;
struct virtio_blk_req *virtio_headers;
static volatile virtio_mmio_regs_t *regs;
uintptr_t requests_paddr;
uintptr_t requests_vaddr;

blk_queue_handle_t blk_queue;
blk_queue_handle_t blk_c2v_queue;
blk_queue_handle_t blk_v2c_queue;
blk_queue_handle_t blk_v2d_queue;

volatile struct virtq virtq;

/*
* Due to the out-of-order nature of virtIO, we need a way of allocating indexes in a
* non-linear way.
*/
ialloc_t ialloc_desc;
uint32_t descriptors[QUEUE_SIZE];

uint16_t last_seen_used = 0;

// blk driver process
static helper_thread_t blk_driver_thread;
// client process
static helper_thread_t blk_client_thread;
// virt process
static helper_thread_t blk_virt_thread;

typedef struct blk_driver_boot_info {
    uintptr_t regs_vaddr;
    uintptr_t regs_paddr;
    uintptr_t headers_vaddr;
    uintptr_t headers_paddr;

    uintptr_t meta_vaddr;
    uintptr_t meta_paddr;

    uintptr_t storage_info_vaddr;
    uintptr_t request_shmem_vaddr;
    uintptr_t request_paddr;
    uintptr_t response_shmem_vaddr;
    uintptr_t response_paddr;

    // notification
    seL4_CPtr badged_nt;
    seL4_CPtr irq_cap;

    seL4_CPtr driver_nt;
    seL4_CPtr virt_nt;
    seL4_CPtr driver_RECV_nt;

} blk_driver_boot_info_t;

typedef struct blk_client_boot_info {
    uintptr_t storage_info_vaddr;
    uintptr_t request_shmem_vaddr;
    uintptr_t request_paddr;
    uintptr_t response_shmem_vaddr;
    uintptr_t response_paddr;
    uintptr_t client_data_vaddr;
    uintptr_t client_data_paddr;

    // notification
    seL4_CPtr virt_nt;
    seL4_CPtr client_nt;

} blk_client_boot_info_t;

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


static size_t bytes_to_size_bits(size_t size_bytes) {
    assert(size_bytes > 0);
    ZF_LOGI("In bytes_to_size_bits, bytes: %d\n", size_bytes);
    int bits = 0;
    size_t tmp = size_bytes;

    if ((tmp & (tmp - 1)) != 0) {
        tmp--;
        while (tmp > 0) {
            tmp >>= 1;
            bits++;
        }
        bits++; 
    } else {
        while (tmp > 1) {
            tmp >>= 1;
            bits++;
        }
    }

    return bits;
}

#define LINE_INDEX(a) (a >> CONFIG_L1_CACHE_LINE_SIZE_BITS)

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

// alloc frame
static void map_and_share_frame(struct env *env, sel4utils_process_t target_process1, sel4utils_process_t target_process2, 
        uintptr_t vaddr1, uintptr_t vaddr2, size_t size_bytes, void** paddr, int cached) 
{
    // ZF_LOGI("cached: %d\n", cached);
    seL4_Error err;
    // ZF_LOGI("In map_and_share_frame2, bytes: %d, vaddr1: %p, vaddr2: %p\n", size_bytes, vaddr1, vaddr2);
        size_t size_bits = bytes_to_size_bits(size_bytes);
    // ZF_LOGI("In map_and_share_frame2 after reservation, bits: %d\n", size_bits);
    // get frame cap
    vka_object_t result;
    err = vka_alloc_untyped(&env->vka, size_bits, &result);
    assert(err == seL4_NoError);

    // ZF_LOGI("In map_and_share_frame2 after alloc untyped\n");
    size_t num_pages = (size_bytes + BIT(12) - 1) / BIT(12);
    // ZF_LOGI("In 2 num page: %d\n", num_pages);
    seL4_CPtr frame_caps[num_pages];
    for (int i = 0; i < num_pages; i++) {
        seL4_Word slot = get_free_slot(env);
        cspacepath_t dest_path;
        vka_cspace_make_path(&env->vka, slot, &dest_path);
        frame_caps[i] = dest_path.capPtr;
        err = vka_untyped_retype(&result, seL4_ARCH_4KPage, seL4_PageBits, 1, &dest_path);
        assert(err == seL4_NoError);
    }

    seL4_ARM_Page_GetAddress_t res = seL4_ARM_Page_GetAddress(frame_caps[0]);
    *paddr = (void*)res.paddr;
    ZF_LOGI("get physical address: %p\n", *paddr);

    uintptr_t vaddr;
    reservation_t reservation = vspace_reserve_range(&env->vspace, size_bytes, seL4_ReadWrite, cached, (void **)&vaddr);
    assert(reservation.res != NULL);
    err = vspace_map_pages_at_vaddr(&env->vspace, frame_caps, NULL, (void *)vaddr, num_pages, seL4_PageBits, reservation);
    // ZF_LOGI("After vspace_map_pages_at_vaddr return error: %d\n",err);
    assert(err == seL4_NoError);

    if (vaddr1 != 0) {
        reservation_t reservation1 = vspace_reserve_range_at(&target_process1.vspace,(void *)vaddr1, size_bytes, seL4_ReadWrite, cached);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process1.vspace, (void *)vaddr, num_pages, seL4_PageBits, (void *)vaddr1, reservation1);
        assert(err == seL4_NoError);
    }
    if (vaddr2 != 0) {
        reservation_t reservation2 = vspace_reserve_range_at(&target_process2.vspace,(void *)vaddr2, size_bytes, seL4_ReadWrite, cached);
        assert(err == seL4_NoError);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process2.vspace, (void *)vaddr, num_pages, seL4_PageBits, (void *)vaddr2, reservation2);
        // ZF_LOGI("After sel4utils_share_mem_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
    }
    // ZF_LOGI("After create reservations\n");
}


static void map_and_share_frame2(struct env *env, sel4utils_process_t target_process1, sel4utils_process_t target_process2, 
        uintptr_t vaddr1, uintptr_t vaddr2, size_t size_bytes, uintptr_t paddr, int cached)
{
    // ZF_LOGI("cached: %d\n", cached);
    seL4_Error err;
    // ZF_LOGI("In map_and_share_frame2, bytes: %d, vaddr1: %p, vaddr2: %p\n", size_bytes, vaddr1, vaddr2);
    size_t size_bits = bytes_to_size_bits(size_bytes);
    // ZF_LOGI("In map_and_share_frame2 after reservation, bits: %d\n", size_bits);
    // get frame cap
    vka_object_t result;
    if (paddr == 0) {
        err = vka_alloc_untyped(&env->vka, size_bits, &result);
    }
    else {
        // err = vka_alloc_untyped(&env->vka, size_bits, &result);
        // seL4_ARM_Page_GetAddress_t res = seL4_ARM_Page_GetAddress(result.cptr);
        // ZF_LOGI("After alloc untyped, %p\n", res.paddr);
        err = vka_alloc_untyped_at(&env->vka, size_bits, paddr, &result);
    }
    assert(err == seL4_NoError);
    // ZF_LOGI("In map_and_share_frame2 after alloc untyped\n");
    size_t num_pages = (size_bytes + BIT(12) - 1) / BIT(12);
    // ZF_LOGI("In 2 num page: %d\n", num_pages);
    seL4_CPtr frame_caps[num_pages];
    for (int i = 0; i < num_pages; i++) {
        seL4_Word slot = get_free_slot(env);
        cspacepath_t dest_path;
        vka_cspace_make_path(&env->vka, slot, &dest_path);
        frame_caps[i] = dest_path.capPtr;
        err = vka_untyped_retype(&result, seL4_ARCH_4KPage, seL4_PageBits, 1, &dest_path);
        assert(err == seL4_NoError);
    }
    // ZF_LOGI("In map_and_share_frame2 after untyped retype, frame capptr[0]: %p\n", frame_caps[0]);
    uintptr_t vaddr;
    reservation_t reservation = vspace_reserve_range(&env->vspace, size_bytes, seL4_ReadWrite, cached, (void **)&vaddr);
    assert(reservation.res != NULL);
    err = vspace_map_pages_at_vaddr(&env->vspace, frame_caps, NULL, (void *)vaddr, num_pages, seL4_PageBits, reservation);
    // ZF_LOGI("After vspace_map_pages_at_vaddr return error: %d\n",err);
    assert(err == seL4_NoError);

    if (vaddr1 != 0) {
        reservation_t reservation1 = vspace_reserve_range_at(&target_process1.vspace,(void *)vaddr1, size_bytes, seL4_ReadWrite, cached);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process1.vspace, (void *)vaddr, num_pages, seL4_PageBits, (void *)vaddr1, reservation1);
        assert(err == seL4_NoError);
    }
    if (vaddr2 != 0) {
        reservation_t reservation2 = vspace_reserve_range_at(&target_process2.vspace,(void *)vaddr2, size_bytes, seL4_ReadWrite, cached);
        assert(err == seL4_NoError);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process2.vspace, (void *)vaddr, num_pages, seL4_PageBits, (void *)vaddr2, reservation2);
        // ZF_LOGI("After sel4utils_share_mem_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
    }
    // ZF_LOGI("After create reservations\n");
}

static void setupMMIO_regs_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes= 0x1000;
    ZF_LOGI("In setupMMIO_regs_shmem: %d\n", size_bytes);
    void *mapped_vaddr = (void *)bootinfo->regs_vaddr;

    reservation_t res = vspace_reserve_range_at(&env->vspace, mapped_vaddr, size_bytes, seL4_ReadWrite, 0);
    assert(res.res != NULL);

    vka_object_t frame;
    seL4_Error err = vka_alloc_frame_at(&env->vka, seL4_PageBits, bootinfo->regs_paddr, &frame);
    assert(err == seL4_NoError);

    err = vspace_map_pages_at_vaddr(&env->vspace, &frame.cptr, NULL, mapped_vaddr, 1, seL4_PageBits, res);
    assert(err == seL4_NoError);

    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, mapped_vaddr, 1, seL4_PageBits, mapped_vaddr, res);
    assert(err == seL4_NoError);
 }


static void setupMMIO_virtio_headers_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x10000;
    ZF_LOGI("In setupMMIO_virtio_headers_shmem: %d\n", size_bytes);
    void* paddr;
    map_and_share_frame(env, target_process, target_process, bootinfo->headers_vaddr, 0, size_bytes, &paddr, 0);
    bootinfo->headers_paddr = paddr;
}

static void setupMMIO_virtio_metadata_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x200000;
    ZF_LOGI("In setupMMIO_virtio_metadata_shmem: %d\n", size_bytes);
    void* paddr;
    map_and_share_frame(env, target_process, target_process, bootinfo->meta_vaddr, 0, size_bytes, &paddr, 0);
    bootinfo->meta_paddr = paddr;
}

static void setup_driver_storage_info_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x1000;
    ZF_LOGI("In setup_driver_storage_info_shmem: %d\n", size_bytes);

    void *mapped_vaddr = (void *)bootinfo->storage_info_vaddr;
    reservation_t res = vspace_reserve_range_at(&env->vspace, mapped_vaddr, size_bytes, seL4_ReadWrite, 1);
    assert(res.res != NULL);

    vka_object_t frame;
    seL4_Error err = vka_alloc_frame(&env->vka, seL4_PageBits, &frame);
    assert(err == seL4_NoError);

    err = vspace_map_pages_at_vaddr(&env->vspace, &frame.cptr, NULL, mapped_vaddr, 1, seL4_PageBits, res);
    assert(err == seL4_NoError);

    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, mapped_vaddr, 1, seL4_PageBits, mapped_vaddr, res);
    assert(err == seL4_NoError);
}

void notified(blk_driver_boot_info_t *boot_info)
{
    ZF_LOGI("In Driver Notified\n");
    volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;
    int count = 0;
    while (1) {
        ZF_LOGI("count: %d\n", count);
        seL4_Word badge = 0;
        ZF_LOGI("Waiting ... ...\n");
        seL4_Wait(boot_info->badged_nt, &badge);
        ZF_LOGI("After wait %lx, count: %d\n", badge, count);
        if (badge & VIRT_IRQ) {
            ZF_LOGI("Get IRQ NOTIFICATION from device\n");
            handle_irq(boot_info);
            handle_request(boot_info);
            /* Ack */
            seL4_IRQHandler_Ack(boot_info->irq_cap);
        }
        if (badge & BADGE_V2D) {
            ZF_LOGI("Get NOTIFICATION from VIRT\n");
            handle_request(boot_info);
        }
        if (!(badge & (VIRT_IRQ | BADGE_V2D))) {
            ZF_LOGE("received notification from unknown channel: 0x%x\n", badge);
        }
        count ++;
    }
}

void handle_irq(blk_driver_boot_info_t *boot_info)
{
    ZF_LOGI("In IRQ handler\n");
    volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;
    uint32_t irq_status = regs->InterruptStatus;
    if (irq_status & VIRTIO_MMIO_IRQ_VQUEUE) {
        handle_response(boot_info);
        regs->InterruptACK = VIRTIO_MMIO_IRQ_VQUEUE;
    }

    if (irq_status & VIRTIO_MMIO_IRQ_CONFIG) {
        ZF_LOGE("unexpected change in configuration\n");
    }
}

void handle_response(blk_driver_boot_info_t *boot_info)
{
    bool notify = false;

    uint16_t i = last_seen_used;
    uint16_t curr_idx = virtq.used->idx;
    while (i != curr_idx) {
        uint16_t virtq_idx = i % virtq.num;
        struct virtq_used_elem hdr_used = virtq.used->ring[virtq_idx];
        assert(virtq.desc[hdr_used.id].flags & VIRTQ_DESC_F_NEXT);

        struct virtq_desc hdr_desc = virtq.desc[hdr_used.id];
        LOG_DRIVER("response header addr: 0x%lx, len: %d\n", hdr_desc.addr, hdr_desc.len);

        assert(hdr_desc.len == VIRTIO_BLK_REQ_HDR_SIZE);
        struct virtio_blk_req *hdr = &virtio_headers[virtq_idx];
        virtio_blk_print_req(hdr);

        uint16_t data_desc_idx = virtq.desc[hdr_used.id].next;
        struct virtq_desc data_desc = virtq.desc[data_desc_idx % virtq.num];
        uint32_t data_len = data_desc.len;
#ifdef DEBUG_DRIVER
        uint64_t data_addr = data_desc.addr;
        ZF_LOGI("response data addr: 0x%lx, data len: %d\n", data_addr, data_len);
#endif

        uint16_t footer_desc_idx = virtq.desc[data_desc_idx].next;

        blk_resp_status_t status;
        if (hdr->status == VIRTIO_BLK_S_OK) {
            status = BLK_RESP_OK;
        } else {
            status = BLK_RESP_ERR_UNSPEC;
        }
        int err = blk_enqueue_resp(&blk_queue, status, data_len / BLK_TRANSFER_SIZE, virtio_header_to_id[hdr_used.id]);
        assert(!err);

        /* Free up the descriptors we used */
        err = ialloc_free(&ialloc_desc, hdr_used.id);
        assert(!err);
        err = ialloc_free(&ialloc_desc, data_desc_idx);
        assert(!err);
        err = ialloc_free(&ialloc_desc, footer_desc_idx);
        assert(!err);

        i += 1;
        notify = true;
    }

    if (notify) {
        // microkit_notify(config.virt.id);
        ZF_LOGI("Driver signal to vrit\n");
        seL4_Signal(boot_info->virt_nt);
    }

    last_seen_used = i;
}

void handle_request(blk_driver_boot_info_t *boot_info)
{
    volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;
    /* Whether or not we notify the virtIO device to say something has changed
     * in the virtq. */
    bool virtio_queue_notify = false;
    /* Consume all requests and put them in the 'avail' ring of the virtq. We do not
     * dequeue unless we know we can put the request in the virtq. */
    while (!blk_queue_empty_req(&blk_queue) && ialloc_num_free(&ialloc_desc) >= 3) {
        blk_req_code_t req_code;
        uintptr_t phys_addr;
        uint64_t block_number;
        uint16_t count;
        uint32_t id;
        int err = blk_dequeue_req(&blk_queue, &req_code, &phys_addr, &block_number, &count, &id);
        assert(!err);

        /*
         * The block size sDDF expects is different to virtIO, so we must first convert the request
         * parameters to virtIO.
         */
        assert(BLK_TRANSFER_SIZE >= VIRTIO_BLK_SECTOR_SIZE);
        size_t virtio_block_number = block_number * (BLK_TRANSFER_SIZE / VIRTIO_BLK_SECTOR_SIZE);
        size_t virtio_count = count * (BLK_TRANSFER_SIZE / VIRTIO_BLK_SECTOR_SIZE);
        switch (req_code) {
        case BLK_REQ_READ:
        case BLK_REQ_WRITE: {
            /*
             * Read and write requests are almost identical with virtIO so we combine them
             * here to save a bit of code duplication.
             * Each sDDF read/write is split into three virtIO descriptors:
             *     * header
             *     * data
             *     * footer (status field of the header)
             *
             * This is a bit weird, but the reason it is done is so that we do not have to do
             * any copying to/from the sDDF data region. The 'data' is expected between some of
             * the fields in the header and so we have one descriptor for all the fields, then
             * a 'footer' descriptor with the single remaining field of the header
             * (the status field).
             *
             */

            /* It is the responsibility of the virtualiser to check that the request is valid,
             * so we just assert that the block number and count do not exceed the capacity. */
            assert(virtio_block_number + virtio_count <= virtio_config->capacity);

            if (req_code == BLK_REQ_READ) {
                ZF_LOGI("handling read request with physical address 0x%lx, block_number: 0x%x, count: 0x%x, id: 0x%x\n",
                           phys_addr, block_number, count, id);
            } else {
                ZF_LOGI("handling write request with physical address 0x%lx, block_number: 0x%x, count: 0x%x, id: 0x%x\n",
                           phys_addr, block_number, count, id);
            }

            uint32_t hdr_desc_idx = -1;
            uint32_t data_desc_idx = -1;
            uint32_t footer_desc_idx = -1;

            int err;
            err = ialloc_alloc(&ialloc_desc, &hdr_desc_idx);
            assert(!err && hdr_desc_idx != -1);
            err = ialloc_alloc(&ialloc_desc, &data_desc_idx);
            assert(!err && data_desc_idx != -1);
            err = ialloc_alloc(&ialloc_desc, &footer_desc_idx);
            assert(!err && footer_desc_idx != -1);

            uint16_t data_flags = VIRTQ_DESC_F_NEXT;
            uint16_t type;
            if (req_code == BLK_REQ_READ) {
                type = VIRTIO_BLK_T_IN;
                /* Doing a read request, so device needs to be able to write into the DMA region. */
                data_flags |= VIRTQ_DESC_F_WRITE;
            } else {
                type = VIRTIO_BLK_T_OUT;
            }

            struct virtio_blk_req *hdr = &virtio_headers[hdr_desc_idx];
            hdr->type = type;
            hdr->sector = virtio_block_number;

            virtq.desc[hdr_desc_idx] = (struct virtq_desc) {
                .addr = virtio_headers_paddr + (hdr_desc_idx * sizeof(struct virtio_blk_req)),
                .len = VIRTIO_BLK_REQ_HDR_SIZE,
                .flags = VIRTQ_DESC_F_NEXT,
                .next = data_desc_idx,
            };

            virtq.desc[data_desc_idx] = (struct virtq_desc) {
                .addr = phys_addr,
                .len = VIRTIO_BLK_SECTOR_SIZE * virtio_count,
                .flags = data_flags,
                .next = footer_desc_idx,
            };

            virtq.desc[footer_desc_idx] = (struct virtq_desc) {
                .addr = virtq.desc[hdr_desc_idx].addr + VIRTIO_BLK_REQ_HDR_SIZE,
                .len = 1,
                .flags = VIRTQ_DESC_F_WRITE,
            };

            virtq.avail->ring[virtq.avail->idx % virtq.num] = hdr_desc_idx;
            virtq.avail->idx++;


            // debug
            ZF_LOGI("Descriptor hdr: idx=%d, addr=0x%lx, len=%u, flags=0x%x, next=%u",
                    hdr_desc_idx,
                    virtq.desc[hdr_desc_idx].addr,
                    virtq.desc[hdr_desc_idx].len,
                    virtq.desc[hdr_desc_idx].flags,
                    virtq.desc[hdr_desc_idx].next);

            ZF_LOGI("Descriptor data: idx=%d, addr=0x%lx, len=%u, flags=0x%x, next=%u",
                    data_desc_idx,
                    virtq.desc[data_desc_idx].addr,
                    virtq.desc[data_desc_idx].len,
                    virtq.desc[data_desc_idx].flags,
                    virtq.desc[data_desc_idx].next);

            ZF_LOGI("Descriptor footer: idx=%d, addr=0x%lx, len=%u, flags=0x%x",
                    footer_desc_idx,
                    virtq.desc[footer_desc_idx].addr,
                    virtq.desc[footer_desc_idx].len,
                    virtq.desc[footer_desc_idx].flags);

            virtio_queue_notify = true;
            virtio_header_to_id[hdr_desc_idx] = id;

            break;
        }
        case BLK_REQ_FLUSH: {
            int err = blk_enqueue_resp(&blk_queue, BLK_RESP_OK, 0, id);
            assert(!err);
            // microkit_notify(config.virt.id);
            ZF_LOGI("Driver signal to vrit\n");
            seL4_Signal(boot_info->virt_nt);
            break;
        }
        case BLK_REQ_BARRIER: {
            int err = blk_enqueue_resp(&blk_queue, BLK_RESP_OK, 0, id);
            assert(!err);
            //microkit_notify(config.virt.id);
            ZF_LOGI("Driver signal to vrit\n");
            seL4_Signal(boot_info->virt_nt);
            break;
        }
        default:
            /* The virtualiser should have sanitised the request code and so we should never get here. */
            ZF_LOGI("unsupported request code: 0x%x\n", req_code);
            break;
        }
    }

    if (virtio_queue_notify) {
        ZF_LOGI("### Avail idx = %d ###", virtq.avail->idx);
        for (int i = 0; i < virtq.avail->idx; i++) {
            uint16_t idx = virtq.avail->ring[i % virtq.num];
            ZF_LOGI("Descriptor %d: addr=0x%lx, len=%u, flags=0x%x, next=%u",
                idx,
                virtq.desc[idx].addr,
                virtq.desc[idx].len,
                virtq.desc[idx].flags,
                virtq.desc[idx].next);
        }
        ZF_LOGI("Before doorbell\n");
        regs->QueueNotify = 0;

        ZF_LOGI("check setting, regs->QueueReady: %d\n", regs->QueueReady);
        ZF_LOGI("check setting, virtq.avail->idx = %u", virtq.avail->idx);
        for (volatile int i = 0; i < 200000; i++) {
            if (virtq.used->idx) {
                ZF_LOGI("used->idx = %u\n", virtq.used->idx);
                break;
            }
        }
        ZF_LOGI("Doorbell rung, InterruptStatus=0x%x", regs->InterruptStatus);
    }
}

static int blk_driver_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_driver_boot_info_t *boot_info = (blk_driver_boot_info_t *)_bootinfo;

    virtio_headers_paddr = boot_info->headers_paddr;
    virtio_headers = (struct virtio_blk_req *) boot_info->headers_vaddr;
    /* Block device configuration, populated during initiliastion. */
    regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;
    requests_paddr = boot_info->meta_paddr;
    requests_vaddr = boot_info->meta_vaddr;
    for (int i = 0; i < 8; i++) {
        volatile virtio_mmio_regs_t *regs_tmp = (volatile virtio_mmio_regs_t *) (boot_info->regs_vaddr + i*0x200);
        // ZF_LOGI("for loop %p\n", regs_tmp);
        if (virtio_mmio_check_magic(regs_tmp) && virtio_mmio_version(regs_tmp) == VIRTIO_VERSION &&
                virtio_mmio_check_device_id(regs_tmp, VIRTIO_DEVICE_ID_BLK)) {
            ZF_LOGI("##$$$$$######   find blk device on %p $#$#$#$#$#$#$#$#$\n", regs_tmp);
            regs = regs_tmp;
            boot_info->regs_vaddr = regs_tmp;
        }
    }

    assert(virtio_headers_paddr);
    assert(virtio_headers);
    assert(requests_paddr);
    assert(requests_vaddr);

    virtio_blk_init(boot_info);
    blk_queue_init(&blk_queue, boot_info->request_shmem_vaddr, boot_info->response_shmem_vaddr, VIRTQ_NUM_REQUESTS);

    //notified
    notified(boot_info);
}

void virtio_blk_init(blk_driver_boot_info_t *boot_info)
    {
        ZF_LOGI("In virtio_blk_init\n");
        ZF_LOGI("request_paddr = 0x%lx, headers_paddr = 0x%lx\n",boot_info->meta_paddr, boot_info->headers_paddr);

        // ZF_LOGI("Before virtio_mmio_check_magic check, %d\n", regs->MagicValue);
        // Do MMIO device init (section 4.2.3.1)
        if (!virtio_mmio_check_magic(regs)) {
            ZF_LOGE("invalid virtIO magic value!\n");
            assert(false);
        }
    
        // ZF_LOGI("Before virtio_mmio_version check, %d\n", regs->Version);
    
        if (virtio_mmio_version(regs) != VIRTIO_VERSION) {
            ZF_LOGE("not correct virtIO version!\n");
            assert(false);
        }
        // ZF_LOGI("After virtio_mmio_version check\n");
        
        // ZF_LOGI("Before virtio_mmio_check_device_id check, %d\n", regs->DeviceID);
        if (!virtio_mmio_check_device_id(regs, VIRTIO_DEVICE_ID_BLK)) {
            ZF_LOGE("not a virtIO block device!\n");
            assert(false);
        }
        // ZF_LOGI("After virtio_mmio_check_device_id check\n");
    
        if (virtio_mmio_version(regs) != VIRTIO_BLK_DRIVER_VERSION) {
            ZF_LOGE("driver does not support given virtIO version: 0x%x\n", virtio_mmio_version(regs));
            assert(false);
        }
        // ZF_LOGI("After virtio_mmio_version2 check\n");
    
        ialloc_init(&ialloc_desc, descriptors, QUEUE_SIZE);
    
        /* First reset the device */
        regs->Status = 0;
        /* Set the ACKNOWLEDGE bit to say we have noticed the device */
        regs->Status = VIRTIO_DEVICE_STATUS_ACKNOWLEDGE;
        /* Set the DRIVER bit to say we know how to drive the device */
        regs->Status = VIRTIO_DEVICE_STATUS_DRIVER;
    
        virtio_config = (volatile struct virtio_blk_config *)regs->Config;
    #ifdef DEBUG_DRIVER
        virtio_blk_print_config(virtio_config);
    #endif
    
        if (virtio_config->capacity < BLK_TRANSFER_SIZE / VIRTIO_BLK_SECTOR_SIZE) {
            LOG_DRIVER_ERR("driver does not support device capacity smaller than 0x%x bytes"
                           " (device has capacity of 0x%lx bytes)\n",
                           BLK_TRANSFER_SIZE, virtio_config->capacity * VIRTIO_BLK_SECTOR_SIZE);
            assert(false);
        }
    
        /* This driver does not support Read-Only devices, so we always leave this as false */
        blk_storage_info_t *storage_info = boot_info->storage_info_vaddr;
        ZF_LOGI("After storage_info assigned, %p\n", storage_info);
        storage_info->read_only = false;
        storage_info->capacity = (virtio_config->capacity * VIRTIO_BLK_SECTOR_SIZE) / BLK_TRANSFER_SIZE;
        storage_info->cylinders = virtio_config->geometry.cylinders;
        storage_info->heads = virtio_config->geometry.heads;
        storage_info->blocks = virtio_config->geometry.sectors;
        storage_info->block_size = 1;
        storage_info->sector_size = VIRTIO_BLK_SECTOR_SIZE;
        ZF_LOGI("After storage_info check\n");
        /* Finished populating configuration */
        __atomic_store_n(&storage_info->ready, true, __ATOMIC_RELEASE);
    
    #ifdef DEBUG_DRIVER
        uint32_t features_low = regs->DeviceFeatures;
        regs->DeviceFeaturesSel = 1;
        uint32_t features_high = regs->DeviceFeatures;
        uint64_t features = features_low | ((uint64_t)features_high << 32);
        virtio_blk_print_features(features);
    #endif
        /* Select features we want from the device */
        regs->DriverFeatures = 0;
        regs->DriverFeaturesSel = 1;
        regs->DriverFeatures = 0;
    
        regs->Status |= VIRTIO_DEVICE_STATUS_FEATURES_OK;
        if (!(regs->Status & VIRTIO_DEVICE_STATUS_FEATURES_OK)) {
            LOG_DRIVER_ERR("device status features is not OK!\n");
            return;
        }
    
        /* Add virtqueues */
        size_t desc_off = 0;
        size_t avail_off = ALIGN(desc_off + (16 * VIRTQ_NUM_REQUESTS), 2);
        size_t used_off = ALIGN(avail_off + (6 + 2 * VIRTQ_NUM_REQUESTS), 4);
        size_t size = used_off + (6 + 8 * VIRTQ_NUM_REQUESTS);  
    
        // Make sure that the metadata region is able to fit all the virtIO specific
        // extra data.
        assert(size <= 0x200000); // hard code here, replace: device_resources.regions[2].region.size, which is metadata size.
    
        virtq.num = VIRTQ_NUM_REQUESTS;
        virtq.desc = (struct virtq_desc *)(requests_vaddr + desc_off);
        virtq.avail = (struct virtq_avail *)(requests_vaddr + avail_off);
        ZF_LOGI("&&&*&*&*&*&*&**&*  in init blk virtio, requests_vaddr: %lx, avail_off: %d\n", requests_vaddr, avail_off);
        virtq.used = (struct virtq_used *)(requests_vaddr + used_off);
        
        ZF_LOGI("desc[0].addr      = 0x%lx", virtq.desc[0].addr);
        ZF_LOGI("avail->flags      = 0x%x",   virtq.avail->flags);
        ZF_LOGI("avail->idx        = %u",     virtq.avail->idx);
        ZF_LOGI("used->flags       = 0x%x",   virtq.used->flags);
        ZF_LOGI("used->idx         = %u",     virtq.used->idx);
    
        assert(regs->QueueNumMax >= VIRTQ_NUM_REQUESTS);
        regs->QueueSel = 0;
        regs->QueueNum = VIRTQ_NUM_REQUESTS;
        regs->QueueDescLow = (requests_paddr + desc_off) & 0xFFFFFFFF;
        regs->QueueDescHigh = (requests_paddr + desc_off) >> 32;
        regs->QueueDriverLow = (requests_paddr + avail_off) & 0xFFFFFFFF;
        regs->QueueDriverHigh = (requests_paddr + avail_off) >> 32;
        regs->QueueDeviceLow = (requests_paddr + used_off) & 0xFFFFFFFF;
        regs->QueueDeviceHigh = (requests_paddr + used_off) >> 32;
        regs->QueueReady = 1;
        ZF_LOGI("After set regs->QueueReady = 1, check: %d\n", regs->QueueReady);
    
        /* Finish initialisation */
        regs->Status |= VIRTIO_DEVICE_STATUS_DRIVER_OK;
        regs->InterruptACK = VIRTIO_MMIO_IRQ_VQUEUE;
        ZF_LOGI("After QueueReady check\n");
    }



void create_and_share_boot_info(struct env *env, sel4utils_process_t target_process, void** vaddr_to_map)
{
    seL4_Error err;
    ZF_LOGI("in create_and_share_boot_info\n");
    vka_object_t frame_obj;
    err = vka_alloc_frame(&env->vka, seL4_PageBits, &frame_obj);
    assert(err == seL4_NoError);
    ZF_LOGI("After alloc frame\n");
    cspacepath_t frame_path;
    vka_cspace_make_path(&env->vka, frame_obj.cptr, &frame_path);

    reservation_t reservation = vspace_reserve_range(&env->vspace, PAGE_SIZE_4K, seL4_ReadWrite, 0, vaddr_to_map);
    assert(reservation.res != NULL);

    // mapping frame to current process
    vspace_map_pages_at_vaddr(&env->vspace, &frame_path.capPtr, NULL, *vaddr_to_map, 1, seL4_PageBits,  reservation);
    ZF_LOGI("After vspace_map_pages_at_vaddr\n");
    // mapping current vaddr to target vaddr
    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, *vaddr_to_map, 1, seL4_PageBits, *vaddr_to_map, reservation);
    assert(err == seL4_NoError);
    ZF_LOGI("After sel4utils_share_mem_at_vaddr\n");
}

static seL4_CPtr badge_endpoint(env_t env, seL4_Word badge, seL4_CPtr ep)
{
    seL4_CPtr slot = get_free_slot(env);
    int error = cnode_mint(env, ep, slot, seL4_AllRights, badge);
    test_error_eq(error, seL4_NoError);
    return slot;
}

static void init_blk_driver_server(struct env *env, struct blk_driver_boot_info *driver_bootinfo) {
    int error;
    seL4_Word prio = 200;
    ZF_LOGI("####################   in init_blk_driver_server   ##########################\n");
    driver_bootinfo->regs_vaddr = 0x20000000;
    ZF_LOGI("After init regs_vaddr\n");
    driver_bootinfo->headers_vaddr = 0x20001000;
    driver_bootinfo->meta_vaddr = 0x20200000;
    driver_bootinfo->storage_info_vaddr = 0x20400000;
    driver_bootinfo->request_shmem_vaddr = 0x20600000;
    driver_bootinfo->response_shmem_vaddr = 0x20800000;
    driver_bootinfo->regs_paddr = 0xa003000;
    ZF_LOGI("After init bootinfo\n");
    setupMMIO_regs_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setupMMIO_virtio_headers_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setupMMIO_virtio_metadata_shmem(env, blk_driver_thread.process, driver_bootinfo);
    ZF_LOGI("After init shared mem\n");
}

/* Use the start of the partition for testing. */
#define REQUEST_BLK_NUMBER 0
#define REQUEST_NUM_BLOCKS 2

enum test_basic_state {
    START,
    READ,
    FINISH,
};

enum test_basic_state test_basic_state = START;

static inline void *sddf_memcpy(void *dest, const void *src, size_t n)
{
    unsigned char *to = dest;
    const unsigned char *from = src;
    while (n-- > 0) {
        *to++ = *from++;
    }
    return dest;
}

bool test_basic(blk_client_boot_info_t *boot_info)
{
    switch (test_basic_state) {
    case START: {
        ZF_LOGI("basic: START state\n");
        // We assume that the data fits into two blocks
        assert(basic_data_len <= BLK_TRANSFER_SIZE * 2);
        // Copy our testing data into the block data region
        char *data_dest = (char *)boot_info->client_data_vaddr;
        sddf_memcpy(data_dest, basic_data, basic_data_len);

        int err = blk_enqueue_req(&blk_c2v_queue, BLK_REQ_WRITE, 0, REQUEST_BLK_NUMBER, REQUEST_NUM_BLOCKS, 0);

        assert(!err);
    
        test_basic_state = READ;

        break;
    }
    case READ: {
        ZF_LOGI("basic: READ state\n");
        /* Check that our previous write was successful */
        blk_resp_status_t status = -1;
        uint16_t count = -1;
        uint32_t id = -1;
        int err = blk_dequeue_resp(&blk_c2v_queue, &status, &count, &id);
        ZF_LOGI("basic: READ state, after blk_dequeue_resp\n");
        assert(!err);
        assert(status == BLK_RESP_OK);
        assert(count == REQUEST_NUM_BLOCKS);
        assert(id == 0);
        

        /* We do the read at a different offset into the data region from the previous request */
        uintptr_t offset = REQUEST_NUM_BLOCKS * BLK_TRANSFER_SIZE;
        err = blk_enqueue_req(&blk_c2v_queue, BLK_REQ_READ, offset, REQUEST_BLK_NUMBER, REQUEST_NUM_BLOCKS, 0);
        assert(!err);

        test_basic_state = FINISH;
        ZF_LOGI("basic: READ state before break\n");
        break;
    }
    case FINISH: {
        ZF_LOGI("basic: FINISH state\n");
        blk_resp_status_t status = -1;
        uint16_t count = -1;
        uint32_t id = -1;
        int err = blk_dequeue_resp(&blk_c2v_queue, &status, &count, &id);
        assert(!err);
        assert(status == BLK_RESP_OK);
        assert(count == REQUEST_NUM_BLOCKS);
        assert(id == 0);

        // Check that the read went okay
        char *read_data = (char *)(boot_info->client_data_vaddr + (REQUEST_NUM_BLOCKS * BLK_TRANSFER_SIZE));
        int count_mismatch = 0;
        for (int i = 0; i < basic_data_len; i++) {
            if (read_data[i] != basic_data[i]) {
                count_mismatch ++;
            }
        }
        ZF_LOGE("basic: mismatch in bytes at positions, count: %d\n", count_mismatch);
        ZF_LOGE("basic: match in bytes at positions, count: %d\n", basic_data_len - count_mismatch);
        ZF_LOGI("basic_data ptr = %p, read_data ptr = %p", basic_data, read_data);

        for (int i = 0; i < 16; i++) {
            ZF_LOGI("basic_data[%d] = %02x, read_data[%d] = %02x", i, basic_data[i] & 0xff, i, read_data[i] & 0xff);
        }

        for (int i = 0; i < BLK_TRANSFER_SIZE; i += 90) {
            for (int j = 0; j < 90; j++) {
                ZF_LOGI("%c", read_data[i + j]);
            }
        }
        ZF_LOGI("\n");

        ZF_LOGI("basic: successfully finished!\n");

        return true;
    }
    default:
        ZF_LOGE("internal error, invalid state\n");
        assert(false);
    }

    return false;
}

void notified_client(blk_client_boot_info_t *boot_info)
{   
    ZF_LOGI("In Client Notified\n");
    while (true) {
        seL4_Word badge;
        seL4_Wait(boot_info->client_nt, &badge);
        if (badge & BADGE_V2C) {
            if (!test_basic(boot_info)) {
                // microkit_notify(config.virt.id);
                ZF_LOGI("SSSSSSSSSSSSSSSSSSSSS    Client signal to virt    SSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
                seL4_Signal(boot_info->virt_nt);
            }
        }
    }
    
}

static int blk_client_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_client_boot_info_t *boot_info = (blk_client_boot_info_t *)_bootinfo;

    blk_queue_init(&blk_c2v_queue, boot_info->request_shmem_vaddr, boot_info->response_shmem_vaddr, 128);

    /* Want to print out the storage info, so spin until the it is ready. */
    blk_storage_info_t *storage_info = boot_info->storage_info_vaddr;
    ZF_LOGI("Before while\n");
    while (!blk_storage_is_ready(storage_info));
    ZF_LOGI("device config ready\n");
    ZF_LOGI("device size: 0x%lx bytes\n", storage_info->capacity * BLK_TRANSFER_SIZE);

    /* Before proceeding, check that the offset into the device we will
     * do I/O on is sane. */
    assert(REQUEST_BLK_NUMBER < storage_info->capacity - REQUEST_NUM_BLOCKS);

    test_basic(boot_info);
    // microkit_notify(config.virt.id);
    ZF_LOGI("SSSSSSSSSSSSSSSSSSSSS    Client signal to virt    SSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
    seL4_Signal(boot_info->virt_nt);
    notified_client(boot_info);
}

static void init_blk_client_virt(struct env *env, struct blk_client_boot_info * client_bootinfo, struct blk_virt_boot_info * virt_bootinfo) {
    int error;
    ZF_LOGI("####################   in init_blk_client_virt   ##########################\n");
    client_bootinfo->storage_info_vaddr = 0x20000000;
    client_bootinfo->request_shmem_vaddr = 0x20200000;
    client_bootinfo->response_shmem_vaddr = 0x20400000;
    client_bootinfo->client_data_vaddr = 0x20600000;
    virt_bootinfo->client_data_vaddr = 0x20e00000;
    virt_bootinfo->client_request_shmem_vaddr = 0x20a00000;
    virt_bootinfo->client_response_shmem_vaddr = 0x20c00000;
    virt_bootinfo->client_storage_info_vaddr = 0x20800000;

    ZF_LOGI("After init bootinfo\n");
    // storage info, size = 0x1000
    void* paddr;
    map_and_share_frame(env, blk_client_thread.process, blk_virt_thread.process, 
        client_bootinfo->client_data_vaddr, virt_bootinfo->client_data_vaddr, 0x200000, &paddr, 1);
    client_bootinfo->client_data_paddr = paddr;
    virt_bootinfo->client_data_paddr = paddr;
    map_and_share_frame2(env, blk_client_thread.process, blk_virt_thread.process, 
        client_bootinfo->storage_info_vaddr, virt_bootinfo->client_storage_info_vaddr, 0x1000, 0, 1);
    map_and_share_frame2(env, blk_client_thread.process, blk_virt_thread.process, 
        client_bootinfo->request_shmem_vaddr, virt_bootinfo->client_request_shmem_vaddr, 0x200000, 0, 1);
    map_and_share_frame2(env, blk_client_thread.process, blk_virt_thread.process, 
        client_bootinfo->response_shmem_vaddr, virt_bootinfo->client_response_shmem_vaddr, 0x200000, 0, 1);

    ZF_LOGI("After init shared mem\n");
}

// virt.c
#define DRIVER_MAX_NUM_BUFFERS 1024
/* Index allocator for driver request id */
static ialloc_t ialloc;
static uint32_t ialloc_idxlist[DRIVER_MAX_NUM_BUFFERS];

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
    // assert(boot_inf >= BLK_TRANSFER_SIZE);
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
#define MICROKIT_MAX_CHANNELS 62
#define SDDF_BLK_MAX_CLIENTS (MICROKIT_MAX_CHANNELS - 1)
/* Client specific info */
typedef struct client {
    blk_queue_handle_t queue_h;
    uint32_t start_sector;
    uint32_t sectors;
} client_t;
client_t client;

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
    ZF_LOGI("Test blk_queue_empty_req h->req_queue, %p", h.req_queue);
    ZF_LOGI("Test blk_queue_empty_req h->req_queue->tail, %u", h.req_queue->tail);
    ZF_LOGI("Test blk_queue_empty_req h->req_queue->head, %u", h.req_queue->head);
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
            
            // test
            uintptr_t paddr_to_driver = cli_data_base_paddr + cli_offset;
            ZF_LOGI("send paddr = 0x%lx", paddr_to_driver);



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

static int blk_virt_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In virt entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_virt_boot_info_t *boot_info = (blk_virt_boot_info_t *)_bootinfo;
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

static void init_blk_virt_driver(struct env *env, struct blk_virt_boot_info * virt_bootinfo, struct blk_driver_boot_info* driver_bootinfo) {
    int error;
    ZF_LOGI("####################   in init_blk_virt_driver   ##########################\n");
    virt_bootinfo->driver_storage_info_vaddr = 0x20000000;
    virt_bootinfo->driver_request_shmem_vaddr = 0x20200000;
    virt_bootinfo->driver_response_shmem_vaddr = 0x20400000;
    virt_bootinfo->driver_data_vaddr = 0x20600000;
    virt_bootinfo->client.conn.storage_info.vaddr = 0x20800000;

    virt_bootinfo->client.partition = 0;
    virt_bootinfo->client.conn.storage_info.vaddr = (void *)0x20800000;
    virt_bootinfo->client.conn.req_queue.vaddr = (void *)0x20a00000;
    virt_bootinfo->client.conn.resp_queue.vaddr = (void *)0x20c00000;
    virt_bootinfo->client.data.region.vaddr = (void *)0x20e00000;
    virt_bootinfo->client.data.region.size = (void *)0x200000;
    virt_bootinfo->client.data.io_addr = virt_bootinfo->client_data_paddr;


    // virt_bootinfo->client.data.io_addr = 0x5f9f0000;

    // config.driver.conn.storage_info.vaddr = (void *)0x20000000;
    // config.driver.conn.req_queue.vaddr = (void *)0x20200000;
    // config.driver.conn.resp_queue.vaddr = (void *)0x20400000;
    // config.driver.data.region.vaddr = (void *)0x20600000;
    // config.driver.data.io_addr = 0x5fbf0000;


    driver_bootinfo->storage_info_vaddr = 0x20400000;
    driver_bootinfo->request_shmem_vaddr = 0x20600000;
    driver_bootinfo->response_shmem_vaddr = 0x20800000;
    ZF_LOGI("After init bootinfo, virt_bootinfo->client.data.io_addr: %p\n", virt_bootinfo->client.data.io_addr);
    void* paddr;
    map_and_share_frame(env, blk_driver_thread.process, blk_virt_thread.process, 
        0, virt_bootinfo->driver_data_vaddr, 0x200000, &paddr, 1);
    virt_bootinfo->driver_data_paddr = paddr;
    map_and_share_frame2(env, blk_driver_thread.process, blk_virt_thread.process, 
        driver_bootinfo->storage_info_vaddr, virt_bootinfo->driver_storage_info_vaddr, 0x1000, 0, 1);
    map_and_share_frame2(env, blk_driver_thread.process, blk_virt_thread.process, 
        driver_bootinfo->request_shmem_vaddr, virt_bootinfo->driver_request_shmem_vaddr, 0x200000, 0, 1);
    map_and_share_frame2(env, blk_driver_thread.process, blk_virt_thread.process, 
        driver_bootinfo->response_shmem_vaddr, virt_bootinfo->driver_response_shmem_vaddr, 0x200000, 0, 1); 
    ZF_LOGI("After init shared mem\n");
}

static void check_region(uintptr_t base, size_t size)
{
    
}


static int test_blk(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_blk, BLK_001   #################\n");

    // seL4_BootInfo *sel4_bootinfo;
    // simple_t simple2;
    // simple_default_init_bootinfo(&simple2, sel4_bootinfo);

    // int ut_count = simple_get_untyped_count(&simple2);
    // ZF_LOGI("ut count: %d\n", ut_count);
    // for (int i = 0; i < ut_count; i++) {
    //     if(i > 330) {
    //         ZF_LOGI("Skip: %d\n", i);
    //         continue;
    //     }
    //     seL4_Word paddr, size_bits, is_device;
    //     simple_get_nth_untyped(&simple2, i, &size_bits, &paddr, &is_device);
    //     ZF_LOGI("UT[%02d] 0x%08lx-0x%08lx (%2ld KB) %s",
    //             i, paddr, paddr + BIT(size_bits) - 1,
    //             BIT(size_bits) / 1024,
    //             is_device ? "DEVICE" : "RAM");
        
    // }
    // ZF_LOGI("List all ut\n");

    // size_t bits = 0x10000;
    // uintptr_t paddr = 0x5fff0000;
    // vka_object_t ut_obj;
    // int err = vka_alloc_frame(&env->vka, 21, &ut_obj);
    // uintptr_t pres = vka_utspace_paddr(&env->vka, ut_obj.ut, ut_obj.type, ut_obj.size_bits);
    // ZF_LOGI("res alloc  ut, res: %d, paddr: %p\n", err, pres);
    // seL4_ARM_Page_GetAddress_t res = seL4_ARM_Page_GetAddress(ut_obj.cptr);
    // ZF_LOGI("After alloc untyped, %p\n", res.paddr);

    // vka_object_t meta_obj;
    // err = vka_alloc_frame(&env->vka, 21, &meta_obj);
    // pres = vka_utspace_paddr(&env->vka, meta_obj.ut, meta_obj.type, meta_obj.size_bits);
    // ZF_LOGI("res alloc  ut, res: %d, paddr: %p\n", err, pres);
    // res = seL4_ARM_Page_GetAddress(meta_obj.cptr);
    // ZF_LOGI("After alloc untyped meta_obj, %p\n", res.paddr);

    // vka_object_t blk_driver_data_obj;
    // err = vka_alloc_frame(&env->vka, 21, &blk_driver_data_obj);
    // pres = vka_utspace_paddr(&env->vka, blk_driver_data_obj.ut, blk_driver_data_obj.type, blk_driver_data_obj.size_bits);
    // ZF_LOGI("res alloc  ut, res: %d, paddr: %p\n", err, pres);
    // res = seL4_ARM_Page_GetAddress(blk_driver_data_obj.cptr);
    // ZF_LOGI("After alloc untyped blk_driver_data_obj, %p\n", res.paddr);

    // vka_object_t client_data_obj;
    // err = vka_alloc_frame(&env->vka, 21, &client_data_obj);
    // pres = vka_utspace_paddr(&env->vka, client_data_obj.ut, client_data_obj.type, client_data_obj.size_bits);
    // ZF_LOGI("res alloc  ut, res: %d, paddr: %p\n", err, pres);
    // res = seL4_ARM_Page_GetAddress(client_data_obj.cptr);
    // ZF_LOGI("After alloc untyped client_data_obj, %p\n", res.paddr);


    /* 
     * 1. init blk_client_process and start
     */
    create_helper_process(env, &blk_client_thread);
    set_helper_priority(env, &blk_client_thread, 1);
    // init shared memory
    void *vaddr_to_map;
    create_and_share_boot_info(env, blk_client_thread.process, &vaddr_to_map);
    blk_client_boot_info_t *client_bootinfo = (blk_client_boot_info_t *) vaddr_to_map;
    ZF_LOGI("After create client process, %p\n", client_bootinfo->storage_info_vaddr);
    /*
     * 2. init blk_virt and start
     */
    create_helper_process(env, &blk_virt_thread);
    set_helper_priority(env, &blk_virt_thread, 199);
    // init shared memory
    create_and_share_boot_info(env, blk_virt_thread.process, &vaddr_to_map);
    blk_virt_boot_info_t *virt_bootinfo = (blk_virt_boot_info_t *) vaddr_to_map;
    ZF_LOGI("After create virt process, %p\n", virt_bootinfo->client_storage_info_vaddr);

    /**
     * 3. init blk_driver_process and start
    **/ 
    create_helper_process(env, &blk_driver_thread);
    set_helper_priority(env, &blk_driver_thread, 200);
    create_and_share_boot_info(env, blk_driver_thread.process, &vaddr_to_map);
    blk_driver_boot_info_t *driver_bootinfo = (blk_driver_boot_info_t *) vaddr_to_map;
    ZF_LOGI("After create driver process, %p\n", driver_bootinfo->storage_info_vaddr);

    // init shared memory
    init_blk_client_virt(env, client_bootinfo, virt_bootinfo);
    init_blk_virt_driver(env, virt_bootinfo, driver_bootinfo);
    init_blk_driver_server(env, driver_bootinfo);

    ZF_LOGI("After init client vs virt shared memory\n");

    // notification
    seL4_CPtr nt_cd2v = vka_alloc_notification_leaky(&env->vka);
    seL4_CPtr nt_v2d = vka_alloc_notification_leaky(&env->vka);
    seL4_CPtr nt_v2c = vka_alloc_notification_leaky(&env->vka);

    seL4_CPtr badged_c2v = badge_endpoint(env, BADGE_C2V, nt_cd2v);
    seL4_CPtr badged_d2v = badge_endpoint(env, BADGE_D2V, nt_cd2v);
    seL4_CPtr badged_v2c = badge_endpoint(env, BADGE_V2C, nt_v2c);
    seL4_CPtr badged_v2d = badge_endpoint(env, BADGE_V2D, nt_v2d);
    seL4_CPtr badged_v_recv = badge_endpoint(env, 0x0, nt_cd2v);
    
    seL4_CPtr nt_c2v_in_client = sel4utils_copy_cap_to_process(&blk_client_thread.process, &env->vka, badged_c2v);  
    seL4_CPtr nt_v2c_in_client = sel4utils_copy_cap_to_process(&blk_client_thread.process, &env->vka, badged_v2c);

    seL4_CPtr nt_d2v_in_driver =  sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, badged_d2v);
    seL4_CPtr nt_v2d_in_driver = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, badged_v2d); 

    seL4_CPtr nt_v2d_in_virt = sel4utils_copy_cap_to_process(&blk_virt_thread.process, &env->vka, badged_v2d);
    seL4_CPtr nt_v2c_in_virt = sel4utils_copy_cap_to_process(&blk_virt_thread.process, &env->vka, badged_v2c);
    seL4_CPtr nt_cd2v_in_virt = sel4utils_copy_cap_to_process(&blk_virt_thread.process, &env->vka, badged_v_recv);

    client_bootinfo->client_nt = nt_v2c_in_client; 
    client_bootinfo->virt_nt = nt_c2v_in_client;

    driver_bootinfo->driver_nt = nt_v2d_in_driver;
    driver_bootinfo->virt_nt = nt_d2v_in_driver;

    virt_bootinfo-> virt_nt = nt_cd2v_in_virt;
    virt_bootinfo-> client_nt = nt_v2c_in_virt;
    virt_bootinfo-> driver_nt = nt_v2d_in_virt;

    ZF_LOGI("Before irq init\n");
    // irq
    seL4_CPtr irq_ctrl = env->irq_ctrl;
    ZF_LOGI("Get irq_ctrl: %lu\n", irq_ctrl);
    seL4_CPtr badged_nt = badge_endpoint(env, VIRT_IRQ, nt_v2d);
    assert(error == seL4_NoError);

    cspacepath_t irq_handler_path;
    seL4_Word irq_number = 79;
    error = vka_cspace_alloc_path(&env->vka, &irq_handler_path);
    ZF_LOGI("before seL4_IRQControl_GetTrigger: irq_ctrl: %lu, root: %lu, capPtr: %lu, capDepth: %lu\n",irq_ctrl, irq_handler_path.root, irq_handler_path.capPtr, irq_handler_path.capDepth);
    error = seL4_IRQControl_GetTrigger(irq_ctrl, irq_number, 0x01, irq_handler_path.root, irq_handler_path.capPtr, irq_handler_path.capDepth);
    assert(error == seL4_NoError);
    seL4_IRQHandler_SetNotification(irq_handler_path.capPtr, badged_nt);
    seL4_IRQHandler_Ack(irq_handler_path.capPtr);
    ZF_LOGI("Before sel4utils_copy_path_to_process: root: %lu, capRtr: %lu, capDepth: %lu\n", irq_handler_path.root, irq_handler_path.capPtr, irq_handler_path.capDepth);
    seL4_CPtr childIrqCptr = sel4utils_copy_path_to_process(&blk_driver_thread.process, irq_handler_path);
    seL4_CPtr childNTCptr = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, badged_nt);
    seL4_CPtr driver_recv_nt = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, nt_v2d);
    driver_bootinfo->badged_nt = childNTCptr;
    driver_bootinfo->irq_cap = childIrqCptr;
    driver_bootinfo->driver_RECV_nt = driver_recv_nt;
    ZF_LOGI("After irq init\n");

    // start processes

    start_helper(env, &blk_client_thread, &blk_client_entry_point, (seL4_Word)client_bootinfo, 0, 0, 0);
    start_helper(env, &blk_virt_thread, &blk_virt_entry_point, (seL4_Word)virt_bootinfo, 0, 0, 0);
    start_helper(env, &blk_driver_thread, &blk_driver_entry_point, (seL4_Word)driver_bootinfo, 0, 0, 0);

    error = wait_for_helper(&blk_client_thread);
    test_eq(error, 0);

}
DEFINE_TEST(BLK_001, "BLK Example", test_blk, true)
