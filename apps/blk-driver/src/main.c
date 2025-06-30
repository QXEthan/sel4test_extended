
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
#include <virtio.h>
#include <virtio_queue.h>
#include <ialloc.h>
#include <block.h>
#include <storage_info.h>
#include <msdos_mbr.h>
#include <simple-default/simple-default.h>


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

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE

/*
* Due to the out-of-order nature of virtIO, we need a way of allocating indexes in a
* non-linear way.
*/
ialloc_t ialloc_desc;
uint32_t descriptors[QUEUE_SIZE];

uint16_t last_seen_used = 0;

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

blk_queue_handle_t blk_v2d_queue;

volatile struct virtq virtq;


#define VIRT_IRQ 0x1
#define VIRT_NOTIFICATION 0x2
#define BADGE_V2C 0x11
#define BADGE_C2V 0x22
#define BADGE_D2V 0x44
#define BADGE_V2D 0x88

#define ALIGN(x, align)   (((x) + (align) - 1) & ~((align) - 1))


#define BOOTINFO_VADDR 0x30000000

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
        ZF_LOGI("response header addr: 0x%lx, len: %d\n", hdr_desc.addr, hdr_desc.len);

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



int main(int argc, char *argv[]){
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_driver_boot_info_t *boot_info = (blk_driver_boot_info_t *)BOOTINFO_VADDR;

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