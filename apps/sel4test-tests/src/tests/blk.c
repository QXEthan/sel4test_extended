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
#include <simple-default/simple-default.h>
#include "../test.h"
#include "../helpers.h"

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE
#define SERSERV_TEST_ALLOCMAN_PREALLOCATED_MEMSIZE (64 * 1024)
#define SHMEM_DRIVER_KB 4096

#define ALIGN(x, align)   (((x) + (align) - 1) & ~((align) - 1))

#define VIRT_IRQ 1
#define VIRT_NOTIFICATION 2
#define BADGE_V2C 0x11
#define BADGE_C2V 0x22


volatile struct virtio_blk_config *virtio_config;

/*
* A mapping from virtIO header index in the descriptor virtq ring, to the sDDF ID given
* in the request. We need this mapping due to out of order operations.
*/
uint32_t virtio_header_to_id[QUEUE_SIZE];

uintptr_t virtio_headers_paddr;
struct virtio_blk_req *virtio_headers;

blk_queue_handle_t blk_queue;
blk_queue_handle_t blk_c2v_queue;

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
    seL4_CPtr driver_virt_nt;

} blk_driver_boot_info_t;

typedef struct blk_client_boot_info {
    uintptr_t storage_info_vaddr;
    uintptr_t request_shmem_vaddr;
    uintptr_t request_paddr;
    uintptr_t response_shmem_vaddr;
    uintptr_t response_paddr;
    uintptr_t client_data;

    // notification
    seL4_CPtr virt_nt;
    seL4_CPtr client_nt;

} blk_client_boot_info_t;

typedef struct blk_virt_boot_info {
    uintptr_t driver_storage_info_vaddr;
    uintptr_t driver_request_shmem_vaddr;
    uintptr_t driver_request_paddr;
    uintptr_t driver_response_shmem_vaddr;
    uintptr_t driver_response_paddr;
    uintptr_t driver_data;

    uintptr_t client_storage_info_vaddr;
    uintptr_t client_request_shmem_vaddr;
    uintptr_t client_request_paddr;
    uintptr_t client_response_shmem_vaddr;
    uintptr_t client_response_paddr;
    uintptr_t client_data;

    // notification
    seL4_CPtr virt_nt;
    seL4_CPtr client_nt;

} blk_virt_boot_info_t;

// void notified(microkit_channel ch)
// {
//     if (ch == device_resources.irqs[0].id) {
//         handle_irq();
//         microkit_deferred_irq_ack(ch);
//         /*
//          * It is possible that we could not enqueue all requests when being notified
//          * by the virtualiser because we ran out of space, so we try again now that
//          * we have received a response and have resources freed.
//          */
//         handle_request();
//     } else if (ch == config.virt.id) {
//         handle_request();
//     } else {
//         LOG_DRIVER_ERR("received notification from unknown channel: 0x%x\n", ch);
//     }
// }

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

static void map_and_share_frame(struct env *env, sel4utils_process_t target_process,  uintptr_t vaddr, void **paddr_ptr, size_t size_bytes)
{

    seL4_Error err;
    ZF_LOGI("In map_and_share_frame, bytes: %d\n", size_bytes);
    size_t size_bits = bytes_to_size_bits(size_bytes);
    size_t page_count = size_bytes / BIT(PAGE_BITS_4K) + size_bytes % BIT(PAGE_BITS_4K) == 0 ? 0 : 1;
    assert((1UL << size_bits) == size_bytes);
    ZF_LOGI("In map_and_share_frame after reservation\n");
    uintptr_t current_vaddr  = vaddr;
    uintptr_t current_paddr = (uintptr_t)(*paddr_ptr);

    // get frame cap
    for (int i = 0; i < page_count; i++) {
        current_paddr = current_paddr ? (current_paddr + i * BIT(PAGE_BITS_4K)) : 0;
        current_vaddr = current_vaddr + i * BIT(PAGE_BITS_4K);
        reservation_t reservation = vspace_reserve_range(&env->vspace, BIT(PAGE_BITS_4K),
            seL4_ReadWrite, 0, (void **)&current_vaddr);
        assert(reservation.res != NULL);
        vka_object_t frame;
        int err;

        if (current_paddr) {
            err = vka_alloc_frame_at(&env->vka, PAGE_BITS_4K, current_paddr, &frame);
        } else {
            err = vka_alloc_frame(&env->vka, PAGE_BITS_4K, &frame);
            *paddr_ptr = vka_object_paddr(&env->vka, &frame);
        }
        
        // ZF_LOGI("In for loop alloc frame return error: %d\n",err);
        assert(err == seL4_NoError);
        err = vspace_map_pages_at_vaddr(&env->vspace, &frame.cptr, NULL, (void *)current_vaddr, 1, PAGE_BITS_4K, reservation);
        // ZF_LOGI("After vspace_map_pages_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
        err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, (void *)current_vaddr, 1, PAGE_BITS_4K, (void *)current_vaddr, reservation);
        // ZF_LOGI("After sel4utils_share_mem_at_vaddr return error: %d\n",err);
        assert(err == seL4_NoError);
    }
}

static void map_and_share_frame2(struct env *env, sel4utils_process_t target_process, sel4utils_process_t target_process2, 
        uintptr_t vaddr1, uintptr_t vaddr2, size_t size_bytes)
{
    seL4_Error err;
    ZF_LOGI("In map_and_share_frame2, bytes: %d\n", size_bytes);
    size_t size_bits = bytes_to_size_bits(size_bytes);
    reservation_t reservation = vspace_reserve_range(&env->vspace, size_bytes, seL4_ReadWrite, 0, (void **)&vaddr);
    assert(reservation.res != NULL);
    ZF_LOGI("In map_and_share_frame2 after reservation, bits: %d\n", size_bits);
    // get frame cap
    vka_object_t result;
    err = vka_alloc_untyped(&env->vka, size_bits, &result);
    assert(err == seL4_NoError);
    ZF_LOGI("In map_and_share_frame2 after alloc untyped\n");
    size_t num_pages = (size_bytes + BIT(12) - 1) / BIT(12);
    ZF_LOGI("In 2 num page: %d\n", num_pages);
    seL4_CPtr frame_caps[num_pages];
    for (int i = 0; i < num_pages; i++) {
        seL4_Word slot = get_free_slot(env);
        cspacepath_t dest_path;
        vka_cspace_make_path(&env->vka, slot, &dest_path);
        frame_caps[i] = dest_path.capPtr;
        err = vka_untyped_retype(&result, seL4_ARCH_4KPage, seL4_PageBits, 1, &dest_path);
        assert(err == seL4_NoError);
    }
    ZF_LOGI("In map_and_share_frame2 after untyped retype, frame capptr[0]: %p\n", frame_caps[0]);
    // ZF_LOGI("In for loop alloc frame return error: %d\n",err);
    assert(err == seL4_NoError);
    err = vspace_map_pages_at_vaddr(&env->vspace, frame_caps, NULL, (void *)vaddr, num_pages, seL4_PageBits, reservation);
    // ZF_LOGI("After vspace_map_pages_at_vaddr return error: %d\n",err);
    assert(err == seL4_NoError);
    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, (void *)vaddr, num_pages, seL4_PageBits, (void *)vaddr, reservation);
    // ZF_LOGI("After sel4utils_share_mem_at_vaddr return error: %d\n",err);
    assert(err == seL4_NoError);
}

static void setupMMIO_regs_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes= 0x1000;
    ZF_LOGI("In setupMMIO_regs_shmem: %d\n", size_bytes);
    void *mapped_vaddr = (void *)bootinfo->regs_vaddr;

    reservation_t res = vspace_reserve_range_at(&env->vspace, mapped_vaddr, size_bytes, seL4_ReadWrite, 1);
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
    // uintptr_t mmio_virtio_phys_addr = 0x5fff0000;
    size_t size_bytes = 0x10000;
    ZF_LOGI("In setupMMIO_virtio_headers_shmem: %d\n", size_bytes);
    map_and_share_frame2(env, target_process, bootinfo->headers_vaddr, size_bytes);
}

static void setupMMIO_virtio_metadata_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    // uintptr_t mmio_meta_phys_addr = 0x5fdf0000;
    size_t size_bytes = 0x200000;
    ZF_LOGI("In setupMMIO_virtio_metadata_shmem: %d\n", size_bytes);
    map_and_share_frame2(env, target_process, bootinfo->meta_vaddr, size_bytes);
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

static void setup_driver_queue_shmem(struct env *env, sel4utils_process_t target_process, blk_driver_boot_info_t* bootinfo) {
    size_t size_bytes = 0x200000;
    ZF_LOGI("In setup_driver_queue_shmem: %d\n", size_bytes);
    map_and_share_frame2(env, target_process, bootinfo->request_shmem_vaddr, size_bytes);
    map_and_share_frame2(env, target_process, bootinfo->response_shmem_vaddr, size_bytes);
}

void notified(blk_driver_boot_info_t *boot_info)
{
    volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;
    while (1) {
        seL4_Word badge;
        seL4_Wait(boot_info->badged_nt, &badge);
        if (badge & VIRT_IRQ) {
            ZF_LOGI("Get IRQ from device\n");
            handle_irq(boot_info);
            handle_request(boot_info);
            /* Ack */
            seL4_IRQHandler_Ack(boot_info->irq_cap);
        }
        if (badge & VIRT_NOTIFICATION) {
            handle_request(boot_info);
        }
        if (!(badge & (VIRT_IRQ | VIRT_NOTIFICATION))) {
            ZF_LOGE("received notification from unknown channel: 0x%x\n", badge);
        }
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
        LOG_DRIVER("response data addr: 0x%lx, data len: %d\n", data_addr, data_len);
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
        seL4_Signal(boot_info->driver_virt_nt);
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
                LOG_DRIVER("handling read request with physical address 0x%lx, block_number: 0x%x, count: 0x%x, id: 0x%x\n",
                           phys_addr, block_number, count, id);
            } else {
                LOG_DRIVER("handling write request with physical address 0x%lx, block_number: 0x%x, count: 0x%x, id: 0x%x\n",
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
            virtio_queue_notify = true;

            virtio_header_to_id[hdr_desc_idx] = id;

            break;
        }
        case BLK_REQ_FLUSH: {
            int err = blk_enqueue_resp(&blk_queue, BLK_RESP_OK, 0, id);
            assert(!err);
            // microkit_notify(config.virt.id);
            seL4_Signal(boot_info->driver_virt_nt);
            break;
        }
        case BLK_REQ_BARRIER: {
            int err = blk_enqueue_resp(&blk_queue, BLK_RESP_OK, 0, id);
            assert(!err);
            //microkit_notify(config.virt.id);
            seL4_Signal(boot_info->driver_virt_nt);
            break;
        }
        default:
            /* The virtualiser should have sanitised the request code and so we should never get here. */
            LOG_DRIVER_ERR("unsupported request code: 0x%x\n", req_code);
            break;
        }
    }

    if (virtio_queue_notify) {
        regs->QueueNotify = 0;
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

    virtio_blk_init(boot_info);
    blk_queue_init(&blk_queue, boot_info->request_shmem_vaddr, boot_info->response_shmem_vaddr, VIRTQ_NUM_REQUESTS);

    //notified
    notified(boot_info);
}

void virtio_blk_init(blk_driver_boot_info_t *boot_info)
    {
        ZF_LOGI("In virtio_blk_init\n");
        /* Block device configuration, populated during initiliastion. */
        volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;

        uintptr_t requests_paddr = boot_info->request_paddr;
        uintptr_t requests_vaddr = boot_info->request_shmem_vaddr;
        for (int i = 0; i < 8; i++) {
            volatile virtio_mmio_regs_t *regs_tmp = (volatile virtio_mmio_regs_t *) (boot_info->regs_vaddr + i*0x200);
            ZF_LOGI("for loop %p\n", regs_tmp);
            if (virtio_mmio_check_magic(regs_tmp) && virtio_mmio_version(regs_tmp) == VIRTIO_VERSION &&
                 virtio_mmio_check_device_id(regs_tmp, VIRTIO_DEVICE_ID_BLK)) {
                ZF_LOGI("##$$$$$######   find blk device on %p $#$#$#$#$#$#$#$#$\n", regs_tmp);
                regs = regs_tmp;
            }
        }
        ZF_LOGI("Before virtio_mmio_check_magic check, %d\n", regs->MagicValue);
        // Do MMIO device init (section 4.2.3.1)
        if (!virtio_mmio_check_magic(regs)) {
            ZF_LOGE("invalid virtIO magic value!\n");
            assert(false);
        }
    
        ZF_LOGI("Before virtio_mmio_version check, %d\n", regs->Version);
    
        if (virtio_mmio_version(regs) != VIRTIO_VERSION) {
            ZF_LOGE("not correct virtIO version!\n");
            assert(false);
        }
        ZF_LOGI("After virtio_mmio_version check\n");
        
        ZF_LOGI("Before virtio_mmio_check_device_id check, %d\n", regs->DeviceID);
        if (!virtio_mmio_check_device_id(regs, VIRTIO_DEVICE_ID_BLK)) {
            ZF_LOGE("not a virtIO block device!\n");
            assert(false);
        }
        ZF_LOGI("After virtio_mmio_check_device_id check\n");
    
        if (virtio_mmio_version(regs) != VIRTIO_BLK_DRIVER_VERSION) {
            ZF_LOGE("driver does not support given virtIO version: 0x%x\n", virtio_mmio_version(regs));
            assert(false);
        }
        ZF_LOGI("After virtio_mmio_version2 check\n");
    
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
        virtq.used = (struct virtq_used *)(requests_vaddr + used_off);
    
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
    setup_driver_storage_info_shmem(env, blk_driver_thread.process, driver_bootinfo);
    setup_driver_queue_shmem(env, blk_driver_thread.process, driver_bootinfo);
    ZF_LOGI("After init shared mem\n");

    // notification
    seL4_CPtr nt = vka_alloc_notification_leaky(&env->vka);
    seL4_CPtr badged_nt = badge_endpoint(env, VIRT_IRQ, nt);
    cspacepath_t irq_path;
    vka_cspace_alloc_path(&env->vka, &irq_path);
    seL4_IRQControl_GetTrigger(seL4_CapIRQControl, 47, 1, irq_path.root, irq_path.capPtr, irq_path.capDepth);
    seL4_IRQHandler_SetNotification(irq_path.capPtr, badged_nt);
    seL4_IRQHandler_Ack(irq_path.capPtr);

    seL4_CPtr childIrqCptr = sel4utils_copy_path_to_process(&blk_driver_thread.process, irq_path);
    seL4_CPtr childNTCptr = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, badged_nt);
    driver_bootinfo->badged_nt = childNTCptr;
    driver_bootinfo->irq_cap = childIrqCptr;

    // start driver
    start_helper(env, &blk_driver_thread, &blk_driver_entry_point, (seL4_Word)driver_bootinfo, 0, 0, 0);
    ZF_LOGI("After boot process\n");
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
        char *data_dest = (char *)boot_info->client_data;
        sddf_memcpy(data_dest, basic_data, basic_data_len);

        int err = blk_enqueue_req(&blk_queue, BLK_REQ_WRITE, 0, REQUEST_BLK_NUMBER, REQUEST_NUM_BLOCKS, 0);
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
        int err = blk_dequeue_resp(&blk_queue, &status, &count, &id);
        assert(!err);
        assert(status == BLK_RESP_OK);
        assert(count == REQUEST_NUM_BLOCKS);
        assert(id == 0);

        /* We do the read at a different offset into the data region from the previous request */
        uintptr_t offset = REQUEST_NUM_BLOCKS * BLK_TRANSFER_SIZE;
        err = blk_enqueue_req(&blk_queue, BLK_REQ_READ, offset, REQUEST_BLK_NUMBER, REQUEST_NUM_BLOCKS, 0);
        assert(!err);

        test_basic_state = FINISH;

        break;
    }
    case FINISH: {
        ZF_LOGI("basic: FINISH state\n");
        blk_resp_status_t status = -1;
        uint16_t count = -1;
        uint32_t id = -1;
        int err = blk_dequeue_resp(&blk_queue, &status, &count, &id);
        assert(!err);
        assert(status == BLK_RESP_OK);
        assert(count == REQUEST_NUM_BLOCKS);
        assert(id == 0);

        // Check that the read went okay
        char *read_data = (char *)(boot_info->client_data + (REQUEST_NUM_BLOCKS * BLK_TRANSFER_SIZE));
        for (int i = 0; i < basic_data_len; i++) {
            if (read_data[i] != basic_data[i]) {
                ZF_LOGE("basic: mismatch in bytes at position %d\n", i);
            }
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

static int blk_client_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In client entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_client_boot_info_t *boot_info = (blk_driver_boot_info_t *)_bootinfo;
    blk_queue_init(&blk_c2v_queue, boot_info->request_shmem_vaddr, boot_info->response_shmem_vaddr, VIRTQ_NUM_REQUESTS);

    /* Want to print out the storage info, so spin until the it is ready. */
    blk_storage_info_t *storage_info = boot_info->storage_info_vaddr;
    while (!blk_storage_is_ready(storage_info));
    ZF_LOGI("device config ready\n");
    ZF_LOGI("device size: 0x%lx bytes\n", storage_info->capacity * BLK_TRANSFER_SIZE);

    /* Before proceeding, check that the offset into the device we will
     * do I/O on is sane. */
    assert(REQUEST_BLK_NUMBER < storage_info->capacity - REQUEST_NUM_BLOCKS);

    test_basic(boot_info);
    // microkit_notify(config.virt.id);
    seL4_Signal(boot_info->virt_nt);
}

static void init_blk_client_virt(struct env *env, struct blk_client_boot_info * client_bootinfo, struct blk_virt_boot_info * virt_bootinfo) {
    int error;
    ZF_LOGI("####################   in init_blk_client_virt   ##########################\n");
    client_bootinfo->storage_info_vaddr = 0x20000000;
    client_bootinfo->request_shmem_vaddr = 0x20200000;
    client_bootinfo->response_shmem_vaddr = 0x20400000;
    client_bootinfo->client_data = 0x20600000;
    virt_bootinfo->client_data = 0x20e00000;
    virt_bootinfo->client_request_shmem_vaddr = 0x20a00000;
    virt_bootinfo->client_response_shmem_vaddr = 0x20c00000;
    virt_bootinfo->client_storage_info_vaddr = 0x20800000;
    ZF_LOGI("After init bootinfo\n");
    // storage info, size = 0x1000
    map_and_share_frame2(env, blk_client_thread.process, client_bootinfo->storage_info_vaddr, 0x1000);
    map_and_share_frame2(env, blk_client_thread.process, client_bootinfo->request_shmem_vaddr, 0x200000);
    map_and_share_frame2(env, blk_client_thread.process, client_bootinfo->response_shmem_vaddr, 0x200000);
    map_and_share_frame2(env, blk_client_thread.process, client_bootinfo->client_data, 0x200000);
    ZF_LOGI("After init shared mem\n");

    // notification
    seL4_CPtr nt_c2v = vka_alloc_notification_leaky(&env->vka);
    seL4_CPtr badged_nt = badge_endpoint(env, BADGE_V2C, nt_c2v);
    seL4_CPtr nt_v2c_in_client = sel4utils_copy_cap_to_process(&blk_client_thread.process, &env->vka, badged_nt);     /* client Wait */
    seL4_CPtr nt_v2c_send_in_virt = sel4utils_copy_cap_to_process(&blk_virt_thread.process, &env->vka, badged_nt);     /* virt Signal */
    seL4_CPtr badged_nt_c2v = badge_endpoint(env, BADGE_C2V, nt_c2v);
    seL4_CPtr nt_c2v_in_virt =  sel4utils_copy_cap_to_process(&blk_virt_thread.process, &env->vka, badged_nt_c2v);     /* virt Wait */
    seL4_CPtr nt_c2v_send_in_client = sel4utils_copy_cap_to_process(&blk_client_thread.process, &env->vka, badged_nt_c2v);     /* client Signal */

    client_bootinfo->client_nt = nt_v2c_in_client;       /* client Wait badge 0x11 */
    client_bootinfo->virt_nt = nt_c2v_send_in_client;  /* client Signal badge 0x22 */

    // virt_boot->clients[cli_id].nt_wait   = nt_c2v_in_virt;
    // virt_cfg->clients[cli_id].nt_signal = nt_v2c_send_in_virt;

    // start driver
    start_helper(env, &blk_client_thread, &blk_client_entry_point, (seL4_Word)client_bootinfo, 0, 0, 0);
    ZF_LOGI("After boot process\n");
}

static void init_blk_virt_process(struct env *env, struct blk_virt_boot_info * client_bootinfo) {
    int error;
    ZF_LOGI("####################   in init_virt_client   ##########################\n");

}


static int test_blk(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_blk, BLK_001   #################\n");

    /* 
     * 1. init blk_client_process and start
     */
    create_helper_process(env, &blk_client_thread);
    set_helper_priority(env, &blk_client_thread, 1);
    // init shared memory
    ZF_LOGI("After create client process\n");
    void *vaddr_to_map;
    create_and_share_boot_info(env, blk_client_thread.process, &vaddr_to_map);
    blk_client_boot_info_t *client_bootinfo = (blk_client_boot_info_t *) vaddr_to_map;
    init_blk_client(env, client_bootinfo);
    /*
     * 2. init blk_virt and start
     */
    create_helper_process(env, &blk_virt_thread);
    set_helper_priority(env, &blk_virt_thread, 199);
    // init shared memory
    ZF_LOGI("After create virt process\n");
    create_and_share_boot_info(env, blk_virt_thread.process, &vaddr_to_map);
    blk_virt_boot_info_t *virt_bootinfo = (blk_virt_boot_info_t *) vaddr_to_map;
    init_blk_virt_process(env, virt_bootinfo);

    /**
     * 3. init blk_driver_process and start
    **/ 
    create_helper_process(env, &blk_driver_thread);
    set_helper_priority(env, &blk_driver_thread, 200);
    ZF_LOGI("After create driver process\n");
    create_and_share_boot_info(env, blk_driver_thread.process, &vaddr_to_map);
    blk_driver_boot_info_t *driver_bootinfo = (blk_driver_boot_info_t *) vaddr_to_map;
    init_blk_driver_server(env, driver_bootinfo);

    // init shared memory


    error = wait_for_helper(&blk_client_thread);
    test_eq(error, 0);

}
DEFINE_TEST(BLK_001, "BLK Example", test_blk, true)
