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

// blk driver process
static helper_thread_t blk_driver_thread;

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

} blk_driver_boot_info_t;


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

static void map_and_share_frame2(struct env *env, sel4utils_process_t target_process,  uintptr_t vaddr, size_t size_bytes)
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

void notified(microkit_channel ch)
{
    if (ch == device_resources.irqs[0].id) {
        handle_irq();
        microkit_deferred_irq_ack(ch);
        /*
         * It is possible that we could not enqueue all requests when being notified
         * by the virtualiser because we ran out of space, so we try again now that
         * we have received a response and have resources freed.
         */
        handle_request();
    } else if (ch == config.virt.id) {
        handle_request();
    } else {
        LOG_DRIVER_ERR("received notification from unknown channel: 0x%x\n", ch);
    }
}

static int blk_driver_entry_point(seL4_Word _bootinfo, seL4_Word a1, seL4_Word a2, seL4_Word a3)
{
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     In entry point     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_driver_boot_info_t *boot_info = (blk_driver_boot_info_t *)_bootinfo;
    
    blk_queue_handle_t blk_queue;

    uintptr_t virtio_headers_paddr = boot_info->headers_paddr;
    struct virtio_blk_req *virtio_headers = (struct virtio_blk_req *) boot_info->headers_vaddr;

    /*
    * A mapping from virtIO header index in the descriptor virtq ring, to the sDDF ID given
    * in the request. We need this mapping due to out of order operations.
    */
    uint32_t virtio_header_to_id[QUEUE_SIZE];

    virtio_blk_init(boot_info);
    blk_queue_init(&blk_queue, boot_info->request_shmem_vaddr, boot_info->response_shmem_vaddr, VIRTQ_NUM_REQUESTS);
}

void virtio_blk_init(blk_driver_boot_info_t *boot_info)
    {
        ZF_LOGI("In virtio_blk_init\n");
        /* Block device configuration, populated during initiliastion. */
        volatile struct virtio_blk_config *virtio_config;
        volatile virtio_mmio_regs_t *regs = (volatile virtio_mmio_regs_t *) boot_info->regs_vaddr;

        uintptr_t requests_paddr = boot_info->request_paddr;
        uintptr_t requests_vaddr = boot_info->request_shmem_vaddr;
        volatile struct virtq virtq;

        /*
        * Due to the out-of-order nature of virtIO, we need a way of allocating indexes in a
        * non-linear way.
        */
        ialloc_t ialloc_desc;
        uint32_t descriptors[QUEUE_SIZE];

        uint16_t last_seen_used = 0;
        
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


static void init_blk_driver_server(struct env *env) {
    int error;
    seL4_Word prio = 200;
    ZF_LOGI("in init_blk_driver_server\n");
    // init process
    create_helper_process(env, &blk_driver_thread);
    set_helper_priority(env, &blk_driver_thread, prio);
    // init shared memory
    ZF_LOGI("After create process\n");
    void *vaddr_to_map;
    create_and_share_boot_info(env, blk_driver_thread.process, &vaddr_to_map);
    blk_driver_boot_info_t *driver_bootinfo = (blk_driver_boot_info_t *) vaddr_to_map;
    ZF_LOGI("After create bootinfo, %x\n", vaddr_to_map);
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
    // start driver
    start_helper(env, &blk_driver_thread, &blk_driver_entry_point, (seL4_Word)driver_bootinfo, 0, 0, 0);
    error = wait_for_helper(&blk_driver_thread);
    test_eq(error, 0);
    ZF_LOGI("After boot process\n");
}

static int test_blk(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_blk, BLK_001   #################\n");

    /**
     * 1. init blk_driver_process and start
    **/ 
    init_blk_driver_server(env);
    /* 
     * 2. init blk_virt_process and start
     * 3. init blk_client and start
     */

    // init blk_driver_process

}
DEFINE_TEST(BLK_001, "BLK Example", test_blk, true)
