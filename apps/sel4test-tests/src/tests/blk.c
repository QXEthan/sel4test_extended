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
#include <cpio/cpio.h>

extern char _child_cpio[];
extern char _child_cpio_end[];


#define SERSERV_TEST_ALLOCMAN_PREALLOCATED_MEMSIZE (64 * 1024)
#define SHMEM_DRIVER_KB 4096


#define VIRT_IRQ 0x1
#define VIRT_NOTIFICATION 0x2
#define BADGE_V2C 0x11
#define BADGE_C2V 0x22
#define BADGE_D2V 0x44
#define BADGE_V2D 0x88

#define QUEUE_SIZE 1024
#define VIRTQ_NUM_REQUESTS QUEUE_SIZE
#define DRIVER_MAX_NUM_BUFFERS 1024

#define BOOTINFO_VADDR  0x30000000
/* Index allocator for driver request id */
static ialloc_t ialloc;
static uint32_t ialloc_idxlist[DRIVER_MAX_NUM_BUFFERS];


volatile struct virtio_blk_config *virtio_config;

/*
* A mapping from virtIO header index in the descriptor virtq ring, to the sDDF ID given
* in the request. We need this mapping due to out of order operations.
*/

uintptr_t virtio_headers_paddr;
struct virtio_blk_req *virtio_headers;
static volatile virtio_mmio_regs_t *regs;
uintptr_t requests_paddr;
uintptr_t requests_vaddr;

blk_queue_handle_t blk_queue;

blk_queue_handle_t blk_v2d_queue;

volatile struct virtq virtq;


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

void create_helper_process_custom_test(env_t env, helper_thread_t *thread, const char *elf_name, int priority, seL4_CPtr asid)
{
    UNUSED int error;

    ZF_LOGI("before assert\n");
    error = vka_alloc_endpoint(&env->vka, &thread->local_endpoint);
    assert(error == 0);
    ZF_LOGI("after assert\n");

    thread->is_process = true;

    sel4utils_process_config_t config = process_config_default_simple(&env->simple, elf_name, priority);
    config = process_config_mcp(config, seL4_MaxPrio);
    config = process_config_auth(config, simple_get_tcb(&env->simple));
    config = process_config_create_cnode(config, TEST_PROCESS_CSPACE_SIZE_BITS);
    error = configure_test_process_custom(&thread->process, &env->vka, &env->vspace, config);
    assert(error == 0);
    ZF_LOGI("after sel4utils_configure_process_custom\n");
    assert(error == 0);

    thread->thread = thread->process.thread;
    assert(error == 0);
}

void start_process(env_t env, helper_thread_t *thread) {
    UNUSED int error;
    error = sel4utils_spawn_process_v(&thread->process, &env->vka, &env->vspace, 0, NULL, 1);
    assert(error == 0);
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



void create_and_share_boot_info(struct env *env, sel4utils_process_t target_process, void **bootinfo)
{
    seL4_Error err;
    ZF_LOGI("in create_and_share_boot_info\n");
    vka_object_t frame_obj;
    err = vka_alloc_frame(&env->vka, seL4_PageBits, &frame_obj);
    assert(err == seL4_NoError);
    ZF_LOGI("After alloc frame\n");
    cspacepath_t frame_path;
    vka_cspace_make_path(&env->vka, frame_obj.cptr, &frame_path);
    ZF_LOGI("After vka_cspace_make_path\n");

    reservation_t reservation = vspace_reserve_range(&env->vspace, PAGE_SIZE_4K, seL4_ReadWrite, 0, bootinfo);
    assert(reservation.res != NULL);
    ZF_LOGI("After vspace_reserve_range_at\n");

    // mapping frame to current process
    vspace_map_pages_at_vaddr(&env->vspace, &frame_path.capPtr, NULL, *bootinfo, 1, seL4_PageBits,  reservation);
    ZF_LOGI("After vspace_map_pages_at_vaddr\n");
    reservation_t reservation_target = vspace_reserve_range_at(&target_process.vspace, (void *)BOOTINFO_VADDR, PAGE_SIZE_4K, seL4_ReadWrite, 0);
    // mapping current vaddr to target vaddr
    err = sel4utils_share_mem_at_vaddr(&env->vspace, &target_process.vspace, *bootinfo, 1, seL4_PageBits, (void *)BOOTINFO_VADDR, reservation_target);
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

static int test_blk(struct env *env)
{
    int error;
    ZF_LOGI("############    In test_blk, BLK_001   #################\n");
    void *bootinfo_vaddr;
    
    /* 
     * 1. init blk_client_process and start
     */
    create_helper_process_custom_test(env, &blk_client_thread, "blk-client", 1, env->asid_pool);
    ZF_LOGI("After create_helper_process_custom\n");
    create_and_share_boot_info(env, blk_client_thread.process, &bootinfo_vaddr);
    ZF_LOGI("After create_and_share_boot_info\n");
    blk_client_boot_info_t *client_bootinfo = (blk_client_boot_info_t *) bootinfo_vaddr;
    ZF_LOGI("After create client process, %p\n", client_bootinfo);
    /*
     * 2. init blk_virt and start
     */
    create_helper_process_custom_test(env, &blk_virt_thread, "blk-virt", 199, env->asid_pool);
    create_and_share_boot_info(env, blk_virt_thread.process, &bootinfo_vaddr);
    blk_virt_boot_info_t *virt_bootinfo = (blk_virt_boot_info_t *) bootinfo_vaddr;
    ZF_LOGI("After create virt process, %p\n", virt_bootinfo);

    /**
     * 3. init blk_driver_process and start
    **/
    create_helper_process_custom_test(env, &blk_driver_thread, "blk-driver", 200, env->asid_pool);
    create_and_share_boot_info(env, blk_driver_thread.process, &bootinfo_vaddr);
    blk_driver_boot_info_t *driver_bootinfo = (blk_driver_boot_info_t *) bootinfo_vaddr;
    ZF_LOGI("After create driver process, %p\n", driver_bootinfo);

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
    test_init_data_t *idata = (test_init_data_t *) env->simple.data;
    seL4_CPtr irq_ctrl = idata->irq_ctrl;
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
    // comments
    seL4_IRQHandler_Ack(irq_handler_path.capPtr);
    // ZF_LOGI("Before sel4utils_copy_path_to_process: root: %lu, capRtr: %lu, capDepth: %lu\n", irq_handler_path.root, irq_handler_path.capPtr, irq_handler_path.capDepth);
    seL4_CPtr childIrqCptr = sel4utils_copy_path_to_process(&blk_driver_thread.process, irq_handler_path);
    seL4_CPtr childNTCptr = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, badged_nt);
    seL4_CPtr driver_recv_nt = sel4utils_copy_cap_to_process(&blk_driver_thread.process, &env->vka, nt_v2d);
    driver_bootinfo->badged_nt = childNTCptr;
    driver_bootinfo->irq_cap = childIrqCptr;
    driver_bootinfo->driver_RECV_nt = driver_recv_nt;

    
    // start processes
    start_process(env, &blk_client_thread);
    start_process(env, &blk_virt_thread);
    start_process(env, &blk_driver_thread);

    // wait_for_helper(&blk_client_thread);
    wait_for_helper(&blk_virt_thread);
    wait_for_helper(&blk_driver_thread);


    test_eq(error, 0);

}
DEFINE_TEST(BLK_001, "BLK Example", test_blk, true)
