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
#include <basic_data.h>
#include <virtio.h>
#include <virtio_queue.h>
#include <queue.h>
#include <ialloc.h>
#include <block.h>
#include <storage_info.h>
#include <msdos_mbr.h>
#include <simple-default/simple-default.h>

#define BADGE_V2C 0x11


blk_queue_handle_t blk_c2v_queue;

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


/* Use the start of the partition for testing. */
#define REQUEST_BLK_NUMBER 0
#define REQUEST_NUM_BLOCKS 2
#define BOOTINFO_VADDR 0x30000000

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
                seL4_DebugPutChar(read_data[i + j]);
            }
        }
        seL4_DebugPutString("\n");

        seL4_DebugPutString("[BLK_001] BLK Example ran successfully!\n");

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
            // add sequense diagram.
            if (!test_basic(boot_info)) {
                // microkit_notify(config.virt.id);
                ZF_LOGI("SSSSSSSSSSSSSSSSSSSSS    Client signal to virt    SSSSSSSSSSSSSSSSSSSSSSSSSSS\n");
                seL4_Signal(boot_info->virt_nt);
            }
        }
    }
}


int main(int argc, char *argv[]){
    ZF_LOGI("@@@@@@@@@@@@@@@@@@@@@@@@@@     blk_client: hello from independent ELF!     @@@@@@@@@@@@@@@@@@@@@@@@@@\n");


    blk_client_boot_info_t *boot_info = (blk_client_boot_info_t *)BOOTINFO_VADDR;

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
    
    return 0;
}