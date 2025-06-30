/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <autoconf.h>
#include <sel4utils/gen_config.h>

#define _GNU_SOURCE
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <elf/elf.h>
#include <cpio/cpio.h>
#include <sel4/sel4.h>
#include <vka/object.h>
#include <vka/capops.h>
#include <stdarg.h>
#include <sel4runtime/auxv.h>
#include <sel4utils/vspace.h>
#include "test_process.h"
#include <sel4utils/util.h>
#include <sel4utils/elf.h>
#include <sel4utils/mapping.h>
#include <sel4utils/helpers.h>

/* This library works with our cpio set up in the build system */
extern char _child_cpio[];
extern char _child_cpio_end[];


int create_reservations(vspace_t *vspace, int num, sel4utils_elf_region_t regions[])
{
    for (int i = 0; i < num; i++) {
        sel4utils_elf_region_t *region = &regions[i];
        region->reservation = vspace_reserve_range_at(vspace, region->elf_vstart,
                                                      region->size, region->rights, region->cacheable);
        if (region->reservation.res == NULL) {
            ZF_LOGE("Failed to create region\n");
            for (int j = i - 1; j >= 0; j--) {
                vspace_free_reservation(vspace, regions[i].reservation);
            }
            return -1;
        }
    }

    return 0;
}


void clear_objects(sel4utils_process_t *process, vka_t *vka)
{
    assert(process != NULL);
    assert(vka != NULL);

    while (process->allocated_object_list_head != NULL) {
        object_node_t *prev = process->allocated_object_list_head;

        process->allocated_object_list_head = prev->next;

        vka_free_object(vka, &prev->object);
        free(prev);
    }
}

int next_free_slot(sel4utils_process_t *process, cspacepath_t *dest)
{
    if (process->cspace_next_free >= (BIT(process->cspace_size))) {
        ZF_LOGE("Can't allocate slot, cspace is full.\n");
        return -1;
    }

    dest->root = process->cspace.cptr;
    dest->capPtr = process->cspace_next_free;
    dest->capDepth = process->cspace_size;

    return 0;
}

void allocate_next_slot(sel4utils_process_t *process)
{
    assert(process->cspace_next_free < (BIT(process->cspace_size)));
    process->cspace_next_free++;
}

seL4_CPtr get_asid_pool(seL4_CPtr asid_pool)
{
    if (asid_pool == 0) {
        ZF_LOGW("This method will fail if run in a thread that is not in the root server cspace\n");
        asid_pool = seL4_CapInitThreadASIDPool;
    }

    return asid_pool;
}

seL4_CPtr assign_asid_pool(seL4_CPtr asid_pool, seL4_CPtr pd)
{
    int error = seL4_ARCH_ASIDPool_Assign(get_asid_pool(asid_pool), pd);
    if (error) {
        ZF_LOGE("Failed to assign asid pool\n");
    }

    return error;
}

int create_cspace(vka_t *vka, int size_bits, sel4utils_process_t *process,
                         seL4_Word cspace_root_data, seL4_CPtr asid_pool)
{
    /* create a cspace */
    int error = vka_alloc_cnode_object(vka, size_bits, &process->cspace);
    if (error) {
        ZF_LOGE("Failed to create cspace: %d\n", error);
        return error;
    }

    process->cspace_size = size_bits;
    /* first slot is always 1, never allocate 0 as a cslot */
    process->cspace_next_free = 1;

    /*  mint the cnode cap into the process cspace */
    cspacepath_t src;
    vka_cspace_make_path(vka, process->cspace.cptr, &src);
    UNUSED seL4_CPtr slot = sel4utils_mint_cap_to_process(process, src, seL4_AllRights, cspace_root_data);
    assert(slot == SEL4UTILS_CNODE_SLOT);

    /* copy fault endpoint cap into process cspace */
    if (process->fault_endpoint.cptr != 0) {
        vka_cspace_make_path(vka, process->fault_endpoint.cptr, &src);
        slot = sel4utils_copy_path_to_process(process, src);
        assert(slot == SEL4UTILS_ENDPOINT_SLOT);
    } else {
        /* no fault endpoint, update slot so next will work */
        allocate_next_slot(process);
    }

    /* copy page directory cap into process cspace */
    vka_cspace_make_path(vka, process->pd.cptr, &src);
    slot = sel4utils_copy_path_to_process(process, src);
    assert(slot == SEL4UTILS_PD_SLOT);

    if (!config_set(CONFIG_X86_64)) {
        vka_cspace_make_path(vka, get_asid_pool(asid_pool), &src);
        slot = sel4utils_copy_path_to_process(process, src);
    } else {
        allocate_next_slot(process);
    }
    assert(slot == SEL4UTILS_ASID_POOL_SLOT);

    return 0;
}

int create_fault_endpoint(vka_t *vka, sel4utils_process_t *process)
{
    /* create a fault endpoint and put it into the cspace */
    int error = vka_alloc_endpoint(vka, &process->fault_endpoint);
    process->own_ep = true;
    if (error) {
        ZF_LOGE("Failed to allocate fault endpoint: %d\n", error);
        return error;
    }

    return 0;
}

int configure_test_process_custom(sel4utils_process_t *process, vka_t *vka,
                                       vspace_t *spawner_vspace, sel4utils_process_config_t config)
{
    int error;
    sel4utils_alloc_data_t *data = NULL;
    memset(process, 0, sizeof(sel4utils_process_t));
    seL4_Word cspace_root_data = api_make_guard_skip_word(seL4_WordBits - config.one_level_cspace_size_bits);

    /* create a page directory */
    process->own_vspace = config.create_vspace;
    if (config.create_vspace) {
        error = vka_alloc_vspace_root(vka, &process->pd);
        if (error) {
            ZF_LOGE("Failed to allocate page directory for new process: %d\n", error);
            goto error;
        }

        /* assign an asid pool */
        if (!config_set(CONFIG_X86_64) &&
            assign_asid_pool(config.asid_pool, process->pd.cptr) != seL4_NoError) {
            goto error;
        }
    } else {
        process->pd = config.page_dir;
    }

    if (config.create_fault_endpoint) {
        if (create_fault_endpoint(vka, process) != 0) {
            goto error;
        }
    } else {
        process->fault_endpoint = config.fault_endpoint;
    }

    process->own_cspace = config.create_cspace;
    if (config.create_cspace) {
        if (create_cspace(vka, config.one_level_cspace_size_bits, process, cspace_root_data,
                          config.asid_pool) != 0) {
            goto error;
        }
    } else {
        process->cspace = config.cnode;
    }

    /* create a vspace */
    if (config.create_vspace) {
        sel4utils_get_vspace(spawner_vspace, &process->vspace, &process->data, vka, process->pd.cptr,
                             sel4utils_allocated_object, (void *) process);

        if (config.num_reservations > 0) {
            if (create_reservations(&process->vspace, config.num_reservations,
                                    config.reservations)) {
                goto error;
            }
        }
    } else {
        memcpy(&process->vspace, config.vspace, sizeof(process->vspace));
    }

    /* finally elf load */
    if (config.is_elf) {
        ZF_LOGI("78787878787878788     In elf      7878787878787877\n");
        unsigned long size;
        unsigned long cpio_len = _child_cpio_end - _child_cpio;
        ZF_LOGI("Before cpio_get_file, _cpio_archive_end: %p, image_name: %s, CPIO: %p\n", _child_cpio_end, config.image_name, _child_cpio);
        uint8_t *cpio_bytes = (uint8_t *)_child_cpio;
        ZF_LOGI("CPIO first 8 bytes: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            cpio_bytes[0], cpio_bytes[1], cpio_bytes[2], cpio_bytes[3],
            cpio_bytes[4], cpio_bytes[5], cpio_bytes[6], cpio_bytes[7]);
        char const *file = cpio_get_file(_child_cpio, cpio_len, config.image_name, &size);
        ZF_LOGI("Aftercpio_get_file file: %p\n", file);
        elf_t elf;
        elf_newFile(file, size, &elf);
        ZF_LOGI("After elf_newfile\n");
        if (config.do_elf_load) {
            ZF_LOGI("In do elf load, process: %p\n", process);
            process->entry_point = sel4utils_elf_load(&process->vspace, spawner_vspace, vka, vka, &elf);
        } else {
            ZF_LOGI("In not do elf load\n");
            process->num_elf_regions = sel4utils_elf_num_regions(&elf);
            process->elf_regions = calloc(process->num_elf_regions, sizeof(*process->elf_regions));
            if (!process->elf_regions) {
                ZF_LOGE("Failed to allocate memory for elf region information");
                goto error;
            }
            process->entry_point = sel4utils_elf_reserve(&process->vspace, &elf, process->elf_regions);
        }
        ZF_LOGI("After do elf load\n");

        if (process->entry_point == NULL) {
            ZF_LOGE("Failed to load elf file\n");
            goto error;
        }

        process->sysinfo = sel4utils_elf_get_vsyscall(&elf);

        ZF_LOGI("Before Retrieve elf phdrs\n");
        /* Retrieve the ELF phdrs */
        process->num_elf_phdrs = sel4utils_elf_num_phdrs(&elf);
        process->elf_phdrs = calloc(process->num_elf_phdrs, sizeof(Elf_Phdr));
        if (!process->elf_phdrs) {
            ZF_LOGE("Failed to allocate memory for elf phdr information");
            goto error;
        }
        ZF_LOGI("Before read elf phdrs\n");
        sel4utils_elf_read_phdrs(&elf, process->num_elf_phdrs, process->elf_phdrs);

        /* If PT_PHDR exists in the program headers, assign PT_NULL to it.
         * This is because muslc libc searches for PT_PHDR and if found,
         * it assumes it's part of the ELF image and relocates the entire
         * subsequent program header segments according to PT_PHDR's base. This is
         * wrong and will trigger mapping errors.
         */
        Elf_Phdr *phdr = process->elf_phdrs;

        for (int i = 0; i < process->num_elf_phdrs; i++, phdr++) {
            if (phdr->p_type == PT_PHDR) {
                phdr->p_type = PT_NULL;
            }
        }

    } else {
        ZF_LOGI("78787878787878788     Not in elf      7878787878787877\n");
        process->entry_point = config.entry_point;
        process->sysinfo = config.sysinfo;
    }

    /* select the default page size of machine this process is running on */
    process->pagesz = PAGE_SIZE_4K;

    /* create the thread, do this *after* elf-loading so that we don't clobber
     * the required virtual memory*/
    sel4utils_thread_config_t thread_config = {0};
    thread_config = thread_config_cspace(thread_config, process->cspace.cptr, cspace_root_data);
    if (config_set(CONFIG_KERNEL_MCS)) {
        /* on the MCS kernel, use the fault endpoint in the current cspace */
        thread_config = thread_config_fault_endpoint(thread_config, process->fault_endpoint.cptr);
    } else if (process->fault_endpoint.cptr != 0) {
        /* on the master kernel, the fault ep must be in the cspace of the process */
        thread_config = thread_config_fault_endpoint(thread_config, SEL4UTILS_ENDPOINT_SLOT);
    }
    thread_config.sched_params = config.sched_params;
    thread_config.create_reply = config.create_cspace;
    error = sel4utils_configure_thread_config(vka, spawner_vspace, &process->vspace, thread_config,
                                              &process->thread);
    if (error) {
        ZF_LOGE("ERROR: failed to configure thread for new process %d\n", error);
        goto error;
    }

    /* copy tcb cap to cspace */
    if (config.create_cspace) {
        cspacepath_t src;
        vka_cspace_make_path(vka, process->thread.tcb.cptr, &src);
        UNUSED seL4_CPtr slot = sel4utils_copy_path_to_process(process, src);
        assert(slot == SEL4UTILS_TCB_SLOT);
        process->dest_tcb_cptr = SEL4UTILS_TCB_SLOT;
    } else {
        process->dest_tcb_cptr = config.dest_cspace_tcb_cptr;
    }

    if (config.create_cspace) {
        if (config_set(CONFIG_KERNEL_MCS)) {
            seL4_CPtr UNUSED slot = sel4utils_copy_cap_to_process(process, vka, process->thread.sched_context.cptr);
            assert(slot == SEL4UTILS_SCHED_CONTEXT_SLOT);
            slot = sel4utils_copy_cap_to_process(process, vka, process->thread.reply.cptr);
            assert(slot == SEL4UTILS_REPLY_SLOT);
        } else {
            /* skip the sc slot */
            allocate_next_slot(process);
            /* skip the reply object slot */
            allocate_next_slot(process);
        }
    }

    return 0;

error:
    /* try to clean up */
    if (config.create_fault_endpoint && process->fault_endpoint.cptr != 0) {
        vka_free_object(vka, &process->fault_endpoint);
    }

    if (config.create_cspace && process->cspace.cptr != 0) {
        vka_free_object(vka, &process->cspace);
    }

    if (config.create_vspace && process->pd.cptr != 0) {
        vka_free_object(vka, &process->pd);
        if (process->vspace.data != 0) {
            ZF_LOGE("Could not clean up vspace\n");
        }
    }

    if (process->elf_regions) {
        free(process->elf_regions);
    }

    if (process->elf_phdrs) {
        free(process->elf_phdrs);
    }

    if (data != NULL) {
        free(data);
    }

    return -1;
}
