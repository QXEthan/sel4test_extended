/*
 * Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <autoconf.h>
#include <sel4utils/gen_config.h>

#include <vka/vka.h>
#include <vspace/vspace.h>

#include <sel4utils/thread.h>
#include <sel4utils/process_config.h>
#include <sel4utils/vspace.h>
#include <sel4utils/elf.h>
#include <sel4platsupport/timer.h>
#include <sel4utils/process.h>

/**
 * Configure a process with more customisations (Create your own vspace, customise cspace size).
 *
 * @param process               uninitialised process struct
 * @param vka                   allocator to use to allocate objects.
 * @param spawner_vspace        vspace to use to allocate virtual memory in the current address space.
 * @param config process config.
 *
 * @return 0 on success, -1 on error.
 */
int configure_test_process_custom(sel4utils_process_t *process, vka_t *target_vka,
                                       vspace_t *spawner_vspace, sel4utils_process_config_t config);
