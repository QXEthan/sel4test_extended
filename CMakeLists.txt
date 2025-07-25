#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)

include(settings.cmake)

project(sel4test C CXX ASM)

find_package(seL4 REQUIRED)
find_package(elfloader-tool REQUIRED)

# Set the roottask CNode size to be large enough for DTB, timer caps, etc
# Note that this works for the platforms that we have at the moment, and may
# need to be increased in the future
set(KernelRootCNodeSizeBits 13 CACHE INTERNAL "")

# Set our custom domain schedule
set(KernelDomainSchedule "${CMAKE_CURRENT_LIST_DIR}/domain_schedule.c" CACHE INTERNAL "")
sel4_import_kernel()

if((NOT Sel4testAllowSettingsOverride) AND (KernelArchARM OR KernelArchRiscV))
    # Elfloader settings that correspond to how Data61 sets its boards up.
    ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
endif()
elfloader_import_project()

add_subdirectory(apps/sel4test-driver)

if(SIMULATION)
    include(simulation)
    if(KernelSel4ArchX86_64)
        SetSimulationScriptProperty(MEM_SIZE "3G")
    endif()
    if(KernelPlatformQEMUArmVirt)
        SetSimulationScriptProperty(MEM_SIZE "2G")
    endif()
    GenerateSimulateScript()
endif()

# Path to mkvirtdisk script
set(MKVIRTDISK ${CMAKE_SOURCE_DIR}/mkvirtdisk)

# Target virtual disk output path
set(VIRTUAL_DISK ${CMAKE_BINARY_DIR}/disk)

# Define how to generate the virtual disk
add_custom_command(
    OUTPUT ${VIRTUAL_DISK}
    COMMAND ${MKVIRTDISK} ${VIRTUAL_DISK} 1 512 16777216
    DEPENDS ${MKVIRTDISK}
    COMMENT "Generating virtual disk image using mkvirtdisk..."
)

# Hook this generation into the build system
add_custom_target(create_virtual_disk ALL DEPENDS ${VIRTUAL_DISK})
