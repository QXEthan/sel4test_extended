#
# Copyright 2024 Ethan Xu
#
# SPDX-License-Identifier: BSD-2-Clause
#

cmake_minimum_required(VERSION 3.7.2)
project(blk-virt C)

file(GLOB deps src/*.c)

add_executable(blk-virt EXCLUDE_FROM_ALL ${deps})

target_include_directories(blk-virt PRIVATE include)

target_link_libraries(
  blk-virt
  PUBLIC sel4_autoconf
         muslc
         sel4
         sel4runtime
         sel4allocman
         sel4vka
         sel4utils
         sel4rpc
         sel4test
         sel4sync
         sel4muslcsys
         sel4testsupport
         sel4serialserver_tests
  PRIVATE sel4test-driver_Config)
