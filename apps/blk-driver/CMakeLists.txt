cmake_minimum_required(VERSION 3.7.2)
project(blk-driver C)

file(
    GLOB
        deps
        src/*.c
)

add_executable(blk-driver EXCLUDE_FROM_ALL ${deps})

target_include_directories(
    blk-driver
    PRIVATE include
)

target_link_libraries(
    blk-driver
    PUBLIC
        sel4_autoconf
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
    PRIVATE sel4test-driver_Config
)