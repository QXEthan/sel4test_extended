<!--
     Copyright 2017, Data61, CSIRO (ABN 41 687 119 230)

     SPDX-License-Identifier: CC-BY-SA-4.0
-->

# sel4test_extended

Library for building and running the **blk example** on seL4.

## Setup
[seL4test](https://github.com/seL4/sel4test-manifest) is a test suite for seL4.  

[seL4test_extended](https://github.com/QXEthan/sel4test_extended) is an extended project based on seL4test, adding a ported blk driver and its test example. It currently runs only on the `qemu-arm-virt` virtual platform.


First make sure you have [set up your machine](https://docs.sel4.systems/HostDependencies#sel4-build-dependencies).

## Getting the Code

```bash
mkdir sel4test_extended
cd sel4test_extended
repo init -u https://github.com/QXEthan/sel4test-manifest.git
repo sync
ls
# apps/ CMakeLists.txt init-build.sh kernel libs/ projects/ tools/
```

## Build it
### Configuration

To start a build with a specific configuration we can create a new subdirectory from the project root and initialise it with CMake:

```bash
# create build directory
mkdir build_arm
cd build_arm
# configure build directory
../init-build.sh -DPLATFORM=qemu-arm-virt
```
To configure to run only the **blk example test**, add:  

```bash
cmake . -DLibSel4TestPrinterRegex=BLK_001
```
This configures your build directory with the necessary CMake configuration for your chosen platform.  

You can look at and edit the CMake configuration options from your build directory using
```bash
ccmake .
```

## Useful configuration options

For an overview of the CMake build system and further configuration options, including configuring for cross compilation (targeting ARM), see [seL4’s CMake Build System](https://docs.sel4.systems/Developing/Building/Using).

## Building

When you’ve configured the system, you can build it by running:

```bash
ninja
```

Executing the following command will run QEMU with a **customized configuration** for the blk example and point it towards the image we just built:

```bash
./simulate --extra-qemu-args="-global virtio-mmio.force-legacy=false -drive file=disk,if=none,format=raw,id=hd -device virtio-blk-device,drive=hd"
```
To exit qemu, press `Ctrl-a`, then `x`.

When you see the following output, it means the blk example has run successfully:  
the data has been written from the local file to the virtual disk, read back, and verified to match exactly.
```bash

                                                           -------
                                                           -------
           --------                                        -------             --------
       -----------------            -----       -----      -------           ----------
     ---------------------        ---- ----   ----------   -------          -----------
   -------------------------      ----  ---  -----  ----   -------        -------------
  ---------------------------     -------    -----------   -------       -----  -------
 -----------------------------       ------  -----         -------     -----    -------
--------   -------------------    ----  ---- -----  ----   -------   ------     -------
------       ------------------------- -----------  --------------------------------------
------       -----------------------------------------------------------------------------
-------     ------------------------------------------------------------------------------
 -----------------------------                                                  -------
 ----------------------------                                                   -------
   -------------------------
    -----------------------
      ------------------
          -----------


Lorem ipsum dolor sit amet, consectetur adipiscing elit. Proin ante libero, eleifend ac enim et, accumsan mollis velit. Sed a efficitur risus. Nam in purus imperdiet lorem euismod ultricies. Vestibulum dui orci, suscipit et magna a, sodales lacinia odio. Vivamus a aliquam dui. Suspendisse et nisl ornare, lacinia odio sed, malesuada nibh. Curabitur in quam vel nisi fringilla rhoncus in a risus. Integer accumsan risus elit, et porta ligula viverra eu. Aliquam luctus elit in vulputate sodales. Mauris tempor magna a tincidunt blandit. Mauris et condimentum odio. Interdum et malesuada fames ac ante ipsum primis in faucibus.

Class aptent taciti sociosqu ad litora torquent per conubia nostra, per inceptos himenaeos. Pellentesque egestas sed nisl eget commodo. Morbi lobortis mattis ex. Donec venenatis, nisi nec viverra rutrum, orci ante porta dui, quis lacinia risus nisi non mauris. Integer imperdiet arcu facilisis mi tempus consequat. Praesent risus eros, elementum id diam vitae, faucibus hendrerit dolor. Aenean posuere sit amet nulla id interdum. Pellentesque vel massa ac velit posuere mollis.

Sed scelerisque commodo porta. Ut efficitur purus et dui commodo rhoncus. Nullam in auctor lectus, eu mattis eros. Ut libero ligula, malesuada mattis arcu et, aliquet sagittis ipsum. Nullam eget bibendum nulla, vel mattis risus. In aliquam lectus non finibus lobortis. Nunc id orci eu quam mattis rutrum sed ac ante. Etiam at risus fringilla, mollis elit eu, porttitor nibh. Proin ornare turpis augue, dictum facilisis mi ultricies eu. Phasellus vitae augue et sapien faucibus imperdiet. Duis euismod posuere congue. Integer iaculis efficitur fringilla. Proin tellus ligula, molestie sit amet sagittis at, ullamcorper et sapien. Fusce aliquet ornare arcu et egestas. Pellentesque eleifend metus non mauris vestibulum, sit amet pulvinar dolor sagittis. Cras a eleifend nisi.

Pellentesque mauris libero, posuere et urna ac, euismod hendrerit leo. Nam quis lectus elit. Suspendisse lacinia ante nisi, eget rhoncus tellus ornare in. Ut convallis dapibus eros, quis egestas ante porttitor at. Pellentesque euismod justo libero, id dignissim enim lacinia vel. Morbi non eros in lorem laoreet vulputate ac in mauris. Vivamus efficitur ligula eu quam interdum, a consequat purus imperdiet. Phasellus ipsum massa, iaculis vel lorem vulputate, eleifend euismod dolor. Morbi sit amet aliquet libero, nec aliquet lorem. Praesent imperdiet lacinia orci eget finibus. Etiam sit amet porttitor turpis. Nunc non dignissim nibh. Vivamus convallis erat ac orci gravida aliquet. Etiam convallis lobortis metus non ultricies.

Nullam lacus justo, ornare at luctus sit amet, porttitor nec risus. Maecenas quis leo libero. Praesent tellus quam, viverra sed arcu vitae, eleifend commodo nulla. Duis at nulla aliquet enim gravida blandit. Praesent ac
```
## Testing a Customised Kernel

Suppose you’ve got seL4 checked out in `~/projects/seL4`, and seL4test in `~/tests/sel4test`, and you have been making changes on a feature branch of seL4 named `awesome-new-feature`. You want to test if your modified kernel still passes all the tests in seL4test.

```bash
cd ~/tests/sel4tests/kernel
git remote add feature ~/projects/seL4
git fetch feature
git checkout feature/awesome-new-feature
cd ..
```
Now the kernel in seL4test has been changed to your custom kernel. Now just build and run the test suite as above.

## Running a subset of the tests

You can use a regular expression to select a subset of the tests. This can be configured by setting the CMake variable `LibSel4TestPrinterRegex`. We can modify this variable by running `ccmake .` in our build directory. By default the test suite runs all tests.


## Usage

*Small unit tests* can be defined anywhere, such as libraries outside of `sel4test` or in `sel4test-driver`. *Larger tests* that do things like creating processes need to be declared inside `sel4test-tests`.

### Unit tests

To define a small unit test in a library outside of `sel4test` or in `sel4test-driver`:

1. Declare `libsel4test` as a dependency for your library and include `<sel4test/test.h>`. You may also find the functions in `<sel4test/testutil.h>` handy.
2. Write your tests. Then, for each test you want to run, call one of the macros that define a test, such as the `DEFINE_TEST` macro. They are declared
[here](https://github.com/seL4/seL4_libs/blob/master/libsel4test/include/sel4test/test.h#L88).
3. Add your library as dependency to
[`libsel4testsupport`](https://github.com/seL4/sel4test/blob/master/libsel4testsupport).
Add a call to any function in your test file to `testreporter.c` in [`dummy_func()`](https://github.com/seL4/sel4test/blob/master/libsel4testsupport/src/testreporter.c#L35). If you have multiple test files, then you need to call one function for each test file.

For an example, take a look at [`libsel4serialserver/src/test.c`](https://github.com/seL4/seL4_libs/blob/master/libsel4serialserver/src/test.c) in `sel4_libs`.

### Other tests

To define a larger test in `sel4test-tests`:

1. Place your test in `apps/sel4test-tests/src/tests`.
2. Include `<../helpers.h>`.
3. Write your tests. Then, for each test you want to run, call one of the macros that define a test,
    such as the `DEFINE_TEST` macro. They are declared [here](https://github.com/seL4/seL4_libs/blob/master/libsel4test/include/sel4test/test.h#L88).

For an example, take a look at [`trivial.c`](https://github.com/seL4/sel4test/blob/master/apps/sel4test-tests/src/tests/trivial.c) in `sel4test`.

[build]: https://docs.sel4.systems/Resources#setting-up-your-machine
