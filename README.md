# README

This repository contains an example of structuring your libopencm3 based project.

The libopencm3 project aims to create an open-source firmware library for
various ARM Cortex-M3 microcontrollers.

For more information visit http://libopencm3.org

The example are meant as starting point for a stm32f4 discovery board project.

The goal is to demonstrate how one could organize a project. Even though it is
set up to use stm32f4 discovery board as a target it should be fairly easy to
adapt to your own platform.

## Usage

You will have to fetch the libopencm3 submodule by running:

    git submodule init
    git submodule update

You compile the needed part of the library and the project firmware by invoking
"make" in the toplevel directory.

Executing "make flash" will try to use arm-none-eabi-gdb to connect ta a Black
Magic Probe and upload the firmware to your target.

## Contributions

Pull requests simplifying and making this example easier to adapt to other
platforms and projects are welcome! Please strive to keep things fairly simple
and magical as possible. :)
