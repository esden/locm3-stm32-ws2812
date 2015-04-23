# README

This code contains a very simple driver for the WS2812B RGB leds. It is meant
for the STM32F4 microcontroller and uses libopencm3. The code uses a hardware
timer to generate the bit timings and a DMA to load the "bits" into the timer
capture compare register. This lets the CPU free to do other tasks in the mean
time and not be preoccupied with bitbanging of the bits onto the gpio pin. The
current code is buffering up to 6LEDs at a time, and will unpack the remaining
led color values when the transfer is over. It is using a half transfer
interrupt to allow double buffering.

For more information visit http://libopencm3.org

## Usage

You will have to fetch the libopencm3 submodule by running:

    git submodule init
    git submodule update

You compile the needed part of the library and the project firmware by invoking
"make" in the toplevel directory.

Executing "make flash" will try to use arm-none-eabi-gdb to connect ta a Black
Magic Probe and upload the firmware to your target.

If you want to use black magic probe flash procedure. You can run:

make flash BMP\_PORT=/dev/ttyACM0

## Contributions

Pull requests are welcome. Just fork this repository and send a pull request against this code here. :)

## Ideas for improvements and alternative implementations

* Implement explicit led reset function (currently we rely on the fact that the
  user will not send new led values for at least 40us)
* Use the STM32F4 DMA FIFO to decrease the SRAM occupation
* Allow the use of STM32F4 double buffering feature of the DMA
* Test using smaller and bigger bit buffers, also with boundries in the middle
  of a LED value
