The Quadrotor Control Board (QCB) Firmware
------------------------------------------
The QCB provides control of the motors and expansion modules of the system. The
motors are controlled by four independent PWMs and communication with the phone
occurs over Bluetooth.

Building Firmware and flashing
------------------------------
Committed to this project are the Eclipse project files nessecary to build the
firmware. A GNU ARM toolchain is required to compile the code. Yagarto is
recommended. The QCB has a twenty pin JTAG header
that can be used for flashing and debugging code.

1) Install a GDB server, install Yagarto, install and configure Eclipse
   Follow the instructions on the Yagarto website
   http://www.yagarto.de/howto.html

2) Build project
   Provided that Yagarto is setup properly and in your systems path, executing
   build in the Eclipse project should produce a number of files.

3) Flash and debug
   There are two debug configurations. The qcb-firmware_ram configuration can
   be run directly and executes out of ram. The qcb-firmware_rom configuration
   requires that the firmware first be flashed to the QCB. There are a number
   of ways to do so, the easiest is to use SEGGER's J-Flash ARM utility
   (assuming a JLink is being used). It is also possible to flash ATMEL's
   SAM-BA bootloader to the microcontroller by connecting the TST jumper to the
   (*) pin and resetting the board. After this, it is possible to use the
   SAM-BA software to flash a new firmware image.

Acknowledgements
----------------
We would like to acknowledge the Yagarto project for their toolchain and their
sample projects. In particular, our project uses slightly modified linker
files, gdb scripts, and Makefile from the SAM7S256Test project which saved us a
lot of effort.

Yagarto example projects -> http://www.yagarto.de/examples/index.html
