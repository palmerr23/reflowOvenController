reflowOvenController
====================

**Reflow Oven Controller with PID Control **

* (c) 2013 Ed Simmons <ed@estechnical.co.uk>
* (c) 2014 Karl Pitrich <karl@pitrich.com>

Suitable for the controler hardware sold by [ESTechnical] or for DIY controllers based on Arduino hardware.


Changes
====================
* 2014-05-17: Support for click encoder
  * The `#define USE_CLICKENCODER 1` in LCDMenu.h selects the use of a rotary encoder with button instead of the buttons in the T962x oven.
  * Configure the Pins used in `LCDMenu.cpp`

Important
====================

After cloning this repository, you need to do 

    git submodule update --init

to fetch all involved libraries. (See: [Submodule Cheat Sheet])



Introduction
====================

The ESTechnical reflow oven controller was designed to operate the T962, T962A and T962C ovens.

It operates using PID control of the heater and fan output to improve the control compared to the original manufacturer's controller.

ESTechnical sell the controller hardware ready to install on eBay and on the ESTechnical website: http://www.estechnical.co.uk

Thanks go to Toby Wilkinson for his excellent menu codebase - http://tobestool.net/t962-t962a-reflow-oven-controller/
Thanks are also due to Brett Beauregard for his excellent work on the arduino PID library - http://playground.arduino.cc/Code/PIDLibrary


Obtaining the source code
====================

Use one of the following methods to get the source code:

Using GIT:
If you are familiar with git and want to use git to check out the source code, use the following command in a terminal:
(This will check out the source code from git into the current directory. I will cover no further use of git here.)

	git clone https://github.com/estechnical/reflowOvenController.git
    git submodule update --init


Installation
====================

To edit/compile the source code for the reflow controller, first install the arduino IDE - http://arduino.cc/en/Main/Software

**Note:** Tested with Arduino 1.5.x only.

If you use the ESTechnical reflow controller which uses a 20MHz crystal instead of a 16MHz crystal,  special build settings for the arduino IDE are required.

Browse to your arduino installation directory, then browse to hardware/arduino/boards.txt. Edit this file (using, for example Programmer's Notepad http://www.pnotepad.org) and add the following block to the top of the file, taking care to not mess up the layout of the file:

	##############################################################

	atmega328_20MHz.name=ESTechnical Reflow controller

	atmega328_20MHz.upload.protocol=stk500
	atmega328_20MHz.upload.maximum_size=30720
	atmega328_20MHz.upload.speed=57600

	atmega328_20MHz.bootloader.low_fuses=0xFF
	atmega328_20MHz.bootloader.high_fuses=0xDA
	atmega328_20MHz.bootloader.extended_fuses=0x05
	atmega328_20MHz.bootloader.path=atmega
	atmega328_20MHz.bootloader.file=ATmegaBOOT_168_atmega328.hex
	atmega328_20MHz.bootloader.unlock_bits=0x3F
	atmega328_20MHz.bootloader.lock_bits=0x0F

	atmega328_20MHz.build.mcu=atmega328p
	atmega328_20MHz.build.f_cpu=20000000L
	atmega328_20MHz.build.core=arduino
	atmega328_20MHz.build.variant=standard


	##############################################################

Save the file and quit.

Setting up the source code to build firmware
====================

Locate the sketchbook directory that the arduino IDE created when installing, you can see the path to your sketchbook folder in the arduino preferences menu (File-> Preferences).

From the files downloaded and unzipped from github:

Copy the ReflowController directory from the zip file into the Sketchbook directory. 

Copy all the directories in the Libraries directory to the libraries directory in the arduino sketchbook directory.

Open the arduino IDE, open the ReflowController.pde sketch (using File->Open). 

Select the right kind of hardware from the Tools->Board menu. 

Compile the firmware with Ctrl+R to test everything is installed correctly. Choose the right serial port from Tools->Serial Port and then upload the code using Ctrl+U.



[Submodule Cheat Sheet]:http://blog.jacius.info/git-submodule-cheat-sheet/
[ESTechnical]:http://www.estechnical.co.uk

