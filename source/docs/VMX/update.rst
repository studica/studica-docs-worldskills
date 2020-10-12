Updating Firmware
=================

In certain cases you may need to update the VMX firmware; use the following instructions to accomplish updating the VMX firmware.

Requirements
------------

- VMX Circuit Board (rev. 5.35 or higher)
- PC with USB 2.0 port running Windows 7 or greater.
- Micro-USB Cable

Updating the Firmware
---------------------

- Download the `VMX Tools for Windows <https://www.kauailabs.com/public_files/vmx-pi/vmx-pi.zip>`__ latest build.
- Unpack the contents of the vmx-pi.zip file and run the setup.exe program, which will install the tools as well as all necessary device drivers for communicating over USB with the VMX-pi, as well as some additional tools.  In addition, the setup program will install the latest firmware at the following location:

  <HomeDirectory>\\vmx-pi\\firmware

  For example, if your user name is Robot, the directory name will be C:\Users\Robot\vmx-pi\firmware.

  Within that directory, the firmware file will be named using this pattern:

    vmx-pi_X.Y.ZZZ.hex

    (X = Major Version Number Y = Minor Version Number Z = Revision Number)

- Press and hold down the “CAL” button on the VMX circuit board.  While holding this button down, connect a USB-micro cable from a Windows PC to the VMX circuit board.  Use the micro-usb connector immediately to the left of the VMX power connector.  Applying power when the “CAL” button is held down places the board into “bootloader” mode, at which point the firmware can be loaded.

- From your Start Menu, select “Kauai Labs” and then click on the VMXFirmwareUpdater menu item, and follow the directions included in the program.

- Once you have downloaded the firmware, you can use the “Currently-loaded Firmware Version” tab of the VMXFirmwareUpdater to verify the version number you have  just installed.