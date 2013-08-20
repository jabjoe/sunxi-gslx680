About
=====

This is a gslx680 driver for the sunxi (AllWinner) platform.
The aim is to make the gslx680 usable with a GNU/Linux build.
The Android build has not been removed, but it not been tested.

The source was originally from:

http://code.google.com/p/yuandao-n90-window-dual-core-2/source/browse/drivers/input/touchscreen/gslx680_ts.c

This was the only source for the gslx680 available.

It has been:

* Ported to the sunxi platform (AllWinner).
* Changed to load firmware from a separate file.
* Given extracted firmware from a Android driver of a 7inch A13 tablet.
* Added normal single point, touch along side the existing multi touch.


Firmware Instructions
=====================

The firmware included is from a 7inch tablet with a 800x480 screen.
If this is does not match you tablet, you will need to extract the 
firmware from the existing Android tablet.

It will be under a path like:
/system/vendor/modules/gslx680.ko

Copy this to a SD card or use a GNU/Linux chroot to scp it over, or use
adb pull.

On your build machine, on the command line use the script
'firmware/fw_extractor' to extract the firmware to it's own file.

./firmware/fw_extractor my_android_gslx680.ko my_tablets.fw


Build Instructions
==================

Open the file Makefile in your text edit of choice. Change the second
line, the KDIR variable to the build folder of your kernel.

On the command line, build the module with just:

make


Install Instructions
====================

You will need to generate a new script.bin from a modified fex file.
You should already be using a fex file generated from your script.bin
from your Android device. Find the ctp_para section.

Ensure the lines:

ctp_used = 1
ctp_name = "gslx680"
ctp_twi_id = 1
ctp_twi_addr = 0x40

On some devices the driver is a hack and isn't using the fex system
properly. Or doing some unknown interaction with semi documented autotp
sections. So the matching parameters in ctp_para may have different
values.


Add the line:

ctp_firmware = "my_tablets.fw"

But using the name of the file of the firmware you wish to use.
Create a new script.bin.
Copy the new script.bin to the SD card you are booting from.

Copy your firmware file to /lib/firmware/ on your GNU/Linux install
(probably also on the SD card).

Copy your gslx680_ts.ko to a your GNU/Linux install.

Boot your device into GNU/Linux.
Insert the gslx680_ts.ko module (insmod gslx680_ts.ko).

Using 'dmesg' you should see something like:

[   64.130000] ===========================gslx680_ts_init=====================
[   64.140000] _fetch_sysconfig_para. 
[   64.160000] gslx680 firmware a13_7inch_800x480.fw. 
[   64.170000] _fetch_sysconfig_para: after: ctp_twi_addr is 0x40, dirty_addr_buf: 0x40. dirty_addr_buf[1]: 0xfffe 
[   64.180000] _fetch_sysconfig_para: ctp_twi_id is 1. 
[   64.190000] _fetch_sysconfig_para: screen_max_x = 800. 
[   64.200000] _fetch_sysconfig_para: screen_max_y = 480. 
[   64.210000] _fetch_sysconfig_para: revert_x_flag = 1. 
[   64.220000] _fetch_sysconfig_para: revert_y_flag = 0. 
[   64.230000] _fetch_sysconfig_para: exchange_x_y_flag = 0. 
[   64.240000] ctp_detect: Detected chip gslx680 at adapter 1, address 0x40
[   64.260000] ====gslx680_ts_probe begin=====.  
[   64.280000] ==kzalloc success=
[   64.280000] [GSLX680] Enter gsl_ts_init_ts
[   64.300000] ctp_set_irq_mode: config gpio to int mode. 
[   64.310000] ctp_set_irq_mode, 924: gpio_int_info, port = 7, port_num = 11. 
[   64.320000]  INTERRUPT CONFIG
[   64.340000] input: gslx680 as /devices/platform/sunxi-i2c.1/i2c-1/1-0040/input/input1
[   64.460000] =============gsl_load_fw start==============
[   65.460000] =============gsl_load_fw end==============
[   65.780000] ==gslx680_ts_probe over =

Use 'evtest' to check it is working. Select the gslx680. Touch the
screen, you should see something like:


Event: time 1377032989.427032, ++++++++++++++ SYN_MT_REPORT ++++++++++++
Event: time 1377032989.427036, type 3 (EV_ABS), code 0 (ABS_X), value 171
Event: time 1377032989.427039, type 3 (EV_ABS), code 1 (ABS_Y), value 225
Event: time 1377032989.427044, type 3 (EV_ABS), code 57 (ABS_MT_TRACKING_ID), value 3
Event: time 1377032989.427047, type 3 (EV_ABS), code 48 (ABS_MT_TOUCH_MAJOR), value 10
Event: time 1377032989.427050, type 3 (EV_ABS), code 53 (ABS_MT_POSITION_X), value 171
Event: time 1377032989.427054, type 3 (EV_ABS), code 54 (ABS_MT_POSITION_Y), value 225
Event: time 1377032989.427057, type 3 (EV_ABS), code 50 (ABS_MT_WIDTH_MAJOR), value 1
Event: time 1377032989.427060, ++++++++++++++ SYN_MT_REPORT ++++++++++++

All is well, ensure you have the xserver-xorg-input-evdev installed
before trying X. Only single point touch is enabled as standard.
