1. Overview
***********

This is the project code for TORUS wearable dock station to perform DFU and extract content in the external flash/SD card. The code is primarily
tested on Raspberry Pi 5 and Raspberry Pi Zero 2 W with RPi OS or Ubuntu.

.. code-block:: none

    wearable_dock/
        |---extracted/                              # Extracted .bin file from the wearable's external flash
        |       |---timestamped extraction folder   # Timestamped folder with extracted .bin file
        |---new_firmware/                           # Contains the new firmware to be flashed with DFU
        |_______|---archive                         # Contains the already flashed old firmware 

2. Requirements
***************

You will need to install the following library before compiling the source code::

    sudo apt-get install build-essential libudev-dev fuse

You have to install dfu-util to perform DFU from application::

    sudo apt-get install dfu-util

Finally, you need to follow `this instructions <https://github.com/littlefs-project/littlefs-fuse>`_ for littlefs-fuse in order to use lfs to extract data.

3. Running the Programme
************************

To build the source code, run::

    gcc -std=c11 -Wall -O2 dfu_and_extract.c  -ludev -o ~/dfu_and_extract

Then navigate to your HOME directory and run::

    sudo ./dfu_and_extract

