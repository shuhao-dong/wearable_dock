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

Before you build the source code, remember to change the broker address and the local host name in the top macros.

To build the source code, run::

    gcc -std=c11 -Wall -O2 dfu_and_extract.c  -ludev -o ~/dfu_and_extract

Then navigate to your HOME directory and run::

    sudo ./dfu_and_extract

4. Add to System Service
************************

You need to firstly move the compiled two executables into $PATH::

    sudo install -m 755 ./scan /usr/local/bin/              # This is the BLE receiver executable
    sudo install -m 755 ./wearable_dock_run /usr/local/bin/   # This is the dock specific executable

Since the BLE receiver codes needs a specified HCI controller index from hciconfig, one needs to install a small launcher script::

    sudo install -m 755 run_scan.sh /usr/local/bin

This will ensure the selected HCI device is always unblocked by rfkill and selected for the BLE receiver executable. 

Then, you will need to copy the two .service files into /etc/systemd/system/, and run::

    sudo systemctl daemon-reload
    sudo systemctl enable --now scan@1.service  # scan@1 will choose hci1, change accordingly
    sudo systemctl enable --now wearable_dock.service

The two services are now added and started, and will automatically start after each reboot. 

To observe the output from each service, run::
    
    journalctl -u scan@1 -f

To stop/start each service, run::

    sudo systemctl stop scan@1
    sudo systemctl restart scan@1   # Use sudo systemctl status scan@1 to verify current status

To change HCI index::

    sudo systemctl disable scan@1
    sudo systemctl enable --now scan@0
 
