# Firmware for UWB localization and anchor autocalibration using Decawave's DWM1001-DEV board
Custom localization and calibration firmware for Decawave's DWM1001 DEV ultra-wideband (UWB) development modules.

## Installation

**NOTE:** We recomment using Keil µVision IDE if SES version is not available any more.

  1. Download Segger Embedded Studio (SES) version V3.34a (Note the version to ensure compatibility)
  2. Download Segger J-Flash Lite and GNU ARM Embedded Toolchain 5.4 2016q3
  3. Clone Decawave's code example from https://github.com/Decawave/dwm1001-examples.git

SES has a free license for nrF52832 development. Consequently, this IDE can be used without any limitation for DWM1001 development.

For more information regarding Segger Embedded Studio, please visit https://www.segger.com/products/development-tools/embedded-studio/

For more information about free license for nrF52832, please read https://www.nordicsemi.com/News/News-releases/Product-Related-News/Nordic-Semiconductor-adds-Embedded-Studio-IDE-support-for-nRF51-and-nRF52-SoC-development

## How to program the devices

Follow these steps to load the initiator and responder firmware on the DWM1001-DEV boards.

### Initiator
1. Open the project inside `dwm1001-examples/examples/ss_twr_init` but replace the files `ss_init_main.c` and `main.c` with the ones of the same name provided in the folder Initiator (inside of the Autocalibration or Localization folder depending on your objective).

2. Paste the file shared_var.h provided in this repository inside the folder `dwm1001-examples/examples/ss_twr_init/SES/`

3. Open the project in Segger, set the device ID to the desired value, connect the target device via USB and click Build and Run.


### Responders
1. Copy the folder `dwm1001-examples/examples/ss_twr_init` and change the name to `ss_twr_resp` to identify it.

2. Replace the files `ss_init_main.c` and `main.c` with the ones of the same name provided in the folder Responder (inside of the Autocalibration or Localization folder depending on your objective).

3. Paste the file `shared_var.h` provided in this repository inside the folder `dwm1001-examples/examples/ss_twr_resp/SES/`

4. Open the project in Segger, set the device ID to the desired value, connect the target device via USB and click Build and Run.



## KEIL µVision IDE

Each example contains a µVision5 project file for Keil µVision IDE. The examples compile and load cleanly to the DWM1001.
The project was created with the KEIL uVision version V5.24.2.0. 

Keil µVision has a free license for project up to 32KB. For more information regarding Keil µVision, please visit http://www2.keil.com/mdk5/uvision/

### µVision Error: Flash Download failed - "Cortex-M4"

This error can be observed if there is a memory conflict between the binary to load and the current firmware on the target hardware. This issue can be easily fixed by fully erasing the target device 's flash memory. Keil µVision cannot perform a full erase and the following free tool can be used :

* J-flash lite 
* nrfjprog command line script

For more information about the issue, please see :

https://devzone.nordicsemi.com/f/nordic-q-a/18278/error-flash-download-failed---cortex---m4-while-flashing-softdevice-from-keil-uvision-5


## Restore the factory firmware

In order to store the factory firmware, first download the `.hex` file from

```
https://www.decawave.com/wp-content/uploads/2019/03/DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v9.zip
```

Extract the ZIP. You will find the `.hex` factory firmware file in `DWM1001_DWM1001-DEV_MDEK1001_Sources_and_Docs_v9/DWM1001/Factory_Firmware_Image/DWM1001_PANS_R2.0.hex`.

Open SEGGER J-Flash Lite (tested with V6.62a, the executable file is `JFlashLiteExe`) and choose the following fields:

1. Device: `NRF52832_XXAA`
2. Interface: `SWD`
3. Speed: `4000 kHz`
4. Data File: `DWM1001_PANS_R2.0.hex` (downloaded following steps above)

Connect the DWM1001-DEV board to a USB port (we found some issues if you connect it after you start JFlashLite, so we recommend that you connect the dev board **BEFORE** starting JFlashLite). Optionally, click first on `Erase Chip`.

Click on `Program Device`. If you also erased the previous board, you should see the following output:

```
Connecting to J-Link...Connecting to target...Erasing...Done
Conecting to J-Link...Connecting to target...Downloading...Done
```

**NOTE:** The LEDs on the DEV board will be OFF while the firmware is being updated, **do not disconnect the board** until you see the `Done` message and the LEDs are flashing again. After erasing the device, you should see a single non-blinking red LED on the DEV board.
