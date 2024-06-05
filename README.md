# MCP2515 SPI-CAN Module Test Example

This example demostrates how to use MCP2515 Module with ESP-IDF. The module works using SPI Communication with MCU.

## How to use example

- To use this example and the library, please assign the MISO, MOSI, CLK and CS pins in the "spi_comp.h" file.
- If your project reqires to interface multiple MCP2515 modules in the same SPI Bus, just assign multiple CS pins in the given above file
- The give exmaple in this file demonstrates 3 MCP2515 connected to the same SPI Bus with different CS pins.
- If you only need to use one device at a time, please comment out the part of the code for other 2 devices in "main.c" accordingly!

NOTE:
	- Plese refer the begining of "spi_comp.h" files to assign required pins for your proejcts application.

### Hardware Required

- This example requires only a single target (e.g., an ESP32, ESP32-S3 or ESP32-S2).
- This connection usually consists of a MISO, MOSI, CLK and CS (more than 1 if needed)


### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.


## Example Breakdown

- In this example, the SPI gets initialized first in HSPI mode and then MCP2515 gets initialized first with CAN_250KBPS, MCP_8MHZ configuration
please make sure you put the data in the arguments according to your project needs.

- In order to use muliple devices in the same bus, remove the first device and add second and only intialize as above and repeat the same steps for other devices to initialze them.

- After initialization, CAN data and ID is read in MCP_RW_task() task. The removing and adding of devices is used to read and write CAN data with ID as given in the step above(only applicable for multiple device). 


## Important Notice

Please refer and go through the library file and its APIs and use it according to your application use cases. You are free to use only the library files and edit it accordingly in your other projects as well but be caseful with the include files and pin assignments!
