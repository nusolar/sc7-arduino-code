This folder contains Arduino code for the driver controls.

USAGE
------

This version of driver controls uses V2.0 of the nu solar can library, which must be placed in C:\NUsolar\sc7-can\v3.0.
sc7-can-libinclude.cpp and sc7-can-libinclude.h from v3.0 of the CAN library must be present in the project folder for the driver controls.

The NUsolar fork of the Metro library must be copied to <Arduino>/libraries. Also, 
the Metro and SPI libraries must be added to the project.

To get the watchdog timer to work, the WDT must be re-enabled (it is automatically disabled by the arduino libraries). See "Setting up Arduino 1.6.3.docx" in the NUsolar team Dropbox, under the "SC7 Electrical" directory.