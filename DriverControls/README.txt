This folder contains Arduino code for the driver controls.

USAGE
------

This version of driver controls uses V2.0 of the nu solar can library, which must be placed in C:\NUsolar\sc7-can\v2.0.
sc7-can-libinclude.cpp and sc7-can-libinclude.h from v2.0 of the CAN library must be present in the project folder for the driver controls.

The NUsolar fork of the Metro library must be copied to <Arduino>/libraries. Also, 
the Metro and SPI libraries must be added to the project.

To get the watchdog timer to work, the line WDT_Disable(WDT) (line 353)
must be commented out in 
<Arduino>/hardware/arduino/sam/variants/arduino_due_x/variants.cpp
