This folder contains Arduino code for the driver controls.

USAGE
------

sc7-can-libinclude.cpp and sc7-can-libinclude.h must be added to the project.

The Metro library must be copied to <Arduino>/libraries. Also, 
the Metro and SPI libraries must be added to the project.

To get the watchdog timer to work, the line WDT_Disable(WDT) (line 353)
must be commented out in 
<Arduino>/hardware/arduino/sam/variants/arduino_due_x/variants.cpp
