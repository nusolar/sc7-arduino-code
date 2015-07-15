--Updated: 7/14/15--

Steering Wheel Controls 
Version: 2.0
NUSOLAR 2015
by Ethan Park
########################

The steering wheel controls allow for the driver to control the car gear, lights, cruise control, horn, and turn signals. Note that the steering wheel controls does not affect the car's physical systems in any way. The code reads what has changed in the switches, if any, and communicates with the driver controls. However, the steering wheel controls ARE responsible for displaying to the driver information regarding the state of the gears, lights, cruise control, turn signals, as well as velocity and battery charge. The last two will be read from CAN packets from the battery management system and the motor control.

----------
Board Info
----------

The steering wheel code runs in a 3.3V SparkFun ProMicro Arduino. Currently, versino 1.0.5 of the Arduino software is needed to program this board with the custom configuration parameters which can be downloaded from 
(https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/hardware-overview-pro-micro)

It is very easy to 'brick' the ProMicro with this code; if you cannot program the board, use the 'RST' trick outlined here

(https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/troubleshooting-and-faq#ts-reset)

---------------------------------------------------
Libraries Used  (location	  ; where to find  )
----------------------------------------------------
Switches 	(Arduino/libraries; included in zip)
Metro  #39c65de	(Arduino/libraries; NUsolar GitHub )
serLCD #fcef62d	(Arduino/libraries; NUsolar GitHub )
SPI		(Built-in)
sc7-can	v3.0	(C:/NUsolar/sc7-can; NUsolar GitHub)
avr/wdt.h	(Built-in)