Steering Wheel Controls 
Version: 1.0
NUSOLAR 2015
by Ethan Park

The steering wheel controls allow for the driver to control the car gear, lights, cruise control, horn, and turn signals. Note that the steering wheel controls does not affect the car's physical systems in any way. The code reads what has changed in the switches, if any, and communicates with the driver controls. However, the steering wheel controls ARE responsible for displaying to the driver information regarding the state of the gears, lights, cruise control, turn signals, as well as velocity and battery charge. The last two will be read from CAN packets from the battery management system and the motor control.

Libraries used include Switch, Metro, serLCD, SPI, and relevant CAN libraries.