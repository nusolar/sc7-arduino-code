CAN library testing
===================

This directory contains code that was used to test the correctness of the CAN_IO library (see sc7-can project). The goal of the project was to have the library code able to send correctly-formatted CAN packets to the motor controller and other vehicle sub-systems.

The CAN library development project completed testing on January 8th, 2015. An Arduino running the can_experimenting.ino code with the MODETX flag defined and DEBUG turned of was able to successfully communicate Drive commands (0x501 CAN packets) to the Tritium WaveSculptor20 over the M12 CAN network. The Arduino was able to correctly control the speed of the wheel using a small dial potentiometer. Spencer Williams and Alexander Martin were present.

Version Information
-------------------

The current version of this testing code is v0.1. It is intended to work with the sc7-can library v0.1, which must be installed in C:/NUsolar/sc7-can/v1.0 in order to compile. 

Misc
----
The Packet Layout Ideas folder contains several different designs that we initially had for the packet Layout objects that would act as a user-friendly interface between CAN Frame objects and car code. Our final design choice was most similar to idea 3.