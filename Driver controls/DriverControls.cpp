/*
 * DriverControls.cpp
 * Contains the implemenation of the DriverControls class.
 */

 #include "DriverControls.h"

DriverControls::DriverControls() {
	// init car state

	// init CAN I/O

	// init instrument I/O
}

void DriverControls::sendCAN() {
	// wait until ready to send packets out, if ready:

		// send drive command packet

		// send display update packet

		// send BMS packet if car is starting?

	// update timers and such
}

void DriverControls::readCAN() {
	// receive incoming packet and identify

		// BMS state? update BMS state

		// Motor velocity? update car velocity

		// Motor bus? update bus state info

		// User cmds? update state info

	// update timers

	// check for time-outs and kill if necessary
}