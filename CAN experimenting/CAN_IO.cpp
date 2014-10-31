/*
 * CAN_IO.cpp
 * Implementation of CAN_IO class.
 */

#include "CAN_IO.h"

CAN_IO::CAN_IO(): controller(1,2) { // not sure of syntax or args
	setup();
}

void CAN_IO::setup() {
	// init the controller
	controller.Init(25, 125); // not sure what this really does or what the args should be

	// return controller to config mode
	controller.Mode(MODE_CONFIG);

	// config RX masks/filters
	setup_rx_filter(0x000, 0x000, true); // 1st buffer; fill with real values
	setup_rx_filter(0x000, 0x000, false); // fill with real values

	// return controller to normal mode
	controller.Mode(MODE_NORMAL);

	// how should errors be detected/handled?
	// what else do we need to do?
}

void CAN_IO::readCAN() {
	// implement!!!
}

void CAN_IO::sendCAN() {
	// implement!!!
}

void CAN_IO::rx_int() {
	// implement!!
}

void CAN_IO::setup_rx_filter(unsigned int mask, unsigned int filter, bool RXB0) {
	// write mask to registers
	controller.Write(first ? RXM0SIDH : RXM1SIDL, first_byte(mask));
	controller.Write(first ? RXM0SIDL : RXM1SIDL, second_byte(mask));
	// write filter to registers
	controller.Write(first ? RXF0SIDH : RXF2SIDH, first_byte(filter)); // RXF2 is first filter for RXB1
	controller.Write(first ? RXF0SIDL : RXF2SIDL, second_byte(filter));

	// how should errors be detected/handled?
	// how can we disable filters that we don't want (do we need to)?
}

byte CAN_IO::first_byte(unsigned int value) {
	return (value - (value % 16)) / 16;
	// how does conversion to byte happen?
}

byte CAN_IO::second_byte(unsigned int value) {
	return (value % 16) * 16;
	// how does conversion to byte happen?
}