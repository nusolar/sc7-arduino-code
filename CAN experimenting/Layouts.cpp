/*
 * Layouts.cpp
 * Implementation of CAN packet layouts.
 */

#include "Layouts.h"
#include "PacketIDs.h"

Frame Layout::generate_frame() {
	Frame f;
	return f;
}

void Layout::set_header(Frame& f) {
	f.id = id;
	f.dlc = 8; // send 8 bytes
	f.ide = 0; // make it a standard frame
	f.rtr = 0; // make it a data frame
	f.srr = 0;
}

Frame DriveCmd::generate_frame() {
	Frame f;
	f.low = velocity;
	f.high = current;
	set_header(f);
	return f;
}

Frame PowerCmd::generate_frame() {
	Frame f;
	f.low = bus_current;
	f.high = 0;
	set_header(f);
	return f;
}