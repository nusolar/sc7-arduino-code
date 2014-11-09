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

Frame Layout::set_header(Frame& f) {
	f.id = id;
	f.dlc = 8; // send 8 bytes
	f.ide = 0; // standard frame
	f.rtr = 0; // data frame
	return f;
}

Frame DriveCmd::generate_frame() {
	Frame f;
	f.low = velocity;
	f.high = current;
	return set_header(f);
}