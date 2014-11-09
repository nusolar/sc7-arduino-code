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

Frame Layout::frame_header(Frame& f) {
	f.id = id;
	f.dlc = 8; // send 8 bytes
	f.ide = 0; // standard frame
	f.rtr = 0; // data frame
	return f;
}

DriveCmd::DriveCmd(Frame& frame) {
	id = DRIVE_CMD_ID;
	velocity = frame.data.low; // convert properly!
	current = frame.data.high; // convert properly!
}

DriveCmd::generate_frame() {
	Frame f;
	f.data.low = velocity;
	f.data.high = current;
	return set_header(f);
}