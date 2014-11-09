/*
 * Layouts.cpp
 * Implementation of CAN packet layouts.
 */

#include "Layouts.h"
#include "PacketIDs.h"

Frame Layout::generate_frame() {
	Frame f;

	f.id = id;
	f.dlc = 8;
	copy_data(data.bytes, f.data);
	//f.low = velocity...

	return f;
}

void Layout::copy_data(uint8_t* source, uint8_t* destination) {
	for (int i = 0; i < 8; i++) {
		destination[i] = source[i];
	}
}

DriveCmd::DriveCmd(float velocity, float current) {
	data.low = velocity; // convert properly!
	data.high = current; // convert properly!
	id = DriveCmd_ID; // make constants for these!
	this->velocity = velocity;
	this->current = current;
}

DriveCmd::DriveCmd(Frame& frame) {

	copy_data(frame.data, data.bytes);
	velocity = data.low; // convert properly!
	current = data.high; // convert properly!
	id = DriveCmd_ID;
}