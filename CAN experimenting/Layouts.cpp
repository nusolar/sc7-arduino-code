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

Frame MC_Heartbeat::generate_frame() {
	Frame f;
	f.low = trituim_id;
	f.high = serial_no;
	set_header(f);
	return f;
}

Frame MC_BusStatus::generate_frame() {
	Frame f;
	f.low = bus_current;
	f.high = bus_voltage;
	set_header(f);
	return f;
}

Frame MC_Velocity::generate_frame() {
	Frame f;
	f.low = car_velocity;
	f.high = motor_velocity;
	set_header(f);
	return f;
}

Frame MC_PhaseCurrent::generate_frame() {
	Frame f;
	f.low = phase_a;
	f.high = phase_b;
	set_header(f);
	return f;
}

Frame DC_Heartbeat::generate_frame() {
	Frame f;
	f.low = dc_id;
	f.high = serial_no;
	set_header(f);
	return f;
}

Frame DC_Drive::generate_frame() {
	Frame f;
	f.low = velocity;
	f.high = current;
	set_header(f);
	return f;
}

Frame DC_Power::generate_frame() {
	Frame f;
	f.low = bus_current;
	f.high = 0;
	set_header(f);
	return f;
}

Frame DC_Reset::generate_frame() {
	Frame f;
	f.low = 0;
	f.high = 0;
	set_header(f);
	return f;
}

Frame DC_SwitchPos::generate_frame() {
	Frame f;
	f.data = is_run ? 0x0020 : 0x0040;
	set_header(f);
	return f;
}