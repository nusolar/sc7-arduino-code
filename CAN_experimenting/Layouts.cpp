/*
 * Layouts.cpp
 * Implementation of CAN packet layouts.
 */

#include "Layouts.h"

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

Frame BMS_Heartbeat::generate_frame() {
	Frame f;
	f.low = device_id;
	f.high = serial_no;
	set_header(f);
	return f;
}

Frame BMS_SOC::generate_frame() {
	Frame f;
	f.low = power_consumed;
	f.high = percent_SOC;
	set_header(f);
	return f;
}

Frame BMS_BalanceSOC::generate_frame() {
	Frame f;
	f.low = power_supplied;
	f.high = SOC_mismatch;
	set_header(f);
	return f;
}

//BMS_PrechargeStatus::BMS_PrechargeStatus(const Frame& frame) 

Frame BMS_PrechargeStatus::generate_frame() {
	Frame f;
	f.data[0] = driver_status;
	f.data[1] = 0x000F & (precharge_status >> 16);
	f.data[2] = 0x000F & (precharge_status >> 12);
	f.data[3] = 0x000F & (precharge_status >> 8);
	f.data[4] = 0x000F & (precharge_status >> 4);
	f.data[5] = 0x000F & precharge_status;
	f.data[6] = timer_elapsed;
	f.data[7] = precharge_timer;
}

Frame BMS_VoltageCurrent::generate_frame() {
	Frame f;
	f.low = voltage;
	f.high = current;
	set_header(f);
	return f;
}

Frame BMS_Status::generate_frame() {
	Frame f;
	f.s0 = voltage_rising;
	f.s1 = voltage_falling;
	f.data[4] = flags;
	f.data[5] = no_cmus;
	f.s3 = firmware_build;
	set_header(f);
	return f;
}

Frame BMS_FanStatus::generate_frame() {
	Frame f;
	f.s0 = fan0_speed;
	f.s1 = fan1_speed;
	f.s2 = fan_consumption;
	f.s3 = cmu_consumption;
	set_header(f);
	return f;
}

Frame MC_Heartbeat::generate_frame() {
	Frame f;
	f.low = trituim_id;
	f.high = serial_no;
	set_header(f);
	return f;
}

Frame MC_Status::generate_frame() {
	Frame f;
	f.s0 = 0;
	f.s1 = active_motor;
	f.s2 = err_flags;
	f.s3 = limit_flags;
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
	f.low = is_run ? 0x0020 : 0x0040;
	set_header(f);
	return f;
}
