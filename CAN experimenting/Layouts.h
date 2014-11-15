/*
 * Layouts.h
 * Definition for CAN layouts.
 */

#ifndef Layouts_h
#define Layouts_h

#include <cstdint>
#include "PacketIDs.h"
#include "MCP2515_defs.h"

/*
 * Abstract base packet.
 */
class Layout {
public:
	uint16_t id;
	
	/*
	 * Creates a Frame object to represent this layout.
	 */
	virtual Frame generate_frame();
protected:
	/*
	 * Fill out the header info for a frame.
	 */
	void set_header(Frame& f);
};

/*
 * Motor controller heartbeat packet.
 */
class MC_Heartbeat : public Layout {
public:
	MC_Heartbeat(uint32_t t_id, uint32_t s_no) : trituim_id(t_id), serial_no(s_no) { id = MC_HEARTBEAT_ID; }
	MC_Heartbeat(Frame& frame) : trituim_id(frame.low), serial_no(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t trituim_id;
	uint32_t serial_no;
};

/*
 * Motor controller bus status packet.
 */
class MC_BusStatus : public Layout {
public:
	MC_BusStatus(uint32_t bc, uint32_t bv) : bus_current(bc), bus_voltage(bv) { id = MC_BUS_STATUS_ID; }
	MC_BusStatus(Frame& frame) : bus_current(frame.low), bus_voltage(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t bus_current;
	uint32_t bus_voltage;
};

/*
 * Motor controller velocity packet.
 */
class MC_Velocity : public Layout {
public:
	MC_Velocity(uint32_t car_v, uint32_t motor_v) : car_velocity(car_v), motor_velocity(motor_v) { id = MC_VELOCITY_ID; }
	MC_Velocity(Frame& frame) : car_velocity(frame.low), motor_velocity(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t car_velocity;
	uint32_t motor_velocity;
};

/*
 * Motor controller motor phase current packet.
 */
class MC_PhaseCurrent : public Layout {
	MC_PhaseCurrent(uint32_t a, uint32_t b) : phase_a(a), phase_b(b) { id = MC_PHASE_ID; }
	MC_PhaseCurrent(Frame& frame) : phase_a(frame.low), phase_b(frame.high) { id = MC_PHASE_ID; }

	Frame generate_frame();

	uint32_t phase_a;
	uint32_t phase_b;
};

/*
 * Driver controls heartbeat packet.
 */
class DC_Heartbeat : public Layout {
public:
	DC_Heartbeat(uint32_t d_id, uint32_t s_no) : dc_id (d_id), serial_no (s_no) { id = DC_HEARTBEAT_ID; }
	DC_Heartbeat(Frame& frame) : dc_id(fame.low), serial_no(frame.high) { id = frame.id }

	Frame generate_frame();

	uint32_t dc_id;
	uint32_t serial_no;
};

/*
 * Driver controls drive command packet.
 */
class DC_Drive : public Layout {
public:
	DC_Drive(uint32_t v, uint32_t c) : current(c), velocity(v) { id = DC_DRIVE_ID; }
	DC_Drive(Frame& frame) : velocity(frame.low), current(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t current;
	uint32_t velocity;
};

/*
 * Driver controls power command packet.
 */
class DC_Power : public Layout {
public:
	DC_Power(uint32_t bc) : bus_current(bc) { id = DC_POWER_ID; }
	DC_Power(Frame& frame) : bus_current(frame.low) { id = frame.id; }

	Frame generate_frame();

	uint32_t bus_current;
};

/*
 * Driver controls reset packet.
 */
class DC_Reset : public Layout {
public:
	DC_Reset() { id = DC_RESET_ID; }
	DC_Reset(Frame& frame) : { id = frame.id }

	Frame generate_frame();
};

/*
 * Driver controls switch position packet.
 */
class DC_SwitchPos : public Layout {
public:
	DC_SwitchPos(bool run) : is_run(run) { id = DC_SWITCHPOS_ID; }
	DC_SwitchPos(Frame& frame) : is_run(frame.data == 0x0020) { id = DC_SWITCHPOS_ID; }

	bool is_run;
};

#endif