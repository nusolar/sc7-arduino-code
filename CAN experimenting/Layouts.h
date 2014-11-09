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
	/*
	 * Creates a Frame object to represent this layout.
	 */
	virtual Frame generate_frame();

	uint16_t id;
protected:
	/*
	 * Fill out the header info for a frame.
	 */
	void set_header(Frame& f);
};

class MC_Heartbeat : public Layout {
	MC_Heartbeat(uint32_t t_id, uint32_t s_no) : trituim_id(t_id), serial_no(s_no) { id = MC_HEARTBEAT_ID; }
	MC_Heartbeat(Frame& frame) : trituim_id(frame.low), serial_no(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t trituim_id;
	uint32_t serial_no;
};

class MC_Velocity : public Layout {
	MC_Velocity(uint32_t car_v, uint32_t motor_v) : car_velocity(car_v), motor_velocity(motor_v) { id = MC_VELOCITY_ID; }
	MC_Velocity(Frame& frame) : car_velocity(frame.low), motor_velocity(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t car_velocity;
	uint32_t motor_velocity;
};

/*
 * Driver controls drive command packet.
 */
class DriveCmd : public Layout {
public:
	DriveCmd(float v, float c) : current(c), velocity(v) { id = DRIVE_CMD_ID; }
	DriveCmd(Frame& frame) : velocity(frame.low), current(frame.high) { id = frame.id; }

	Frame generate_frame();

	float current;
	float velocity;
};

/*
 * Driver controls power command packet.
 */
class PowerCmd : public Layout {
public:
	PowerCmd(float bc) : bus_current(bc) { id = POWER_CMD_ID; }
	PowerCmd(Frame& frame) : bus_current(frame.low) { id = frame.id; }

	Frame generate_frame();

	float bus_current;
};

#endif