/*
 * Layouts.h
 * Definition for CAN layouts.
 */

#ifndef Layouts_h
#define Layouts_h

#include "MCP2515_defs.h"

/*
 * Abstract base packet.
 */
class Layout {
public:
	/*
	 * Creates a Frame object to represent this layout.
	 */
	Frame generate_frame();

	uint16_t id;
protected:
	/*
	 * Fill out the header info for a frame.
	 */
	Frame set_header(Frame& f);
};

/*
 * Drive command packet.
 */
class DriveCmd : Layout {
public:
	/*
	 * Initializes the DriveCmd with current c and velocity v.
	 * Used by the sender.
	 */
	DriveCmd(float c, float v) : id(DRIVE_CMD_ID), current(c), velocity(v) {}

	/*
	 * Initializes a DriveCmd from the given Frame. Used by the receiver.
	 */
	DriveCmd(Frame& frame);

	float current;
	float velocity;
};

/*
 * Motor power command packet.
 */
class PowerCmd : Layout {
public:
	PowerCmd(float busCurrent);
	PowerCmd(Frame& frame);

	float busCurrent;
};

/*
 * Motor velocity packet.
 */
class MotorVelCmd : Layout {
public:
	MotorVelCmd(float carVelocity, float motorVelocity);
	MotorVelCmd(Frame& frame);

	float carVelocity;
	float motorVelocity;
};

class BusStateCmd : Layout {
public:
	BusStateCmd(float busCurrent, float busVoltage);

	float busCurrent;
	float busVoltage;
};

#endif