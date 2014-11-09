/*
 * Layouts.h
 * Definition for CAN layouts.
 */

#ifndef Layouts_h
#define Layouts_h

#include "MCP2515_defs.h"

/*
 * Union for accessing the data of a CAN packet.
 * Data can be accessed as one-byte, two-byte,
 * four-byte, or eight-byte chuncks.
 */
union LayoutUnion {
	// 8 bytes
	uint64_t value;
	// 4 bytes
  	struct {
    	uint32_t low;
    	uint32_t high;
  	};
  	// 2 bytes
  	struct {
        uint16_t s0;
    	uint16_t s1;
    	uint16_t s2;
    	uint16_t s3;
    };
    // 1 byte
  	uint8_t data[8];
};

/*
 * Abstract base packet.
 */
class Layout {
public:
	//LayoutUnion data;
	uint16_t id;
	Frame generate_frame();
protected:
	void copy_data(uint8_t* source, uint8_t* destination);
};

/*
 * Drive command packet.
 */
class DriveCmd : Layout {
public:
	DriveCmd(float current, float velocity);
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