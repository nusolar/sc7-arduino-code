/*
 * Layouts.h
 * Definition for CAN layouts.
 */

#ifndef Layouts_h
#define Layouts_h

#include <cstdint>
#include "PacketIDs.h"
//#include "MCP2515_defs.h"

typedef struct
{
      unsigned long id;      // EID if ide set, SID otherwise
      uint8_t srr;                  // Standard Frame Remote Transmit Request
      uint8_t rtr;                  // Remote Transmission Request
      uint8_t ide;                  // Extended ID flag
      uint8_t dlc;                  // Number of data bytes
      union {
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
} Frame;

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

/*
 * Drive command packet.
 */
class DriveCmd : public Layout {
public:
	DriveCmd(float c, float v) : current(c), velocity(v) { id = DRIVE_CMD_ID; }
	DriveCmd(Frame& frame) : velocity(frame.low), current(frame.high) { id = frame.id; }

	Frame generate_frame();

	float current;
	float velocity;
};

/*
 * Motor power command packet.
 */
class PowerCmd : public Layout {
public:
	PowerCmd(float bc) : bus_current(bc) { id = POWER_CMD_ID; }
	PowerCmd(Frame& frame) : bus_current(frame.low) { id = frame.id; }

	Frame generate_frame();

	float bus_current;
};

#endif