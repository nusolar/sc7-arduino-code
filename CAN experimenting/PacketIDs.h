/*
 * Packet_IDs.h
 * Constant definitions for CAN packet IDs.
 */

#ifndef PacketIDs_h
#define PacketIDs_h

// motor controller TX
#define MC_HEARTBEAT_ID		0x400
#define MC_VELOCITY_ID		0x403

// driver controls TX
#define DC_HEARTBEAT_ID		0x500
#define DRIVE_CMD_ID		0x501
#define POWER_CMD_ID		0x502
#define RESET_CMD_ID		0x503

#endif