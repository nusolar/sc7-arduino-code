/*
 * Packet_IDs.h
 * Constant definitions for CAN packet IDs.
 */

#ifndef PacketIDs_h
#define PacketIDs_h

// motor controller TX
#define MC_HEARTBEAT_ID		0x400
#define MC_BUS_STATUS_ID	0x402
#define MC_VELOCITY_ID		0x403
#define MC_PHASE_ID			0x404

// driver controls TX
#define DC_HEARTBEAT_ID		0x500
#define DC_DRIVE_ID			0x501
#define DC_POWER_ID			0x502
#define DC_RESET_ID			0x503
#define DC_SWITCHPOS_ID		0x505

#endif