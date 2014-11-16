/*
 * Packet_IDs.h
 * Constant definitions for CAN packet IDs.
 */

#ifndef PacketIDs_h
#define PacketIDs_h

// BMS TX
#define BMS_HEARTBEAT_ID	0x600
#define BMS_SOC_ID			0x6F4
#define BMS_BAL_SOC_ID		0x6F5
#define BMS_PRECHARGE_ID	0x6F7
#define BMS_VOLT_CURR_ID	0x6FA
#define BMS_STATUS_ID		0x6FB
#define BMS_FAN_STATUS_ID	0x6FC
#define BMS_STATUS_EXT_ID	0x6FD

// motor controller TX
#define MC_HEARTBEAT_ID		0x400
#define MC_STATUS_ID 		0x401
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