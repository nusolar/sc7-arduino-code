/*
 * Layouts.h
 * Definition for CAN layouts.
 */

#ifndef Layouts_h
#define Layouts_h

#include <stdint.h>
#include "MCP2515_defs.h"

/*
 * Packet_IDs.h
 * Constant definitions for CAN packet IDs.
 */

// BMS TX
#define BMS_HEARTBEAT_ID	0x600
#define BMS_SOC_ID	        0x6F4
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

// Mask ID that specifically work with our SIDs
#define MASK_NONE			0x000
#define MASK_Sx00			0x700
#define MASK_Sxx0			0x7F0
#define MASK_Sxxx			0x7FF
#define MASK_EID			0x7FFFF


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
 * BMS heartbeaat packet.
 */
class BMS_Heartbeat : public Layout {
public:
	BMS_Heartbeat(uint32_t d_id, uint32_t s_no) : device_id(d_id), serial_no(s_no) { id = BMS_HEARTBEAT_ID; }
	BMS_Heartbeat(const Frame& frame) : device_id(frame.low), serial_no(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t device_id;
	uint32_t serial_no;
};

/*
 * BMS state of charging packet.
 */
class BMS_SOC : public Layout {
public:
	BMS_SOC(uint32_t pow_cons, uint32_t per_SOC) : power_consumed(pow_cons), percent_SOC(per_SOC) { id = BMS_SOC_ID; }
	BMS_SOC(const Frame& frame) : power_consumed(frame.low), percent_SOC(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t power_consumed;
	uint32_t percent_SOC;
};

/*
 * BMS state of charging during balancing packet.
 */
class BMS_BalanceSOC : public Layout {
public:
	BMS_BalanceSOC(uint32_t pow_supp, uint32_t SOC_mis) : 
		power_supplied(pow_supp), SOC_mismatch(SOC_mis) 
		{ id = BMS_BAL_SOC_ID; }
	BMS_BalanceSOC(const Frame& frame) : power_supplied(frame.low), SOC_mismatch(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t power_supplied;
	uint32_t SOC_mismatch;
};

/*
 * BMS precharge status packet.
 */
class BMS_PrechargeStatus : public Layout {
public:
	BMS_PrechargeStatus(uint8_t d_status, uint64_t pc_status, uint8_t t_elapsed, uint8_t pc_timer) :
		driver_status(d_status), precharge_status(pc_status), timer_elapsed(t_elapsed), precharge_timer(pc_timer)
		{ id = BMS_PRECHARGE_ID; }
	BMS_PrechargeStatus(const Frame& frame) {
	id = frame.id;
	driver_status = frame.data[0];
	timer_elapsed = frame.data[6];
	precharge_timer = frame.data[7];
	precharge_status = 0x0000 | (frame.data[1] << 16) | (frame.data[2] << 12) | (frame.data[3] << 8) | (frame.data[4] << 4) | frame.data[5];
}

	Frame generate_frame();

	uint8_t driver_status;
	uint64_t precharge_status;
	uint8_t timer_elapsed;
	uint8_t precharge_timer;
};

/*
 * BMS voltage and current packet.
 */
class BMS_VoltageCurrent : public Layout {
public:
	BMS_VoltageCurrent(uint32_t v, uint32_t c) : voltage(v), current(c) { id = BMS_VOLT_CURR_ID; }
	BMS_VoltageCurrent(const Frame& frame) : voltage(frame.low), current(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t voltage;
	uint32_t current;
};

/*
 * BMS status packet.
 */
class BMS_Status : public Layout {
public:
	BMS_Status(uint16_t v_rising, uint16_t v_falling, uint8_t flg, uint8_t cmus, uint16_t firmware) :
		voltage_rising(v_rising), voltage_falling(v_falling), flags(flg), no_cmus(cmus), firmware_build(firmware)
		{ id = BMS_STATUS_ID; }
	BMS_Status(const Frame& frame) : voltage_rising(frame.s0), voltage_falling(frame.s1), 
		flags(frame.data[4]), no_cmus(frame.data[5]), firmware_build(frame.s3)
		{ id = frame.id; }

	Frame generate_frame();

	uint16_t voltage_rising;
	uint16_t voltage_falling;
	uint8_t flags;
	uint8_t no_cmus;
	uint16_t firmware_build;

};

/*
 * BMS fan status packet.
 */
class BMS_FanStatus : public Layout {
public:
	BMS_FanStatus(uint16_t f0_speed, uint16_t f1_speed, uint16_t fan_c, uint16_t cmu_c) :
		fan0_speed(f0_speed), fan1_speed(f1_speed), fan_consumption(fan_c), cmu_consumption(cmu_c)
		{ id = BMS_FAN_STATUS_ID; }
	BMS_FanStatus(const Frame& frame) : fan0_speed(frame.s0), fan1_speed(frame.s1), 
		fan_consumption(frame.s2), cmu_consumption(frame.s3)
		{ id = frame.id; }

	Frame generate_frame();

	uint16_t fan0_speed;
	uint16_t fan1_speed;
	uint16_t fan_consumption;
	uint16_t cmu_consumption;
};

/*
 * BMS extended status packet.
 */
class BMS_StatusExt : public Layout {
	// Implement! Need specs.
};

/*
 * Motor controller heartbeat packet.
 */
class MC_Heartbeat : public Layout {
public:
	MC_Heartbeat(uint32_t t_id, uint32_t s_no) : trituim_id(t_id), serial_no(s_no) { id = MC_HEARTBEAT_ID; }
	MC_Heartbeat(const Frame& frame) : trituim_id(frame.low), serial_no(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t trituim_id;
	uint32_t serial_no;
};

/*
 * Motor controller status packet.
 */
class MC_Status : public Layout {
public:
	MC_Status(uint16_t act_m, uint16_t err_f, uint16_t lim_f) : 
		active_motor(act_m), err_flags(err_f), limit_flags(lim_f)
		{ id = MC_STATUS_ID; }
	MC_Status(const Frame& frame) : active_motor(frame.s1), err_flags(frame.s2), limit_flags(frame.s3)
		{ id = frame.id; }

	Frame generate_frame();

	uint16_t active_motor;
	uint16_t err_flags;
	uint16_t limit_flags;
};

/*
 * Motor controller bus status packet.
 */
class MC_BusStatus : public Layout {
public:
	MC_BusStatus(uint32_t bc, uint32_t bv) : bus_current(bc), bus_voltage(bv) { id = MC_BUS_STATUS_ID; }
	MC_BusStatus(const Frame& frame) : bus_current(frame.low), bus_voltage(frame.high) { id = frame.id; }

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
	MC_Velocity(const Frame& frame) : car_velocity(frame.low), motor_velocity(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t car_velocity;
	uint32_t motor_velocity;
};

/*
 * Motor controller motor phase current packet.
 */
class MC_PhaseCurrent : public Layout {
public:
	MC_PhaseCurrent(uint32_t a, uint32_t b) : phase_a(a), phase_b(b) { id = MC_PHASE_ID; }
	MC_PhaseCurrent(const Frame& frame) : phase_a(frame.low), phase_b(frame.high) { id = frame.id; }

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
	DC_Heartbeat(Frame& frame) : dc_id(frame.low), serial_no(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t dc_id;
	uint32_t serial_no;
};

/*
 * Driver controls drive command packet.
 */
class DC_Drive : public Layout {
public:
	DC_Drive(uint32_t v, uint32_t c) : velocity(v), current(c) { id = DC_DRIVE_ID; }
	DC_Drive(const Frame& frame) : velocity(frame.low), current(frame.high) { id = frame.id; }

	Frame generate_frame();

	uint32_t velocity;
	uint32_t current;
};

/*
 * Driver controls power command packet.
 */
class DC_Power : public Layout {
public:
	DC_Power(uint32_t bc) : bus_current(bc) { id = DC_POWER_ID; }
	DC_Power(const Frame& frame) : bus_current(frame.low) { id = frame.id; }

	Frame generate_frame();

	uint32_t bus_current;
};

/*
 * Driver controls reset packet.
 */
class DC_Reset : public Layout {
public:
	DC_Reset() { id = DC_RESET_ID; }
	DC_Reset(const Frame& frame) { id = frame.id; }

	Frame generate_frame();
};

/*
 * Driver controls switch position packet.
 */
class DC_SwitchPos : public Layout {
public:
	DC_SwitchPos(bool run) : is_run(run) { id = DC_SWITCHPOS_ID; }
	DC_SwitchPos(const Frame& frame) : is_run((frame.low == 0x0020)) { id = frame.id; }

	bool is_run;

        Frame generate_frame();
};

#endif
