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
 * BMS heartbeaat packet.
 */
// class BMS_Heartbeat : public Layout {
// public:
// 	BMS_Heartbeat(uint32_t d_id, uint32_t s_no) : device_id(d_id), serial_no(s_no) { id = BMS_HEARTBEAT_ID; }
// 	BMS_Heartbeat(Frame& frame) : device_id(frame.low), serial_no(frame.high) { id = frame.id; }

// 	Frame generate_frame();

// 	uint32_t device_id;
// 	uint32_t serial_no;
// };

/*
 * BMS state of charging packet.
 */
// class BMS_SOC : public Layout {
// public:
// 	BMS_SOC(uint32_t pow_cons, uint32_t per_SOC) : power_consumed(pow_cons), percent_SOC(per_SOC) { id = BMS_SOC_ID; }
// 	BMS_SOC(Frame& frame) : power_consumed(f.low), percent_SOC(f.high) { id = frame.id; }

// 	Frame generate_frame();

// 	uint32_t power_consumed;
// 	uint32_t percent_SOC;
// };

/*
 * BMS state of charging during balancing packet.
 */
// class BMS_BalanceSOC : public Layout {
// public:
// 	BMS_BalanceSOC(uint32_t pow_supp, uint32_t SOC_mis) : 
// 		power_supplied(pow_supp), SOC_mismatch(SOC_mis) 
// 		{ id = BMS_BAL_SOC_ID; }
// 	BMS_BalanceSOC(Frame& frame) : power_supplied(frame.low), SOC_mismatch(frame.high) { id = frame.id; }

// 	Frame generate_frame();

// 	uint32_t power_supplied;
// 	uint32_t SOC_mismatch;
// };

/*
 * BMS precharge status packet.
 */
// class BMS_PrechargeStatus : public Layout {
// public:
// 	BMS_PrechargeStatus(uint8_t d_status, uint64_t pc_status, uint8_t t_elapsed, uint8_t pc_timer) :
// 		driver_status(d_status), precharge_state(pc_status), timer_elapsed(t_elapsed), precharge_timer(pc_timer)
// 		{ id = BMS_PRECHARGE_ID; }
// 	BMS_PrechargeStatus(Frame& frame);

// 	Frame generate_frame();
	
// 	uint8_t driver_status;
// 	uint64_t precharge_state;
// 	uint8_t timer_elapsed;
// 	uint8_t precharge_timer;
// };

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
 * Motor controller status packet.
 */
// class MC_Status : public Layout {
// public:
// 	MC_Status(uint16_t act_m, uint16_t err_f, uint16_t lim_f) : 
// 		active_motor(act_m), err_flags(err_f), limit_flags(lim_f)
// 		{ id = MC_STATUS_ID; }
// 	MC_Status(Frame& frame) : active_motor(frame.s1), err_flags(frame.s2), limit_flags(frame.s3)
// 		{ id = frame.id; }

// 	Frame generate_frame();

// 	uint16_t active_motor;
// 	uint16_t err_flags;
// 	uint16_t limit_flags;
// };

/*
 * Motor controller bus status packet.
 */
// class MC_BusStatus : public Layout {
// public:
// 	MC_BusStatus(uint32_t bc, uint32_t bv) : bus_current(bc), bus_voltage(bv) { id = MC_BUS_STATUS_ID; }
// 	MC_BusStatus(Frame& frame) : bus_current(frame.low), bus_voltage(frame.high) { id = frame.id; }

// 	Frame generate_frame();

// 	uint32_t bus_current;
// 	uint32_t bus_voltage;
// };

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
// class MC_PhaseCurrent : public Layout {
// public:
// 	MC_PhaseCurrent(uint32_t a, uint32_t b) : phase_a(a), phase_b(b) { id = MC_PHASE_ID; }
// 	MC_PhaseCurrent(Frame& frame) : phase_a(frame.low), phase_b(frame.high) { id = frame.id; }

// 	Frame generate_frame();

// 	uint32_t phase_a;
// 	uint32_t phase_b;
// };

/*
 * Driver controls heartbeat packet.
 */
class DC_Heartbeat : public Layout {
// public:
// 	DC_Heartbeat(uint32_t d_id, uint32_t s_no) : dc_id (d_id), serial_no (s_no) { id = DC_HEARTBEAT_ID; }
// 	DC_Heartbeat(Frame& frame) : dc_id(fame.low), serial_no(frame.high) { id = frame.id }

// 	Frame generate_frame();

// 	uint32_t dc_id;
// 	uint32_t serial_no;
// };

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
// class DC_Reset : public Layout {
// public:
// 	DC_Reset() { id = DC_RESET_ID; }
// 	DC_Reset(Frame& frame) : { id = frame.id }

// 	Frame generate_frame();
// };

/*
 * Driver controls switch position packet.
 */
// class DC_SwitchPos : public Layout {
// public:
// 	DC_SwitchPos(bool run) : is_run(run) { id = DC_SWITCHPOS_ID; }
// 	DC_SwitchPos(Frame& frame) : is_run(frame.data == 0x0020) { id = frame.id; }

// 	bool is_run;
// };

#endif