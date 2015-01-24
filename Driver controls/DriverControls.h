/*
 * DriverControls.h
 * Contains definition for the DriverControls class.
 */

#include <cstdint>
#include "Metro.h"
#include "CAN_IO.h"
#include "WDT.h"

/*
 * Struct that holds information about the car state.
 */
struct CarState {
 	bool brakeEngaged;
 	uint16_t accelPedal;
 	uint16_t regenPedal;

 	uint16_t motorVelocity;
 	float carVelocity;
 	uint16_t busCurrent;
};

/*
 * DriverControls class. Responsible for maintaining the
 * overall state of the car and coordinating various
 * car systems.
 */
class DriverControls {
public:
private:
	// CAN variables
	CAN_IO can;
	byte interrupt;
	byte chipSelect;

	// inputs/outputs
	byte brakePin;
	byte accelPin;
	byte regenPin;

	// car state
	CarState state;

	// timers
	Metro mcHeartbeatTimer;
	Metro swHeartbeatTimer;
	Metro mcSendTimer;
	Metro bmsSendTimer;
	Metro dcSendTimer;


};