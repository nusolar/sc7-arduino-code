/*
 * DriverControls.h
 * Contains the definition for the DriverControls class.
 */

#ifndef DriverControls_h
#define DriverControls_h

/*
 * Enum for BMS state information.
 */
enum Precharge {
	Precharge__Error = 0,
	Precharge__Idle = 1,
	Precharge__EnablePack = 5,
	Precharge__Measure = 2,
	Precharge__Precharge = 3,
	Precharge__Run = 4
};

/*
 * Struct for car state information.
 */
struct CarState {
	// pedals & derived data
	uint16_t accel_raw;
	uint16_t accel_safe;
	bool brake_en;
	float accel_motor;

	// ignition & gears
	bool key_run;
	bool drive_en;
	bool reverse_en;
	bool regen_en;

	// signals
	bool lt_left;
	bool lt_right;
	bool lt_heads;
	bool horn;

	// car state
	Precharge bms_state;

	// optional systems
	bool using_hardware_switches;
	float vehicle_velocity;
	float bus_voltage, bus_current;
};

/*
 * Class responsible for driver-related controls, such as lights
 * and accleration, as well as overall control of the car.
 */
class DriverControls {
public:
	/*
	 * Constructor. Initialize state and I/O.
	 */
	DriverControls();

	/*
	 * Reads the CAN bus, retrieving relevant information.
	 * Updates the car state and takes action if necessary.
	 */
	void readCAN();

	/*
	 * Sends driver control packets out over the CAN bus,
	 * including motor commands and display updates.
	 */
	void sendCAN();
private:
	CarState state;
};

#endif