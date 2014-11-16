#include <SPI.h>
#include <cstdint>

#include "MCP2515.h"
#include "MCP2515_defs.h"
#include "CAN_IO.h"
#include "Layouts.h"
#include "PacketIDs.h"

// can vars
CAN_IO can(CS_PIN, INT_PIN);
MC_Heartbeat mc_heartbeat;
MC_Velocity mc_velocity;
// state vars
const int MAX_VEL = 200;
float motor_vel_percent = 0;
float motor_current_percent = 0;
float car_velocity = 0;
float bus_current = 0;

void setup() {
  // setup can and filters
  FilterInfo filters;
  filters.RXM0 = 0xF00;
  filters.RXM1 = 0xF00;
  filters.RXF0 = 0x400;
  filters.RXF1 = 0x400;
  filters.RXF2 = 0x400;
  filters.RXF3 = 0x400;
  filters.RXF4 = 0x400;
  filters.RXF5 = 0x400;
  can.setup(filters);
}

void loop() {
  // update state vars
  car_velocity = motor_vel_percent/100.0f*MAX_VEL;
  
  // send heartbeat and velocity packets
  mc_heartbeat = MC_Heartbeat(1,2);
  mc_velocity = MC_Velocity(car_velocity, car_velocity);
  can.send_CAN(mc_heartbeat);
  can.send_CAN(mc_velocity);
  
  // wait 500 ms
  delay(500);
}
