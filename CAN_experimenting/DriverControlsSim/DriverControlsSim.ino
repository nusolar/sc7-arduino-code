#include <SPI.h>
#include <cstdint>

#include "MCP2515.h"
#include "MCP2515_defs.h"
#include "CAN_IO.h"
#include "Layouts.h"
#include "PacketIDs.h"

// can vars
CAN_IO can(CS_PIN, INT_PIN);
DC_Drive drive_cmd;
DC_Power power_cmd;

// state vars
float motor_velocity;
float motor_current;
float bus_current;
float car_velocity;
int increment;

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
  
  // init state vars
  motor_velocity = 0;
  motor_current = 0;
  bus_current = 0;
  car_velocity = 0;
  increment = 1;
}

void loop() {
  // send drive and power packets
  drive_cmd = DC_Drive(motor_velocity, motor_current);
  power_cmd = DC_Power(bus_current);
  can.send_CAN(drive_cmd);
  can.send_CAN(power_cmd);
  
  // change state vars
  motor_velocity += increment;
  motor_current += increment;
  bus_current += increment;
  if (motor_velocity >= 100) {
    increment *= -1;
  }
  
  // wait 500 ms
  delay(500);
}
