#define COMPILE_ARDUINO

#include <SPI.h>
#include "CAN_IO.h"

CAN_IO can(4,5);
FilterInfo filters {0xFF8,0, DC_DRIVE_ID,0,0,0,0,0}; //Set up masks and filters. All of them 0 for now.

void setup()
{
  can.setup(filters);
  Serial.begin(9600);
  
}

/* For TX
void loop()
{
  DC_Drive packet(40,5); // Create drive command, vel = 40, cur = 5;
  DC_Power packet2(30); // Create power command
  can.send_CAN(packet);
  can.send_CAN(packet2);
  delay(500);
}*/

/* For RX*/
void loop()
{
  byte errors = 0x0000;
  if (digitalRead(5) == LOW)
  {
     can.receive_CAN(errors); //Reads CAN data into buffer
  }
  
  if (can.messageavailable) {
    DC_Drive packet(can.buffer[can.buffer_index]); //Get the drive packet
    Serial.println(packet.velocity);
    Serial.println(packet.current);
    delay(500);
  }
  else
  {
    Serial.println("NO MESSAGE");
  }
}
  
