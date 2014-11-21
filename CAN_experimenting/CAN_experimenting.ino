#define COMPILE_ARDUINO

#include <SPI.h>
#include "CAN_IO.h"

CAN_IO can(4,5);
FilterInfo filters {0x0FF0,0x0FFF, 0x501,0,0,0,0,0}; //Set up masks and filters. All of them 0 for now.

void setup()
{
  Serial.begin(9600);
  can.setup(filters);
  
  // Read Filter bits for RB0 to make sure that they are correct.
}

/* For TX
void loop()
{
  DC_Drive packet(40,5); // Create drive command, vel = 40, cur = 5;
  DC_Power packet2(30); // Create power command
  can.send_CAN(packet);
  delay(500);
  can.send_CAN(packet2);
  delay(500);
}*/

/* For RX*/
void loop()
{
  byte errors = 0x0000;
  if (digitalRead(5) == LOW)
  {
     Serial.println(can.controller.RXStatus(),BIN);
     can.receive_CAN(errors); //Reads CAN data into buffer
     
  }
  
  if (can.messageavailable) {
    DC_Drive packet(can.buffer[can.buffer_index]); //Get the drive packet
    Serial.println(packet.velocity);
    Serial.println(packet.current);
    Serial.println(packet.id,HEX);
    delay(250);
    can.messageavailable = false;
  }
  else
  {
    Serial.println("NO MESSAGE");
    Serial.println(errors,BIN);
    delay(250);
  }
}
  
