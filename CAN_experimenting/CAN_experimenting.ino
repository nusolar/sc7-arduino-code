#define COMPILE_ARDUINO

#include <SPI.h>
#include "CAN_IO.h"

CAN_IO can(4,5);
FilterInfo filters {0xFF0,0xFFF, 0x500,0,0,0,0,0}; //Set up masks and filters. All of them 0 for now.
byte errors = 0;

void setup()
{
  Serial.begin(9600);
  can.setup(filters, errors);
  Serial.println(errors, BIN);
  
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
  Serial.print("TEC: ");
  Serial.println(can.controller.Read(TEC), BIN);
  Serial.print("REC: ");
  Serial.println(can.controller.Read(REC), BIN);
  Serial.print("EFLG: ");
  Serial.println(can.controller.Read(EFLG), BIN);
}*/

/* For RX*/
void loop()
{
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
    Serial.print("errors: ");
    Serial.println(errors,BIN);
    Serial.print("TEC: ");
    Serial.println(can.controller.Read(TEC), BIN);
    Serial.print("REC: ");
    Serial.println(can.controller.Read(REC), BIN);
    Serial.print("EFLG: ");
    Serial.println(can.controller.Read(EFLG), BIN);
    delay(250);
  }
}
  
