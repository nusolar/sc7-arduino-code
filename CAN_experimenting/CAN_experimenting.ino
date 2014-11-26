/* TODO Documentation for this file */
#define COMPILE_ARDUINO
#define MODETX

#include <SPI.h>
#include "CAN_IO.h"
#include "RX_Queue.h"

#define pedalPin A10

CAN_IO can(4,5);
FilterInfo filters {0xFFF,0xFFF, DC_DRIVE_ID,0,0,0,0,0}; //Set up masks and filters. All of them 0 for now.
byte errors = 0;

struct 
{
	uint16_t raw_pedal;
	float current;
} Status;

void setup()
{
  Serial.begin(9600);
  can.setup(filters, errors);
  Serial.println(errors, BIN);
  
  /* Queue Testing Code -- Works 11/26/14 */
  RX_Queue testqueue;
  
  for (int i = 0; i < 10; i++)
  {
	  testqueue.enqueue(DC_Drive(40,i).generate_frame());
  }

  for(int i = 0; !testqueue.is_empty(); i++)
  {
    Serial.println(testqueue.dequeue().high);
  }
  
}

/* For TX*/
#ifdef MODETX
void loop()
{
  read_ins();
  DC_Drive packet(0,Status.current); // Create drive command, vel = 40, cur = 5;
  DC_Power packet2(30); // Create power command
  can.send_CAN(packet);
  delay(100);
  can.send_CAN(packet2);
  delay(100);
  /*Serial.print("TEC: ");
  Serial.println(can.controller.Read(TEC), BIN);
  Serial.print("REC: ");
  Serial.println(can.controller.Read(REC), BIN);
  Serial.print("EFLG: ");
  Serial.println(can.controller.Read(EFLG), BIN);*/
}

void read_ins()
{
	Status.raw_pedal = analogRead(pedalPin);
	Status.current = Status.raw_pedal*100.0 / 750.0;
}
#endif

/* For RX */
#ifdef MODERX
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
    /*Serial.print("TEC: ");
    Serial.println(can.controller.Read(TEC), BIN);
    Serial.print("REC: ");
    Serial.println(can.controller.Read(REC), BIN);
    Serial.print("EFLG: ");
    Serial.println(can.controller.Read(EFLG), BIN);*/
    delay(250);
  }
}
#endif
  
