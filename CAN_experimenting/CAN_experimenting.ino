/* TODO Documentation for this file */
#define COMPILE_ARDUINO
//#define DEBUG
#define LOOPBACK

#include <SPI.h>
#include "CAN_experimenting.h"

#define CANINT 8
#define CANCS 5
#define CAN_NBT 1000  //Nominal Bit Time
#define CAN_FOSC 16

#define PEDAL_ACCL_MAX_CURRENT 0.8f

CAN_IO can(CANCS,CANINT,CAN_NBT,CAN_FOSC);
uint16_t errors = 0; // for catching can errors
long lastmillis = 0; // for timing 
int switchpos = 0x0020; // run position

void setup()
{
    //Set up push button to toggle message transmission
    pinMode(12,INPUT_PULLUP); 
	
    //Start Serial and wait for MCP2515 to finish 128 clock cycles
    Serial.begin(9600);
    Serial.print("BEGIN");
    delay(100); 
	
    //create a filter options structure
    CANFilterOpt filter;
    filter.setRB0(MASK_NONE,DC_DRIVE_ID,0);
    filter.setRB1(MASK_NONE,DC_SWITCHPOS_ID,0,0,0);
    can.Setup(filter, &errors);
    
#ifdef LOOPBACK
      can.controller.Mode(MODE_LOOPBACK);
#endif
    
#ifdef DEBUG
	Serial.println(errors, BIN);
#endif
}

void loop()
{
  //Generate Random Can Packets and Send them
  int choice = random(0,10);
  Serial.print(choice);
  switch(choice)
  {
    case 0:{
      can.Send(DC_Heartbeat(100.0,42.0),TXB0);
    break;}
    case 1:{
      can.Send(DC_Drive(100.0,random(80)/100.0),TXB0);
    break;}
    case 2:{
      can.Send(BMS_Heartbeat(400.0,99.0),TXB0);
    break;}
    case 3:
    default:{ //Move down later
      float cvel = random(30);
      float mvel = cvel + random(-3,3);
      can.Send(MC_Velocity(cvel,mvel),TXB0);
    break;}
  }
  delay(200);
    
#ifdef DEBUG
	Serial.print("TEC: ");
	Serial.println(can.controller.Read(TEC), BIN);
	Serial.print("REC: ");
	Serial.println(can.controller.Read(REC), BIN);
	Serial.print("EFLG: ");
	Serial.println(can.controller.Read(EFLG), BIN);
#endif
	while (can.Available())
	{
		Frame& f = can.Read();
		char str[50];
		switch (f.id)
		{
		  case DC_DRIVE_ID:
		  {
			DC_Drive packet(f); //Get the drive packet
			sprintf(str, "Id: %x, Vel: %.1f, Cur: %.4f,", packet.id, packet.velocity, packet.current);
			Serial.println(str); 
			break;
		  }
		  case MC_VELOCITY_ID:
		  {
			MC_Velocity packet(f);
			sprintf(str, "Id: %x, CarVel: %f, MotVel: %f", packet.id, packet.car_velocity, packet.motor_velocity);
			Serial.println(str);
			break;
		  }
                  default:
                  {
                        Serial.print("Id: ");
                        Serial.println(f.id,HEX);
                        break;
                  }
	         }
		
		//Print out buffer size so we can see if there is overflow (this is not accurate when serial is enabled.
		if (millis() > lastmillis + 1000)
		{
                  lastmillis = millis();
		  Serial.print("Buffer Count:");
		  Serial.println(can.RXbuffer.size());
		}
	}
}

//Function for reading "pedal" data
void read_ins()
{
	Status.raw_pedal = analogRead(pedalPin);
	Status.current = constrain(Status.raw_pedal / 710.0 , 0.0f, PEDAL_ACCL_MAX_CURRENT);
}

