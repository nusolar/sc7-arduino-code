/* TODO Documentation for this file */
#define COMPILE_ARDUINO
//#define DEBUG

#include <SPI.h>
#include "CAN_experimenting.h"

#define CANINT 5
#define CANCS 4
#define CAN_NBT 1000  //Nominal Bit Time
#define CAN_FOSC 16

#define PEDAL_ACCL_MAX_CURRENT 0.8f

CAN_IO can(CANCS,CANINT,CAN_NBT,CAN_FOSC);
uint16_t errors = 0; // for catching can errors
long lastmillis = 0; // for timing 

void setup()
{
    //Set up push button to toggle message transmission
    pinMode(12,INPUT_PULLUP); 
	
    //Start Serial and wait for MCP2515 to finish 128 clock cycles
    Serial.begin(9600);
    delay(100); 
	
    //create a filter options structure
    CANFilterOpt filter;
    filter.setRB0(MASK_Sxxx,DC_DRIVE_ID,0);
    filter.setRB1(MASK_Sxxx,BMS_SOC_ID,0,0,0);
    can.setup(filter, &errors);
#ifdef DEBUG
	Serial.println(errors, BIN);
#endif
}

void loop()
{
    if (digitalRead(12)== LOW) // if the transmit button is pressed
    {
		read_ins();
		DC_Drive packet(100.0,Status.current); // Create drive command
		can.sendCAN(packet,TXB0);
		delay(100);
#ifdef DEBUG
		Serial.print("TEC: ");
		Serial.println(can.controller.Read(TEC), BIN);
		Serial.print("REC: ");
		Serial.println(can.controller.Read(REC), BIN);
		Serial.print("EFLG: ");
		Serial.println(can.controller.Read(EFLG), BIN);
#endif
    }
	
	if (can.messageExists())
	{
		Frame& f = can.messageDequeue();
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
		  case BMS_SOC_ID:
		  {
			BMS_SOC packet(f);
			sprintf(str, "Id: %x, Pack: %.4f", packet.id, packet.percent_SOC);
			Serial.println(str);
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

