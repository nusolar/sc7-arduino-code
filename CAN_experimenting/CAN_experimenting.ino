/* TODO Documentation for this file */
#define COMPILE_ARDUINO
#define MODETX
#define DEBUG

#include <SPI.h>
#include "CAN_experimenting.h"

#define CANINT 5
#define CANCS 4
#define CAN_NBT 1000  //Nominal Bit Time
#define CAN_FOSC 16

CAN_IO can(CANCS,CANINT,CAN_NBT,CAN_FOSC);
FilterInfo filters{ 0x000, 0x000, DC_DRIVE_ID, 0, BMS_HEARTBEAT_ID, 0, 0, 0 }; //Set up masks and filters. All of them 0 for now.
uint16_t errors = 0;

void setup()
{
        pinMode(12,INPUT_PULLUP); //Set up push button to toggle message transmition
	Serial.begin(9600);
        //filters.setRB0(MASK_Sxxx,DC_DRIVE_ID,0);
        //filters.setRB1(MASK_Sxxx,BMS_HEARTBEAT_ID,0,0,0);
        can.setup(filters, &errors, true);
#ifdef DEBUG
	Serial.println(errors, BIN);
#endif
}

/* For TX*/
#ifdef MODETX

void loop()
{
    if (digitalRead(12)== LOW)
    {
      digitalWrite(13,LOW);
	read_ins();
	DC_Drive packet(0,Status.current); // Create drive command, vel = 40, cur = 5;
	BMS_Heartbeat packet2(5,6); // Create power command
	can.sendCAN(packet,TXB0);
	delay(100);
	can.sendCAN(packet2,TXB0);
	delay(100);
#ifdef DEBUG
	Serial.print("TEC: ");
	Serial.println(can.controller.Read(TEC), BIN);
	Serial.print("REC: ");
	Serial.println(can.controller.Read(REC), BIN);
	Serial.print("EFLG: ");
	Serial.println(can.controller.Read(EFLG), BIN);
#endif
    digitalWrite(13,HIGH);
    }
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
	if (can.messageExists())
	{
                Frame& f = can.RXbuffer.dequeue();
                char str[50]; 
                switch (f.id)
                {
                  case DC_DRIVE_ID:
                  {
        		DC_Drive packet(f); //Get the drive packet
                        sprintf(str, "Id: %x, Vel: %d, Cur: %d,", packet.id, packet.velocity, packet.current);
        		Serial.println(str); 
                   break;
                  }
                  case BMS_HEARTBEAT_ID:
                  {
                        BMS_Heartbeat packet(f);
                        sprintf(str, "Id: %x, S_NO: %d", packet.id, packet.serial_no);
        		Serial.println(str);
                   break;
                  }
                  default:
                    Serial.println("unknown");
                    Serial.println(f.id,HEX);
                  break;
                }
#ifdef DEBUG
                Serial.print("Buffer Count:");
                Serial.println(can.RXbuffer.size());
#endif
	}
}
#endif
