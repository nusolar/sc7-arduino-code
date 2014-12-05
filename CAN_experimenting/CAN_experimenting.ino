/* TODO Documentation for this file */
#define COMPILE_ARDUINO
#define MODERX

#include <SPI.h>
#include "CAN_IO.h"
#include "RX_Queue.h"

#define pedalPin A10

CAN_IO can(4, 5);
FilterInfo filters{ 0xFFF, 0xFFF, DC_DRIVE_ID, 0, BMS_HEARTBEAT_ID, 0, 0, 0 }; //Set up masks and filters. All of them 0 for now.
uint16_t errors = 0;

struct
{
	uint16_t raw_pedal;
	float current;
} Status;

void setup()
{
	Serial.begin(9600);
        //filters.setRB0(MASK_Sxxx,DC_DRIVE_ID,0);
        //filters.setRB1(MASK_Sxxx,BMS_HEARTBEAT_ID,0,0,0);
        can.setup(filters, &errors, true);
	Serial.println(errors, BIN);
}

/* For TX*/
#ifdef MODETX
void loop()
{
	read_ins();
	DC_Drive packet(0,Status.current); // Create drive command, vel = 40, cur = 5;
	BMS_Heartbeat packet2(5,6); // Create power command
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
	if (can.messageExists())
	{
                Frame f = can.buffer.dequeue();
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
                    Serial.println(f.id);
                  break;
                }
	}
}
#endif

