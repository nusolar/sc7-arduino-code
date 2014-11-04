#include <SPI.h>

#include "MCP2515.h"
#include "MCP2515_defs.h"

byte bits;
Frame f;
/*
typedef struct
{
      unsigned long id;      // EID if ide set, SID otherwise
      byte srr;                  // Standard Frame Remote Transmit Request
      byte rtr;                  // Remote Transmission Request
      byte ide;                  // Extended ID flag
      byte dlc;                  // Number of data bytes
      byte data[8];            // Data bytes
} Frame;
*/

MCP2515 CCAN(4,5);
void setup() {
  
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(10);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(Serial.read() != '1') ;
  Serial.write(CCAN.Init(125,20) + '0');
  f.id=0x011;
  f.srr = 0;
  f.rtr = 0;
  f.ide = 0;
  f.dlc = 2;
  f.data[0] = 5;
  f.data[1] = 10;
}


void loop() {
  // put your main code here, to run repeatedly:
  /*bits = CCAN.Read(CANCTRL);
Serial.println(bits);
bits = 0;*/
CCAN.LoadBuffer(TXB0,f);
CCAN.SendBuffer(TXB0);
delay(500);
}
