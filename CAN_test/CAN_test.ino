#include <SPI.h>

#include "MCP2515.h"
#include "MCP2515_defs.h"

byte bits;

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

}


void loop() {
  // put your main code here, to run repeatedly:
  bits = CCAN.Read(CANCTRL);
Serial.println(bits);
bits = 0;
delay(500);    
}
