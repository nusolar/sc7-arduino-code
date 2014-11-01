#include <SPI.h>

byte bits;

void setup() {
  
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(SPI_CLOCK_DIV64);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(Serial.read() != '1') ;

}


void loop() {
  // put your main code here, to run repeatedly:
  SPI.transfer(0x30); //send a Configure command to MCP3008
  byte data = SPI.transfer(0x00);
  Serial.println(data);
  delay(500);
}