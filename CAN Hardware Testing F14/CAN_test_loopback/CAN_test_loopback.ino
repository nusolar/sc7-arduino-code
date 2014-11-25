/* LOOPBACK TEST FOR ARDUINO-MCP2515 CAN TXRX
- Alexander Martin, Fall 2014 -
This code puts the MCP2515 in Loopback Mode, where the TX and RX pins are
internally connected. 5 and 10 are sent in data[0] and data[1] bytes in a
CAN frame and received/printed upon reception.

NOTE: Will only work fresh after a power cycle. Reseting the Arduino w/ Serial
or Reset button will cause 2515 to error.
*/

#include <SPI.h>

#include "MCP2515.h"
#include "MCP2515_defs.h"

#include<bitset>
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
  Serial.write(CCAN.Init(125,16) + '0');
  f.id=0x011;
  f.srr = 0;
  f.rtr = 0;
  f.ide = 0;
  f.dlc = 2;
  f.data[0] = 5;
  f.data[1] = 10;
  
  // Place chip in Loopback mode
  Serial.write(CCAN.Mode(MODE_NORMAL)+'0');
  Serial.print((byte)CCAN.Read(CANSTAT));
}

void loop() {
    // Send a Can Message
    CCAN.LoadBuffer(TXB0,f);
    CCAN.SendBuffer(TXB0);
   
    // Wait for a while to give MCP2515 time to process looped signal
    delay(1);
    /*while (CCAN.Interrupt() == false)
    {
      // Check for high interrupt pin
      if (millis()%1000 == 0) 
      {
        Serial.print("Waiting for Message...");
      }
    }*/
    
    // Read the INTF flags.
    byte intf = CCAN.GetInterrupt();
    delay(1);
    Serial.print("CANINTF:");
    Serial.println(intf);
    Frame readframe;
    readframe.data[0] = 3;
    readframe.data[1] = 6;
    if (intf & RX0IF) //if RX0 received
       readframe = CCAN.ReadBuffer(RXB0);
    if (intf & RX1IF) //if RX1 received
       readframe = CCAN.ReadBuffer(RXB1);
       
    // Print data back on Serial
    Serial.print("Messsage Received:");
    Serial.print(readframe.data[0]);
    Serial.println(readframe.data[1]);
    
    // Clear interrupts
    if (CCAN.GetInterrupt() == false)
    {
      Serial.println("Int Cleared");
    }
    delay(1000);
}
