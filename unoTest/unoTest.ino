#include <CAN_IO.h>
#include <SPI.h>


//CAN parameters
const byte     CAN_CS      = 10; // An Arbitrary Pin (DUE pin 52)
const byte     CAN_INT     = 2; // Arduino UNO interrupts at Digital 2 and 3 (DUE pin 14- Tx3)
const uint16_t CAN_BAUD_RATE = 125;  // MUST match the baud rate of the CAN bus. Setting this to 0 will enable Auto-BAUD (untested)
const byte     CAN_FREQ      = 16;    // MUST BE the frequency of the oscillator you use

unsigned long previous_send_time = 0;

// Set up the can controller object.
CAN_IO CanControl(CAN_CS,CAN_INT,CAN_BAUD_RATE,CAN_FREQ);

void setup() {
  Serial.begin(9600);
  
  // setup CAN
  // Extended IDs
  const uint32_t RXM0 = MASK_EID;
  const uint32_t RXF0 = MTBA_FRAME0_REAR_LEFT_ID;
  const uint32_t RXF1 = MTBA_FRAME0_REAR_RIGHT_ID;

  // Standard IDs
  const uint32_t RXM1 = MASK_NONE; // Last three bits free for MPPT offset
  const uint32_t RXF2 = BMS19_VCSOC_ID;
  const uint32_t RXF3 = MPPT_ANS_BASEADDRESS;
  const uint32_t RXF4 = DC_TEMP_0_ID;
  const uint32_t RXF5 = 0; // Place Holder
  
  CanControl.filters.setRB0(RXM0, RXF0, RXF1, true);
  CanControl.filters.setRB1(RXM1, RXF2, RXF3, RXF4, RXF5, false);
  CanControl.Setup(RX0IE | RX1IE | TX0IE | TX1IE | TX2IE);

  if (CanControl.errors != 0)
  {
    Serial.print(F("Init CAN error: "));
    Serial.println(CanControl.errors, HEX);
    byte Txstatus[3] = {0, 0, 0};
    Txstatus[0] = CanControl.controller.Read(TXB0CTRL);
    Txstatus[1] = CanControl.controller.Read(TXB1CTRL);
    Txstatus[2] = CanControl.controller.Read(TXB2CTRL);
    byte canintf = 0;
    canintf = CanControl.last_interrupt;
    byte canctrl = 0;
    canctrl = CanControl.controller.Read(CANCTRL);

    Serial.println(F("TXnCTRL: "));
    Serial.println(Txstatus[0], BIN);
    Serial.println(Txstatus[1], BIN);
    Serial.println(Txstatus[2], BIN);
    Serial.print(F("Last Interrupt: "));
    Serial.println(canintf, BIN);
    Serial.print(F("CANCTRL: "));
    Serial.println(canctrl, BIN);
    Serial.print(F("CANSTAT: "));
    Serial.println(CanControl.canstat_register, BIN);
    Serial.println(F(""));
  }

  Serial.println("\n\nTRYING TO SEND...");
}

void loop() {
  CanControl.Fetch(); //If there are any new messages, they will be received.

  // Sending CAN
  if (millis() - previous_send_time > 500) // Check and see whether the timer has expired
  {
    // This command sends data over any available TX port.
    bool trysend = CanControl.Send(BMS19_Overheat_Precharge(false, false), TXBANY);
    Serial.println(trysend);

    // You can print out the error counters. You can also read registers on the board by using the controller.Read() command.
    CanControl.FetchErrors(); //Call this first to get the error data from the MCP2515
    Serial.println("TEC/REC ");
    Serial.print(CanControl.tec); Serial.print(", ");
    Serial.println(CanControl.rec);

    previous_send_time = millis();
  }

  if (CanControl.Available()) // Check if there are messages that have been received
  {
    // Get the frame of of the buffer
    Frame& f = CanControl.Read();

    // Print the frame
    Serial.print(f.toString());
  }
  delay(100);
}
