#include <CAN_IO.h>
#include <Metro.h>
#include <SPI.h>

#define caninterruptp 14 // CAN interrupt
#define canchipp 52      // CAN chip select

//CAN parameters --> check these, may be diff
const byte CAN_CS = 52;
//const byte CAN_CS = (byte) '\003';
const byte CAN_INT = 14; // Interrupt #1
//const byte CAN_INT = (byte) '\a';
const uint16_t CAN_BAUD_RATE = 250;
const byte CAN_FREQ = 16;

CAN_IO CanControl(CAN_CS, CAN_INT, CAN_BAUD_RATE, CAN_FREQ); //Try initializing without interrupts for now

void setup()
{
  //pinMode(caninterruptp, INPUT_PULLUP); // CAN interrupt
  //pinMode(canchipp, INPUT_PULLUP);      // CAN chip select

  Serial.begin(115200);
  delay(3000);
  Serial.println(true << 10, BIN);

  //initializePins(); // Part of SWControls
  const uint16_t RXM0 = MASK_Sxxx;
  const uint16_t RXF0 = 0;              // Match any steering_wheel packet (because mask is Sx00)
  const uint16_t RXF1 = BMS19_VCSOC_ID; // Can't put 0 here, otherwise it will match all packets that start with 0.

  const uint16_t RXM1 = MASK_Sxxx;
  const uint16_t RXF2 = SW_DATA_ID;
  const uint16_t RXF3 = 0;                                      // No longer relevant, but keeping here to have a value
  const uint16_t RXF4 = (MTBA_FRAME0_REAR_LEFT_ID & MASK_Sxxx); // Not sure if necessary, but MTBA IDs are 29 bits
  const uint16_t RXF5 = (MTBA_FRAME0_REAR_RIGHT_ID & MASK_Sxxx);

  CanControl.filters.setRB0(MASK_Sxxx, RXF0, RXF1);
  CanControl.filters.setRB1(MASK_Sxxx, RXF2, RXF3, RXF4, RXF5); //**MC_VELOCITY_ID, **MC_PHASE_ID
  CanControl.Setup(RX0IE | RX1IE | TX1IE | TX2IE | TX0IE);

// Insert DEBUG and LOOPBACK steps
  if (CanControl.errors != 0)
  {
    Serial.print("Init CAN error: ");
    Serial.println(CanControl.errors, HEX);

    byte Txstatus[3] = {0, 0, 0};
    Txstatus[0] = CanControl.controller.Read(TXB0CTRL);
    Txstatus[1] = CanControl.controller.Read(TXB1CTRL);
    Txstatus[2] = CanControl.controller.Read(TXB2CTRL);
    byte canintf = 0;
    canintf = CanControl.last_interrupt;
    byte canctrl = 0;
    canctrl = CanControl.controller.Read(CANCTRL);

    Serial.println("TXnCTRL: ");
    Serial.println(Txstatus[0], BIN);
    Serial.println(Txstatus[1], BIN);
    Serial.println(Txstatus[2], BIN);
    Serial.print("Last Interrupt: ");
    Serial.println(canintf, BIN);
    Serial.print("CANCTRL: ");
    Serial.println(canctrl, BIN);
    Serial.print("CANSTAT: ");
    Serial.println(CanControl.canstat_register, BIN);
    Serial.println("");
  }
}

void loop()
{
  // Fetch any potential messages from the MCP2515
  CanControl.Fetch();

  if (CanControl.Available())
  {

    // Use available CAN packets to assign values to appropriate members of the data structures
    Frame &f = CanControl.Read();
    Serial.print("Received: ");
    Serial.print(f.id, HEX);
    Serial.println((int)f.value, BIN);

    if (f.id == BMS19_OVERHEAT_PRECHARGE_ID)
    {
      BMS19_Overheat_Precharge packet(f);
      Serial.print("Overheat packet: ");
      Serial.println(packet.overTempLimit);
    }
  }
}
