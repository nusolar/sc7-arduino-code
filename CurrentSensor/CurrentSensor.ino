/*
 * CurrentSensor.ino
 * Contains code to run current sensor board.
 */

#include <SPI.h>

// pins
const int CHIP_SELECT_PIN = 4;

const byte CONTROL_BYTE = 0xE0;
 
void setup() {
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  Serial.begin(9600);
  Serial.println("Connected");
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

  // write control byte to ADC
  digitalWrite(CHIP_SELECT_PIN, LOW); 
  SPI.transfer(CONTROL_BYTE);
  digitalWrite(CHIP_SELECT_PIN, HIGH); 
  delayMicroseconds(300);

  // read two bytes from ADC
  digitalWrite(CHIP_SELECT_PIN, LOW);
  int high = SPI.transfer(0x00);
  int low = SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  
  SPI.endTransaction();
  
  Serial.println(high);
  Serial.println(low);
  delay(100);
}
