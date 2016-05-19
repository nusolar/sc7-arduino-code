/*
 * CurrentSensor.ino
 * Contains code to run current sensor board.
 */

#include <SPI.h>

// pins
const int CHIP_SELECT_PIN = 4;

const byte CONTROL_BYTE = 0xE0;

byte high;
byte low;

int bit_current;
float voltage;
void setup() {
  pinMode(CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("Connected");
  SPI.begin();
}

void loop() {
  bit_current = 0;
  
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  
  // write control byte to ADC
  digitalWrite(CHIP_SELECT_PIN, LOW); 
  SPI.transfer(CONTROL_BYTE);
  digitalWrite(CHIP_SELECT_PIN, HIGH); 
  delayMicroseconds(1000);

  // read two bytes from ADC
  digitalWrite(CHIP_SELECT_PIN, LOW);
  high = SPI.transfer(0x00);
  low = SPI.transfer(0x00);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  
  SPI.endTransaction();
  bit_current = (low >> 2) | (high << 6);
  Serial.print("High part: ");
  Serial.println(high, BIN);
  Serial.print("Low part: ");
  Serial.println(low, BIN);
  Serial.print("Combined: ");
  Serial.println(bit_current, BIN);

  voltage = 4.096 / pow(2,14) * bit_current;
  Serial.print("Actual current: ");
  Serial.println(voltage);
  
  delay(1000);
}
