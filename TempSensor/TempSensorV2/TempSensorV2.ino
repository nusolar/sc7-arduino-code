/*
 * TempSensor.io
 * Code for interfacing with chain of tmp107-q1 temperature sensors.
 * See documentation:
 * http://www.ti.com/lit/ds/symlink/tmp107-q1.pdf
 * http://www.ti.com/lit/an/sboa138a/sboa138a.pdf
 */

const int RX_PIN = 19;
const int TX_PIN = 18;
const long BAUD_RATE = 57600;
const int NUM_DEVICES = 1;
const int TEMP_RESOLUTION = 0.015625; //degrees celsius per LSB

/*
 * calibrate
 * 
 * All commands must be preceded by a call to calibrate
 * to set the baud rate for the devices to expect
 */
void calibrate() {
  Serial1.write(0x55);
}

/*
 * initilizeAddresses
 * 
 * Each time the devices are turned on, they need to be assigned 
 * addresses to uniquely identify each one
 */
byte initializeAddresses() {
  byte num_devices = (byte) NUM_DEVICES;
  byte starting_address;
  if (NUM_DEVICES == 32) {
    starting_address = 0;
  }
  else {
    starting_address = 1;
  }
  byte max_address = starting_address + num_devices - 1;

  // write calibration byte, address initialize command byte, then starting address byte
  calibrate();
  delay(100);
  Serial1.write(0x95);
  delay(100);
  Serial1.write(0x05 + (max_address << 3));

  // if everything is working, each device sends back it's address one at a time separated by 7ms
  byte response_byte;
  byte response_address;
  byte correct_address = starting_address;
  for (int i = 0; i < NUM_DEVICES; i++) {
    //delay(7);
    
    while (Serial1.available() <= 0) {
      //Serial.println("serial not available");
      // do nothing until serial is available
    }
    
    response_byte = Serial1.read();
    Serial.println(response_byte, BIN);
    response_address = response_byte >> 3;
    Serial.println(response_address);
  }

  return max_address;
}


/**
 * readAllTemps
 * 
 * Reads temperature data from each sensor in the chain
 */
void readAllTemps(byte max_address){
  // write calibration, command, and register bytes
  byte command_byte = 0x3 + (max_address << 3);
  byte temp_register_byte = 0xa0;
  calibrate();
  Serial1.write(command_byte);
  Serial1.write(temp_register_byte);

  // read temperature data- 14 bits, least significant byte first
  // from each device
  for (int i = 0; i < NUM_DEVICES; i++) {
    while (Serial1.available() <= 0) {
      // do nothing until serial is available
    }
    byte low_byte = Serial1.read();
    byte high_byte = Serial1.read();

    if (high_byte & B00100000) {
      Serial.print("negative numbers? uh oh...");
      return;
    }
    // truncate to 14 bits, convert to celsius
    high_byte = high_byte & B00111111;
    int temp_data = high_byte * 2^8 + low_byte;
    float temp = temp_data * TEMP_RESOLUTION;

    Serial.print("Sensor: ");
    Serial.print(NUM_DEVICES - i);
    Serial.print(", Temperatre: ");
    Serial.println(temp);
  }
  
  
}

byte max_address;
void setup() {
  // Serial1 will be used to communicate with sensors
  Serial1.begin(BAUD_RATE);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  // Serial will be used to communicate with computer for debugging
  Serial.begin(9600);
  Serial.print("hey it's will");
  max_address = initializeAddresses();
}

void loop() {
  // put your main code here, to run repeatedly:
  // 11 ms to send global read to 32 devices at 57.6 kBd
  //readAllTemps(max_address);
  
  // read data lsb first. two bytes least sig byte first
}
