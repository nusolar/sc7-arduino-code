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
const int NUM_DEVICES = 32;

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
void initializeAddresses(int int_num_devices) {
  byte num_devices = (byte) int_num_devices;
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
  Serial1.write(0x95);
  Serial1.write(0x05 + (max_address << 3));

  // if everything is working, each device sends back it's address one at a time separated by 7ms
  byte response_byte;
  byte response_address;
  byte correct_address = starting_address;
  for (int i = 0; i < NUM_DEVICES; i++) {
    delay(7);
    if (Serial1.available() > 0) {
      response_byte = Serial1.read();
      response_address = response_byte >> 3;
      Serial.println(response_address);
    }
    else {
      Serial.println("You need to fix the timing of initialize addresses");
    }
  }
}

void setup() {
  // Serial1 will be used to communicate with sensors
  Serial1.begin(BAUD_RATE);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  // Serial will be used to communicate with computer for debugging
  Serial.begin(9600);

  initializeAddresses(NUM_DEVICES);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 11 ms to send global read to 32 devices at 57.6 kBd

  // read data lsb first. two bytes least sig byte first
}
