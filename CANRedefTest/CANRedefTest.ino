#include <avr/pgmspace.h>

struct subframe
{
    uint8_t start;
    uint8_t len;
};

struct frame
{
    uint32_t id;
    uint8_t num;
    struct subframe *subframes;
};

// motor DEFINITIONS
#define motor_MC_BASE_ADDRESS 0
#define motor_DC_BASE_ADDRESS 0
#define motor_DRIVE_ID motor_DC_BASE_ADDRESS + 0x01
#define motor_POWER_ID motor_DC_BASE_ADDRESS + 0x02
#define motor_RESET_ID motor_DC_BASE_ADDRESS + 0x03
#define motor_STATUS_ID motor_MC_BASE_ADDRESS + 0x01
#define motor_BUS_MEASURE_ID motor_MC_BASE_ADDRESS + 0x02
#define motor_VELOCITY_MEASURE_ID motor_MC_BASE_ADDRESS + 0x03
#define motor_TEMP_MEASURE_ID motor_MC_BASE_ADDRESS + 0x0B

// motor Drive
#define motor_drive_vel 0
const subframe sf_motor_drive_vel PROGMEM = {.start = 0, .len = 32};
#define motor_drive_I 1
const subframe sf_motor_drive_I PROGMEM = {.start = 32, .len = 32};

#define motor_drive 0
const subframe sf_motor_drive[] PROGMEM = {sf_motor_drive_vel, sf_motor_drive_I};
const frame f_motor_drive PROGMEM = {.id = motor_DRIVE_ID, .num = 2, .subframes=sf_motor_drive};

// motor Bus Measurement
#define motor_power_V 0
const subframe sf_motor_power_V PROGMEM = {.start = 32, .len = 32};
#define motor_power_I 1
const subframe sf_motor_power_I PROGMEM = {.start = 0, .len = 32};

#define motor_power 1
const subframe sf_motor_power[] PROGMEM = {sf_motor_power_V, sf_motor_power_I};
const frame f_motor_power PROGMEM = {.id = motor_POWER_ID, .num = 2, .subframes=sf_motor_power};

// BMS DEFINITIONS
#define BMS_IVSOC_ID 0x6B0

// BMS IVSOC
#define bms_ivsoc_I 0
const subframe sf_bms_ivsoc_I PROGMEM = {.start = 0, .len = 16};
#define bms_ivsoc_V 1
const subframe sf_bms_ivsoc_V PROGMEM = {.start = 16, .len = 16};
#define bms_ivsoc_soc 2
const subframe sf_bms_ivsoc_soc PROGMEM = {.start = 32, .len = 8};

#define bms_ivsoc 2
const subframe sf_bms_ivsoc[] PROGMEM = {sf_bms_ivsoc_I, sf_bms_ivsoc_V, sf_bms_ivsoc_soc};
const frame f_bms_ivsoc PROGMEM = {.id = BMS_IVSOC_ID, .num = 3, .subframes=sf_bms_ivsoc};

// Array with all frame defintions
const frame layouts[] PROGMEM = {f_motor_drive, f_motor_power, f_bms_ivsoc};

void setup() {
  Serial.begin(9600);
  // Number of subframes
  Serial.println(layouts[motor_drive].num);
  Serial.println(layouts[motor_power].num);
  Serial.println(layouts[bms_ivsoc].num);
  Serial.println(layouts[bms_ivsoc].subframes[bms_ivsoc_I].start);
  Serial.println(layouts[bms_ivsoc].subframes[bms_ivsoc_V].start);
  Serial.println(layouts[bms_ivsoc].subframes[bms_ivsoc_soc].start);
  parse(0, layouts[bms_ivsoc]);

}

// Parse packet
void parse(uint64_t packet, struct frame f) {
   for (int ii = 0; ii < f.num; ii++) {
    int value = (packet >> f.subframes[ii].start) && (2^f.subframes[ii].len -1);
    Serial.println(value);
   }
}

void loop() {
  // put your main code here, to run repeatedly:

}
