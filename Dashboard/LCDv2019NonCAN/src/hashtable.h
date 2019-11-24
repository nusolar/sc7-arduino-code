#include <avr/pgmspace.h>

struct subframe
{
    uint8_t start;
    uint8_t length;
};

struct frame
{
    uint32_t id;
    subframe[] subframes;
};

// TRI88 DEFINITIONS
#define TRI88_MC_BASE_ADDRESS 0
#define TRI88_DC_BASE_ADDRESS 0
#define TRI88_DRIVE_ID TRI88_DC_BASE_ADDRESS + 0x01
#define TRI88_POWER_ID TRI88_DC_BASE_ADDRESS + 0x02
#define TRI88_RESET_ID TRI88_DC_BASE_ADDRESS + 0x03
#define TRI88_STATUS_ID TRI88_MC_BASE_ADDRESS + 0x01
#define TRI88_BUS_MEASURE_ID TRI88_MC_BASE_ADDRESS + 0x02
#define TRI88_VELOCITY_MEASURE_ID TRI88_MC_BASE_ADDRESS + 0x03
#define TRI88_TEMP_MEASURE_ID TRI88_MC_BASE_ADDRESS + 0x0B

// TRI88 Drive
#define TRI88_Drive_Vel 0
const subframe sf_TRI88_Drive_Vel = {.start = 0, .length = 32};
#define TRI88_Drive_I 1
const subframe sf_TRI88_Drive_I = {.start = 32, .length = 32};
#define TRI88_Drive 0
const frame f_TRI88_Drive PROGMEM = {TRI88_DRIVE_ID, [sf_TRI88_Drive_Vel, sf_TRI88_Drive_I] };


const int *const layouts[] PROGMEM = {f_TRI88_Drive};