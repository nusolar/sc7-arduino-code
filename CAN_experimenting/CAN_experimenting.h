#include "sc7-can-libinclude.h"

#define pedalPin A10

struct
{
	uint16_t raw_pedal;
	float current;
} Status;
