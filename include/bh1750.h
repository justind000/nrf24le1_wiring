#ifndef BH1750_h
#define BH1750_h
#include "wiring.h"

#define BH1750_ADDR 0x23

// No active state
#define BH1750_POWER_DOWN 0x00

// Wating for measurment command
#define BH1750_POWER_ON 0x01

// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET 0x07

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE  0x10

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
#define BH1750_CONTINUOUS_HIGH_RES_MODE_2  0x11

// Start measurement at 4lx resolution. Measurement time is approx 16ms.
#define BH1750_CONTINUOUS_LOW_RES_MODE  0x13

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE  0x20

// Start measurement at 0.5lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_HIGH_RES_MODE_2  0x21

// Start measurement at 1lx resolution. Measurement time is approx 120ms.
// Device is automatically set to Power Down after measurement.
#define BH1750_ONE_TIME_LOW_RES_MODE  0x23

uint16_t readLuxLevel(){
	uint16_t lux;
	uint8_t msb = 0;
	uint8_t lsb = 0;

	//power on and set measurement mode
	wireWrite8(BH1750_ADDR, BH1750_POWER_ON);
	wireWrite8(BH1750_ADDR, BH1750_ONE_TIME_HIGH_RES_MODE);
	//delay while measurement is taken
	delay(120);
	//read 2 bytes
	msb = wireRead8(BH1750_ADDR, BH1750_ADDR);
	lsb = wireRead8(BH1750_ADDR, BH1750_ADDR);

	// | and divide by 1.2 per datasheet
	lux = (msb << 8 | lsb) / 1.2;
	return lux;
}

#endif
