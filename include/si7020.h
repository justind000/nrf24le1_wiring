#ifndef SI7020_H_
#define SI7020_H_
#include "wiring.h"

// Define the Sensor Address
const SI7020 = 0x40;
 
// Define register map
const SI7020_HUMIDITY = 0xE5;
const SI7020_TEMP = 0xE3;

uint16_t si7020data = 0;

uint16_t si7020Measurement(uint8_t command);

uint16_t si7020Humidity(){
	si7020data = si7020Measurement(SI7020_HUMIDITY);
	return (125.0*si7020data/65536)-6;;
}

uint16_t si7020Temperature(){
	si7020data = si7020Measurement(SI7020_TEMP);
	return ((175.25*si7020data/65536)-46.85) * 1.8 + 32;
}

uint16_t si7020Measurement(uint8_t command){
	uint8_t msb = 0;
	uint8_t lsb = 0;
	msb = wireRead8(SI7020, command);
	lsb = wireRead8(SI7020, command);
	lsb &= 0xFC;
	si7020data = msb << 8 | lsb;
	return si7020data;
}
#endif
