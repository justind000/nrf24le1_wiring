// Copyright (C) 2015 Justin Decker

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version 
// 3 of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


#ifndef Wiring_h
#define Wiring_h
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>
#include "gpio.h"
#include "uart.h"
#include "rng.h"
#include "w2.h"
#include "pwm.h"
#include "adc.h"
#include "watchdog.h"
#include "pwr_clk_mgmt.h"
#include "timer0.h"
#include "interrupt.h"
#include "delay.h"

#define debugPrint(fmt, ...) do { if (DEBUG) printf_small(fmt, __VA_ARGS__); } while (0)
#define debugPrintLn(fmt, ...) do { if (DEBUG) printf_small(fmt, __VA_ARGS__); printf_tiny("\n\r"); } while (0)
#define ISR(p1) void isr##p1() __interrupt(p1)

//Data
// #define bit()
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
// #define bitWrite()
#define highByte(w) ((uint8_t) ((w) >> 8))
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define makeWord(msb, lsb) (msb << 8 | lsb)

//Character classification - done
#define isAlpha isalpha
#define isAlphaNumeric isalnum
#define isASCII isascii
#define isControl iscntrl
#define isDigit isdigit
#define isGraph isgraph 
#define isHexadecimalDigit isxdigit 
#define isLowerCase islower 
#define isPrintable isprint 
#define isPunct ispunct 
#define isSpace isspace
#define isUpperCase isupper 
#define isWhitespace isspace 

//Pin digital Input/Output - done
#define digitalRead gpio_pin_val_read
#define digitalWrite gpio_pin_val_write
#define pinMode gpio_pin_configure
#define HIGH_POWER GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_HIGH_DRIVE_STRENGTH
#define NORMAL_POWER GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH
#define PULLDOWN GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR
#define PULLUP GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_UP_RESISTOR

//Pin analog Input - done
#define INTERNAL1V2 ADC_CONFIG_OPTION_REF_SELECT_INT_1_2V
#define VDD ADC_CONFIG_OPTION_REF_SELECT_VDD
#define EXTERNAL3 ADC_CONFIG_OPTION_REF_SELECT_EXT_AIN3
#define EXTERNAL9 ADC_CONFIG_OPTION_REF_SELECT_EXT_AIN9
#define analogRead adc_start_single_conversion_get_value
#define analogReference(p1) adc_configure ((uint16_t)ADC_CONFIG_OPTION_RESOLUTION_12_BITS| p1 |ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT)

//Pin PWM (analog) Output - done
#define analogWrite(p1, p2) pwm_start(p1, p2)
#define noAnalogWrite(p1) pwm_start(p1, 0)

//Time - done
#define delay delay_ms
#define delayMilliseconds delay_us

// millis() implementation
#define TLSTART 256-16000000/1000/12/6
unsigned long ml=0;
uint8_t mcs=0;	

ISR(INTERRUPT_VECTOR_T0){
	TL0 = TLSTART;
  
	if (mcs>=6)
	{
		ml++;
		mcs=0;
		
	}
	mcs++;
}

void millisBegin()
{
	interrupt_control_global_enable();
	interrupt_control_t0_enable()	;
	timer0_configure(TIMER0_CONFIG_OPTION_MODE_3_TWO_8_BIT_CTRS_TMRS,TLSTART);
	timer0_run();

}
#define millis() ml
#define micros() ml*1000

//Interrupts - done
#define attachInterrupt(p1, p2) interrupt_configure_ifp(p1, p2 | INTERRUPT_IFP_CONFIG_OPTION_ENABLE)
#define detachInterrupt(p1) interrupt_configure_ifp(p1, INTERRUPT_IFP_CONFIG_OPTION_DISABLE)
#define interrupts() interrupt_control_global_enable()
#define noInterrupts() interrupt_control_global_disable()

//Serial
#define serialPrint(fmt, ...) do { printf_small(fmt, __VA_ARGS__);} while (0)
#define serialPrintLn(fmt, ...) do { printf_small(fmt, __VA_ARGS__); printf_tiny("\n\r"); } while (0)
void serialBegin();
//serialAvailable
//serialRead
//serialWrite
//serialPeek
//serialFlush
//serialEnd

//Advanced Power Management - done
#define ACTIVE PWR_CLK_MGMT_PWRDWN_MODE_ACTIVE
#define STANDBY PWR_CLK_MGMT_PWRDWN_MODE_STANDBY
#define REGISTER_RET PWR_CLK_MGMT_PWRDWN_MODE_REGISTER_RET
#define MEMORY_TIMER_ON PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_ON
#define MEMORY_TIMER_OFF PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_OFF
#define DEEP_SLEEP PWR_CLK_MGMT_PWRDWN_MODE_DEEP_SLEEP
#define sleep(mode) PWRDWN = ((PWRDWN & ~(PWRDWN_PWR_CNTL_MASK)) | mode) 
#define noSleep() PWRDWN = ((PWRDWN & ~(PWRDWN_PWR_CNTL_MASK)) | ACTIVE) 

//Calculation
#define abs(x) ((x)>0?(x):-(x))
#define ceil ceilf
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define exp expf
#define fabs fabsf 
#define floor floorf 
//#define fma 
//#define fmax
//#define fmin
//#define fmod
#define ldexp ldexpf 
#define log logf 
#define log10 log10f 
#define map(x, in_min, in_max, out_min, out_max) (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define pow powf 
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
//#define signbit
#define sq(x) ((x)*(x))
#define sqrt sqrtf 
//#define square
//#define trunc

//Trigonometry
#define acos acosf 
#define asin asinf 
#define atan atanf 
#define atan2 atan2f 
#define cos cosf 
// #define degrees
// #define hypot
// #define radians
#define sin sinf 
#define sinh sinhf 
#define tan tanf 
#define tanh tanhf 

//HW random - returns a byte - done
#define random rng_get_one_byte_and_turn_off

//Constants - done
#define FALLING INTERRUPT_IFP_CONFIG_OPTION_TYPE_FALLING_EDGE
#define HIGH 1
#define INPUT GPIO_PIN_CONFIG_OPTION_DIR_INPUT
#define LOW 0
#define LSBFIRST 0
#define MSBFIRST 1
#define OUTPUT GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT

//Pin definitions
#define P0_0 GPIO_PIN_ID_P0_0
#define P0_1 GPIO_PIN_ID_P0_1
#define P0_2 GPIO_PIN_ID_P0_2
#define P0_3 GPIO_PIN_ID_P0_3
#define P0_4 GPIO_PIN_ID_P0_4
#define P0_5 GPIO_PIN_ID_P0_5
#define P0_6 GPIO_PIN_ID_P0_6
#define P0_7 GPIO_PIN_ID_P0_7
#define P1_0 GPIO_PIN_ID_P1_0
#define P1_1 GPIO_PIN_ID_P1_1
#define P1_2 GPIO_PIN_ID_P1_2
#define P1_3 GPIO_PIN_ID_P1_3
#define P1_4 GPIO_PIN_ID_P1_4
#define P1_5 GPIO_PIN_ID_P1_5
#define P1_6 GPIO_PIN_ID_P1_6
#define P1_7 GPIO_PIN_ID_P1_7
#define P2_0 GPIO_PIN_ID_P2_0
#define P2_1 GPIO_PIN_ID_P2_1
#define P2_2 GPIO_PIN_ID_P2_2
#define P2_3 GPIO_PIN_ID_P2_3
#define P2_4 GPIO_PIN_ID_P2_4
#define P2_5 GPIO_PIN_ID_P2_5
#define P2_6 GPIO_PIN_ID_P2_6
#define P2_7 GPIO_PIN_ID_P2_7
#define P3_0 GPIO_PIN_ID_P3_0
#define P3_1 GPIO_PIN_ID_P3_1
#define P3_2 GPIO_PIN_ID_P3_2
#define P3_3 GPIO_PIN_ID_P3_3
#define P3_4 GPIO_PIN_ID_P3_4
#define P3_5 GPIO_PIN_ID_P3_5
#define P3_6 GPIO_PIN_ID_P3_6



typedef unsigned int word;
typedef uint8_t boolean;
typedef uint8_t byte;

void setup();
void loop();
void putchar(char c);
char getchar();

uint8_t eepromRead(uint16_t address);
void eepromWrite(uint16_t address, uint8_t value);
w2_ack_nack_val_t wireWrite8(uint8_t slave_address, uint8_t data);
uint8_t wireRead8(uint8_t slave_address, uint8_t address);
uint16_t wireRead16(uint8_t slave_address, uint8_t address);
#define watchdogRun(p1) watchdog_start_and_set_timeout_in_ms(p1); CLKLFCTRL=1;

unsigned int i = 0;
unsigned char control_byte, address_byte, data_byte;

void main(){
	//gpio setup
	adc_configure ((uint16_t)ADC_CONFIG_OPTION_RESOLUTION_12_BITS|ADC_CONFIG_OPTION_REF_SELECT_VDD |ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT);
	pwm_configure(PWM_CONFIG_OPTION_PRESCALER_VAL_10 || PWM_CONFIG_OPTION_WIDTH_8_BITS);

	//Set up I2C hardware
	gpio_pin_configure(GPIO_PIN_ID_FUNC_W2SCL,
					   GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
					   GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_UP_RESISTOR);

	gpio_pin_configure(GPIO_PIN_ID_FUNC_W2SDA,
					   GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
					   GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_UP_RESISTOR);

	w2_configure(W2_CONFIG_OPTION_ENABLE | W2_CONFIG_OPTION_MODE_MASTER | W2_CONFIG_OPTION_CLOCK_FREQ_400_KHZ | W2_CONFIG_OPTION_ALL_INTERRUPTS_ENABLE, 0);

	setup();
	while(1){loop();}
}

void serialBegin(){

	//Set up UART pin
	gpio_pin_configure(GPIO_PIN_ID_FUNC_TXD,
					   GPIO_PIN_CONFIG_OPTION_DIR_OUTPUT |
					   GPIO_PIN_CONFIG_OPTION_OUTPUT_VAL_SET |
					   GPIO_PIN_CONFIG_OPTION_PIN_MODE_OUTPUT_BUFFER_NORMAL_DRIVE_STRENGTH);

	gpio_pin_configure(GPIO_PIN_ID_FUNC_RXD,
					   GPIO_PIN_CONFIG_OPTION_DIR_INPUT |
					   GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_NO_RESISTORS);

	//Set up UART
	uart_configure_8_n_1_9600();
}



void putchar(char c)
{
	uart_send_wait_for_complete(c);
}

                                                                                                                                                                                                                                            
char getchar()
{
	unsigned char retchar;
	retchar = uart_wait_for_rx_and_get();
	return retchar;
}

w2_ack_nack_val_t wireWrite8(uint8_t slave_address, uint8_t ww8data){
	
	if(w2_master_write_to(slave_address, &ww8data, 1, 0, 0) ==  W2_NACK_VAL)
	{
		return W2_NACK_VAL;
	}else{
		return W2_ACK_VAL;
	}
}

uint8_t wireRead8(uint8_t slave_address, uint8_t address){
	uint8_t wr8data = 0;

	if(w2_master_random_address_read(slave_address, &address, 1, (uint8_t*)&wr8data, 1) ==  W2_NACK_VAL)
	{
		return W2_NACK_VAL;}
	else{
		return wr8data;
	}
}

uint16_t wireRead16(uint8_t slave_address, uint8_t address){
	uint16_t wr16data = 0;

	if(w2_master_random_address_read(slave_address, &address, 1, (uint8_t*)&wr16data, 2) ==  W2_NACK_VAL)
	{
		return W2_NACK_VAL;}
	else{
		return wr16data;
	}
}

uint8_t eepromRead(uint16_t address){
		control_byte = (unsigned char)(0x50 | (address >> 8));
		address_byte = (unsigned char)address;

		if(w2_master_random_address_read(control_byte, &address_byte, 1, &data_byte, 1) ==  W2_NACK_VAL)
		{
			//printf_tiny("NACK on address 0x%x", i);
			return -1;
		}
	return data_byte;	

}

void eepromWrite(uint16_t address, uint8_t value){
	control_byte = (unsigned char)(0x50 | (address >> 8));
	address_byte = (unsigned char)address;
	data_byte = (unsigned char)value;

	if(w2_master_write_to(control_byte, &address_byte, 1, &data_byte, 1) ==  W2_NACK_VAL)
	{
		//printf_tiny("NACK on address 0x%x", i);
		return;
	}
}

uint8_t aesGaloisMultiply(uint8_t value1, uint8_t value2)
{
	CCPDATIA = value1;
	CCPDATIB = value2;
	return CCPDATO;
}
#endif