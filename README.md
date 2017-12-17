# nrf24le1 Wiring Library

A Wiring-like wrapper to use the nrf24le1 SoC

Go to https://hackaday.io/project/5794-nrf24le1-wiring-library for more details

## Usage

#### pinMode, digitalRead, digitalWrite, analogRead, analogWrite

`pinMode(P0_0, INPUT | PULLUP);`

`digitalRead(P0_0);`

`digitalWrite(P0_0, HIGH);`

pinMode is slightly different in that you `|` together the options which are:
* PULLUP
* PULLDOWN
* INPUT
* OUTPUT
* HIGH_POWER
* NORMAL_POWER

`digitalRead`, `digitalWrite`, `analogWrite` work as usual.

`noPullup()` and `pullup()` are absent and rolled into `pinMode()`

##### Analog and PWM Functions

ADC is setup in 12 bit resolution, PWM pre-scaler to 10 and width to 8 bits. 

#### Interrupts

`interrupts()`, `noInterrupts()`, `detachInterrupt()` work as expected.

`attachInterrupt(GP_INT, condition)` is slightly different in that there is no function argument, conditions are `LOW` and `FALLING`. You need to write an ISR for `INTERRUPT_VECTOR_IFP`. Depending on your chip, the GPINT options are
* INTERRUPT_IFP_INPUT_GPINT0 //P0.0 on 24-pin, P0.5 on 32-pin, P1.2 on 48-pin
* INTERRUPT_IFP_INPUT_GPINT1 //P0.2 on 24-pin, P0.6 on 32-pin, P1.3 on 48-pin
* INTERRUPT_IFP_INPUT_GPINT2 //Not present on 24-pin or 32-pin, P1.4 on 48-pin

You can define an ISR like

```
ISR(INTERRUPT_VECTOR_IFP){
  //do something
}
```

The list of vectors:
* INTERRUPT_VECTOR_IFP
* INTERRUPT_VECTOR_T0
* INTERRUPT_VECTOR_PWR_FAIL
* INTERRUPT_VECTOR_T1
* INTERRUPT_VECTOR_UART
* INTERRUPT_VECTOR_T2
* INTERRUPT_VECTOR_RFRDY
* INTERRUPT_VECTOR_RFIRQ
* INTERRUPT_VECTOR_SPI_2WIRE
* INTERRUPT_VECTOR_WU_ON_PIN
* INTERRUPT_VECTOR_XOSC_ADC_RNG
* INTERRUPT_VECTOR_RTC2

#### Timing

`millis()` is implemented using timer0. Due to it's use of system resources, it is not started automatically. `millisBegin()` will start it. Thanks to maksms' GitHub repo for this routine. Access it through `millis()` which is a uint32_t. 

`delay()` and `delayMilliseconds()` work as expected

#### Serial

`serialBegin()` sets up up at 38400 8n1 which is the maximum speed. 

`serialPrint` and `serialPrintLn` are implemented as wrappers to `printf_small(format, args)`

`serialPrint("wiring wrapper version %u", 1);`

`debugPrint(format, args)` and `debugPrintLn(format, args)` are macros that wrap `printf_small()` and is enabled if DEBUG is defined:

`debugPrint("nrf24le1 with no args", 0);`
`debugPrint("nrf24le1 version %u", 1);`

| format | output type | argument-type   |
| ------ | ----------- | --------------- |
| %d     | decimal     | int             |
| %ld    | decimal     | long            |
| %hd    | decimal     | char            |
| %x     | hexadecimal | int             |
| %lx    | hexadecimal | long            |
| %hx    | hexadecimal | char            |
| %o     | octal       | int             |
| %lo    | octal       | long            |
| %ho    | octal       | char            |
| %c     | character   | char            |
| %s     | character   | generic pointer |
| %f     | float       | float*          |

<sup>SDCC is not compiled by default with float support</sup>
#### I2C

Automatically set up in master mode at 400kHz. I would like this to be made more Wiring-like. 

`wireWrite8(slave address, data)` returns W2_NACK_VAL or W2_ACK_VAL

`wireRead8(slave address, address)` returns W2_NACK_VAL or W2_ACK_VAL

`wireRead16(slave address, address)` returns W2_NACK_VAL or W2_ACK_VAL

#### Watchdog

`watchdogRun(millis)` will start the watchdog and cause a reset if not called before millis runs out. Does not work in deep sleep mode. Minimum is 7.8125 ms, max is 512 seconds. 

#### Sleep Modes

`sleep(mode)` will enter the passed mode. Modes are:
* ACTIVE
* STANDBY
* REGISTER_RET
* MEMORY_TIMER_ON
* MEMORY_TIMER_OFF
* DEEP_SLEEP

####  Random Number Generator

The nrf24le1 has an onboad RNG. The cstd `random()` has been replaced by the board RNG and returns a random byte. 

#### Pins

Are defined as `P0_0` `P1_1`etc. 

gpio.h has defines for all three variants and also the special names for the pins which can be used as well. 

#### AES Accelerator

The chip includes a hardware galois multiplication function.

`uint8_t aesGaloisMultiply(uint8_t value1, uint8_t value2)`
