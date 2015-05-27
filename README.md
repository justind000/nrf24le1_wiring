# nrf24le1 Wiring
A Wiring-like wrapper to use the nrf24le1 SoC

Go to https://hackaday.io/project/5794-nrf24le1-wiring-library for more details

#Basics
####pinMode, digitalRead, digitalWrite, analogRead, analogWrite
`pinMode(PX_D0, INPUT | PULLUP);`
`digitalRead(PX_D0);`
`digitalWrite(PX_D0, HIGH);`

pinMode is slightly different in that you `|` together the options which are:
* PULLUP
* PULLDOWN
* INPUT
* OUTPUT
* HIGH_POWER
* NORMAL_POWER

`digitalRead`, `digitalWrite`, `analogWrite` work as usual.

`noPullup()` and `pullup()` are absent and rolled into `pinMode()`

#####Analog and PWM Functions
ADC is setup for 12 bit resolution, PWM pre-scaler to 10 and width to 8 bits. 

####Interupts
Not implemented yet but `'interupts()` and `noInterrupts()` work as expected.

####Timing
`millis()` not implemented yet
`delay()` and `delayMilliseconds()` work as expected

####Serial
`serialBegin()` setups up at 38400 8n1
print and println are not implemented

`debugPrint(format, args)` is a macro that wraps `printf_tiny()` and is enabled if DEBUG is defined:

`debugPrint("nrf24le1 with no args", 0);`
`debugPrint("nrf24le1 version %u", 1);`

####I2C
`wireBegin()` is called in `main()` and sets up I2C in master mode at 400kHz. 

`wireWrite8(slave address, data)` returns W2_NACK_VAL or W2_ACK_VAL

`wireRead8(slave address, address)` returns W2_NACK_VAL or W2_ACK_VAL

`wireRead16(slave address, address)` returns W2_NACK_VAL or W2_ACK_VAL
