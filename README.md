# nrf24le1_wiring
A Wiring-like wrapper to use the nrf24le1 and u1 SoCs

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
Inside `main()` a function called `gpioSetup()` runs which setups up the ADC for 12 bit resolution, PWM pre-scaler to 10 and width to 8 bits. 

