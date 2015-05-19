#include "wiring.h"
#include "nrf.h"

#define DEBUG 1

void setup(){
	serialBegin();

	debugPrint("i/o node starting - sender", 0);
	watchdogRun(1000);
	
	pinMode(GPIO_PIN_ID_FUNC_AIN0, INPUT | GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR);
	if(!digitalRead(GPIO_PIN_ID_FUNC_AIN0)) 
	{
		//second lowest power mode
		sleep(PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_OFF);
	}
}

void loop(){

}
