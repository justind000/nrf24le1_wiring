#include "wiring.h"
#include "pwr_clk_mgmt.h"

#define DEBUG 1

void main()
{
	serialBegin();
	wireBegin();
	analogSetup();

	debugPrint("i/o node starting - sender", 0);
	watchdogRun(1000)

	pinMode(GPIO_PIN_ID_FUNC_AIN0, INPUT | GPIO_PIN_CONFIG_OPTION_PIN_MODE_INPUT_BUFFER_ON_PULL_DOWN_RESISTOR);
	if(!digitalRead(GPIO_PIN_ID_FUNC_AIN0)) 
	{
		//second lowest power mode
		pwr_clk_mgmt_enter_pwr_mode_memory_ret_tmr_off();
	}
	else
	{
		//spin wheels while waiting to wd reset
		while(1){}
	}
}
