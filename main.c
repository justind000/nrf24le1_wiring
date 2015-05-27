#include "wiring.h"

#define DEBUG 1

void setup(){
	serialBegin();

	debugPrint("i/o node starting - sender", 0);
	watchdogRun(1000);
	
	pinMode(P0_0, INPUT | PULLUP);
	if(!digitalRead(P0_0)) 
	{
		//second lowest power mode
		sleep(PWR_CLK_MGMT_PWRDWN_MODE_MEMORY_RET_TMR_OFF);
	}
}

void loop(){

}
