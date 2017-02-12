#include "wiring.h"
#include "rf.h"
#include <stdio.h>
#include "rtc2.h"

#define printf	printf_fast_f

#define DEBUG 1
#define WATCHDOG_TIME		0xA00	// about 20 seconds. WDSV*256/32768 = wdt reset period
#define LED2	P0_1
#define LED3	P0_2
#define LED_ON	LOW
#define LED_OFF	HIGH
#define SW_K2	P1_6
#define SW_K3	P1_5

uint16_t counter = 0x0D;
//uint8_t txaddr[] = {'J','E','N','S', 0x01}; // backwards on rpi
uint8_t txaddr[] = {0x02, 'S', 'N', 'E', 'J'};

uint8_t regbuf[2];
uint16_t adcraw;
__idata volatile uint32_t seconds = 0;

enum msgtype {
  MSG_POWER_METER,
  MSG_COUNT_VOLTS
};

typedef struct {
  uint8_t msgtype;
  float voltage;
  float current;
  float true_power;
  float power_factor;
  uint32_t counter;
} msg_power_meter;

typedef struct {
	uint8_t msgtype;
	uint32_t counter;
	float voltage;
} msg_count_volts;

//msg_power_meter msg;
msg_count_volts msg;

interrupt_isr_rtc2()
{
	seconds++;
}

float raw2volts(uint16_t raw) {
	#define VREF1_2  1.175		// nominal 1.2V, chosen to get accurate voltage reading
	float voltage = raw * VREF1_2 * 3.0 / 1024.0  ;
	return voltage;
}

void uart_init(){

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

void _sleep(uint8_t s) 
{
	do {	
		// sleep, wake by rtc2 tick
		pwr_clk_mgmt_enter_pwr_mode_register_ret();
		pwr_clk_mgmt_enter_pwr_mode_standby();
	} while (--s);
}

void setup()
{
	msg.msgtype = MSG_COUNT_VOLTS;
	msg.voltage = 5.0001;

	uart_init();

	printf("nrf24le1 node starting..\n");
	//watchdogRun(1000);
	
	pinMode(P0_0, INPUT | PULLUP);
	pinMode(SW_K2, INPUT | PULLUP);
	pinMode(SW_K3, INPUT | PULLUP);
	
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	digitalWrite(LED2, LED_OFF);
	digitalWrite(LED3, LED_OFF);
	/*
	rf_configure_debug_lite(false, 32);	
	*/
	rf_configure( RF_CONFIG_EN_CRC | RF_CONFIG_CRCO | RF_CONFIG_PWR_UP,	// uint8_t config
				 false,							// bool opt_rx_active_mode
				 RF_EN_AA_ENAA_ALL,			// uint8_t en_aa
				 RF_EN_RXADDR_DEFAULT_VAL,		// uint8_t en_rxaddr
				 RF_SETUP_AW_5BYTES,		// uint8_t setup_aw
				 RF_SETUP_RETR_ARD_1000 | RF_SETUP_RETR_ARC_4,		// uint8_t setup_retr
				 1,							// uint8_t rf_ch
				 RF_RF_SETUP_RF_DR_250_KBPS | RF_RF_SETUP_RF_PWR_NEG_12_DBM,		// uint8_t rf_setup
				 txaddr,							// uint8_t * rx_addr_p0
				 NULL,							// uint8_t * rx_addr_p1
				 RF_RX_ADDR_P2_DEFAULT_VAL,		// uint8_t rx_addr_p2
				 RF_RX_ADDR_P3_DEFAULT_VAL,		// uint8_t rx_addr_p3
				 RF_RX_ADDR_P4_DEFAULT_VAL,		// uint8_t rx_addr_p4
				 RF_RX_ADDR_P5_DEFAULT_VAL,		// uint8_t rx_addr_p5
				 txaddr,							// uint8_t * tx_addr
				 RF_RX_PW_P0_DEFAULT_VAL,		// uint8_t rx_pw_p0
				 RF_RX_PW_P1_DEFAULT_VAL,		// uint8_t rx_pw_p1
				 RF_RX_PW_P2_DEFAULT_VAL,		// uint8_t rx_pw_p2
				 RF_RX_PW_P3_DEFAULT_VAL,		// uint8_t rx_pw_p3
				 RF_RX_PW_P4_DEFAULT_VAL,		// uint8_t rx_pw_p4
				 RF_RX_PW_P5_DEFAULT_VAL,		// uint8_t rx_pw_p5
				 RF_DYNPD_DPL_ALL,				// uint8_t dynpd
				 RF_FEATURE_EN_DPL);				// uint8_t feature

	rf_set_as_tx();

	// rtc stuff
	pwr_clk_mgmt_clklf_configure(PWR_CLK_MGMT_CLKLF_CONFIG_OPTION_CLK_SRC_RCOSC32K);
	rtc2_configure(RTC2_CONFIG_OPTION_ENABLE | 
			RTC2_CONFIG_OPTION_COMPARE_MODE_0_RESET_AT_IRQ |
			RTC2_CONFIG_OPTION_DO_NOT_CAPTURE_ON_RFIRQ,
			0x7FFF); //rtc tick = 1 sec
	//rtc2_run();
	interrupt_control_rtc2_enable();
	sti();
	// adc setup
}

void loop()
{
	watchdog_set_wdsv_count(WATCHDOG_TIME); // reset wdt
	digitalWrite(LED2, LED_ON);	
	delay(20);
	digitalWrite(LED2, LED_OFF);	

	adc_configure( ADC_CONFIG_OPTION_ACQ_TIME_3_US | 
		ADC_CONFIG_OPTION_REF_SELECT_INT_1_2V | 
		ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT |
		ADC_CONFIG_OPTION_RESOLUTION_10_BITS );
	adcraw = adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD);

	msg.counter++;
	msg.voltage = raw2volts(adcraw);

	// rf stuff
	rf_flush_tx();
	rf_irq_clear_all();

	rf_write_tx_payload((uint8_t *) msg, sizeof(msg), true);

	// wait for rf transactino to complete
	while (!(rf_irq_pin_active()));
	if (rf_irq_tx_ds_active()) {
		// tx success
	} else if (rf_irq_max_rt_active()) {
		// tx retries failed
	}

	//while(!(rf_irq_pin_active() && rf_irq_tx_ds_active()));
	//while (rf_irq_max_rt_active()); // trap retrans timeout to trigger wdt

	printf("t: %4ld, cnt: %4d, adcraw: %4d, voltage: %0.3f\r\n", seconds, counter, adcraw, msg.voltage);

	digitalWrite(LED3, LED_ON);
	delay(20);
	digitalWrite(LED3, LED_OFF);	
	counter++;
	//delay(21000);
	// power save
	_sleep(8);

}
