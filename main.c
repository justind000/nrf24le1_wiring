#include "wiring.h"
#include "rf.h"
#include <stdio.h>
#define printf	printf_fast_f

#define DEBUG 1

#define LED2	P0_1
#define LED3	P0_2
#define LED_ON	LOW
#define LED_OFF	HIGH
#define SW_K2	P1_6
#define SW_K3	P1_5

uint16_t counter = 1001;
uint8_t txaddr[] = {'J','E','N','S', 0x01}; // backwards on rpi
///uint8_t txaddr[] = {0x01, 'S', 'N', 'E', 'J'};

uint8_t regbuf[2];
uint16_t adcraw;

enum msgtype {
  MSG_POWER_METER
};

typedef struct {
  uint8_t msgtype;
  float voltage;
  float current;
  float true_power;
  float power_factor;
  uint32_t counter;
} msg_power_meter;

msg_power_meter msg;

float raw2volts(uint16_t raw) {
	#define VREF1_2  1.175
	float voltage = raw * VREF1_2 * 3.0 / 1024.0  ;
	return voltage;
}

void setup()
{
	msg.msgtype = MSG_POWER_METER;
	msg.voltage = 5.0001;
	msg.current = 1.0001;
	msg.true_power = 100.01;
	msg.power_factor = 0.99;

	serialBegin();

	printf("nrf24le1 node starting..\r\n");
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

	rf_configure( RF_CONFIG_DEFAULT_VAL | RF_CONFIG_PWR_UP,	// uint8_t config
				 false,							// bool opt_rx_active_mode
				 RF_EN_AA_ENAA_NONE,			// uint8_t en_aa
				 RF_EN_RXADDR_DEFAULT_VAL,		// uint8_t en_rxaddr
				 RF_SETUP_AW_DEFAULT_VAL,		// uint8_t setup_aw
				 RF_SETUP_RETR_DEFAULT_VAL,		// uint8_t setup_retr
				 0x01,							// uint8_t rf_ch
				 RF_RF_SETUP_DEFAULT_VAL,		// uint8_t rf_setup
				 NULL,							// uint8_t * rx_addr_p0
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
				 RF_DYNPD_DPL_NONE,				// uint8_t dynpd
				 RF_FEATURE_NONE);				// uint8_t feature
	*/
	//rf_configure_debug(false, 4, false);

	rf_configure( RF_CONFIG_EN_CRC | RF_CONFIG_CRCO | RF_CONFIG_PWR_UP,	// uint8_t config
				 false,							// bool opt_rx_active_mode
				 RF_EN_AA_ENAA_NONE,			// uint8_t en_aa
				 RF_EN_RXADDR_DEFAULT_VAL,		// uint8_t en_rxaddr
				 RF_SETUP_AW_5BYTES,		// uint8_t setup_aw
				 RF_SETUP_RETR_DEFAULT_VAL,		// uint8_t setup_retr
				 0,							// uint8_t rf_ch
				 RF_RF_SETUP_RF_DR_250_KBPS | RF_RF_SETUP_RF_PWR_NEG_12_DBM,		// uint8_t rf_setup
				 NULL,							// uint8_t * rx_addr_p0
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

	// adc setup

}

void loop()
{
	//debugPrint("%s\n", mystring);
	digitalWrite(LED2, LED_ON);	
	delay(100);
	digitalWrite(LED2, LED_OFF);	
	delay(500);
	digitalWrite(LED3, LED_ON);
	delay(200);
	digitalWrite(LED3, LED_OFF);	
	delay(500);
	
	regbuf[0] = counter & 0xFF;
	regbuf[1] = (counter >> 8) & 0xFF;

	adc_configure( ADC_CONFIG_OPTION_ACQ_TIME_3_US | 
		ADC_CONFIG_OPTION_REF_SELECT_INT_1_2V | 
		ADC_CONFIG_OPTION_RESULT_JUSTIFICATION_RIGHT |
		ADC_CONFIG_OPTION_RESOLUTION_10_BITS );
	adcraw = adc_start_single_conversion_get_value(ADC_CHANNEL_1_THIRD_VDD);

	msg.counter++;
	msg.voltage = raw2volts(adcraw);

	rf_write_tx_payload_noack((uint8_t *) msg, sizeof(msg), true);
	//rf_transmit();

	while(!(rf_irq_pin_active() && rf_irq_tx_ds_active()));
	//rf_irq_clear_all();

	printf("sent %d. adcraw: %d, voltage: %0.3f\r\n", counter, adcraw, msg.voltage);
	delay(200);
	counter++;

}
