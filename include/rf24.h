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

// Original (C) 2011 J. Coliz <maniacbug@ymail.com> released under GNU 
// General Public License 2

#ifndef RF24_H_
#define RF24_H_

#include <string.h>
#include "reg24le1.h"
#include <rf.h>
#include "adc.h"
#include "wiring.h"

#define _BV(bit) (1 << (bit))
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_EN_CRC  3
#define RF_CRCO    2
#define PWR_UP	1
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2
#define RF_ERX_P0   0
#define ARD         4
#define ARC         0
#define DYNPD	    0x1C
#define FEATURE	    0x1D
#define STATUS      0x07
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define PRIM_RX     0
#define EN_ACK_PAY  1
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define CD          0x09

typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

uint8_t pipe0_reading_address[5];
uint8_t addr_width = 5;

void setDataRate(unsigned char speed);
void openWritingPipe(byte address1, byte address2, byte address3, byte address4, byte address5);
void setChannel(unsigned char channel);
void setAutoAck(unsigned char enable);
void setCRCLength(unsigned char length);
void setPALevel(unsigned char level);
void setRetries(uint8_t delay, uint8_t count);
void setAddressWidth(uint8_t a_width);
unsigned char available(uint8_t* pipe_num);
void openReadingPipe(uint8_t child, byte address1, byte address2, byte address3, byte address4, byte address5);
void startListening();
void closeReadingPipe( uint8_t pipe );
void stopListening();
void whatHappened(bool *tx_ok, bool *tx_fail, bool *rx_ready);
uint8_t getDynamicPayloadSize();
void enableDynamicPayloads();
bool testRPD();
bool testCarrier();
rf24_pa_dbm_e getPALevel();
rf24_datarate_e getDataRate();
rf24_crclength_e getCRCLength();
#define powerUp() rf_power_up(true);
#define powerDown() rf_power_down();



#define write(p1, p2, p3) rf_write_tx_payload_noack(p1, p2, true); while(!(rf_irq_pin_active() && rf_irq_tx_ds_active()));
#define read(p1, p2) rf_read_rx_payload(p1, p2);

void setRetries(uint8_t delay, uint8_t count)
{
	uint8_t reg = (delay&0xf)<<ARD | (count&0xf)<<ARC;
	rf_write_register(RF_SETUP_RETR, &reg, 1);
}

void setChannel(unsigned char channel){
	rf_write_register(RF_RF_CH, &channel, 1);
}

void setDataRate(unsigned char speed){

	uint8_t setup;
	rf_read_register(RF_RF_SETUP, &setup, 1);
	setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
	if( speed == RF24_250KBPS ){
		setup |= _BV( RF_DR_LOW ) ;
	}else{
		if ( speed == RF24_2MBPS ){
		  setup |= _BV(RF_DR_HIGH);
		}
	}
	rf_write_register(RF_RF_SETUP, &setup, 1 ) ;
}

void setAutoAck(unsigned char enable)
{
	unsigned char buffer; 
	if (enable==1) buffer=63;
	else buffer=0;
	rf_write_register(RF_EN_AA, &buffer,1);
}

void setCRCLength(unsigned char length)
{
	unsigned char buffer;
	rf_read_register(RF_CONFIG,&buffer,1);
	buffer = buffer & ~( _BV(RF_CRCO) | _BV(RF_EN_CRC)) ;
	if (length==1) buffer |= _BV(RF_EN_CRC);
	else if (length==2) buffer |= _BV(RF_EN_CRC) | _BV( RF_CRCO );
	rf_write_register(RF_CONFIG,&buffer,1 );

}

void setPALevel(unsigned char level)
{
	uint8_t setup;
	rf_read_register(RF_RF_SETUP, &setup, 1);
	setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
	setup|= ((level&3)<<RF_PWR_LOW);
	rf_write_register(RF_RF_SETUP,&setup,1);

}

void setAddressWidth(uint8_t a_width){
	uint8_t aw;

	if(a_width -= 2)
	{
		aw = a_width%4;
		rf_write_register(RF_SETUP_AW,& aw, 1);
		addr_width = (a_width%4) + 2;
	}

}
void openReadingPipe(uint8_t child, byte address1, byte address2, byte address3, byte address4, byte address5)
{
	uint8_t setup;
	unsigned char orpdata[5];
	orpdata[0]=address5;
	orpdata[1]=address4;
	orpdata[2]=address3;
	orpdata[3]=address2;
	orpdata[4]=address1;

	if (child == 0)
	{
		memcpy((const uint8_t*)pipe0_reading_address, &orpdata, addr_width);
	}

    // For pipes 2-5, only write the LSB
    if ( child < 2 ) { 
		rf_write_register(RF_RX_ADDR_P0 + child, (const uint8_t*)(&orpdata), addr_width);
    } else
		rf_write_register(RF_RX_ADDR_P0 + child, (const uint8_t*)(&orpdata[0]), 1);
	// rf_read_register(RF_RX_ADDR_P0+ child,(uint8_t*)(&data),5);printf_tiny("%x ", data[0]);
	// rf_read_register(RF_RX_ADDR_P0+ child,(uint8_t*)(&data),5);printf_tiny("%x ", data[1]);
	// rf_read_register(RF_RX_ADDR_P0+ child,(uint8_t*)(&data),5);printf_tiny("%x ", data[2]);
	// rf_read_register(RF_RX_ADDR_P0+ child,(uint8_t*)(&data),5);printf_tiny("%x ", data[3]);
	// rf_read_register(RF_RX_ADDR_P0+ child,(uint8_t*)(&data),5);printf_tiny("%x ", data[4]);
	setup=32;
	rf_write_register(RF_RX_PW_P0+child,&setup,1);
	rf_read_register(RF_EN_RXADDR,&setup,1);
	setup |= _BV(RF_ERX_P0+child);
	rf_write_register(RF_EN_RXADDR,&setup ,1);
}

void openWritingPipe(byte address1, byte address2, byte address3, byte address4, byte address5)
{
	uint8_t setup;
	unsigned char owpdata[5];
	owpdata[0]=address5;
	owpdata[1]=address4;
	owpdata[2]=address3;
	owpdata[3]=address2;
	owpdata[4]=address1;
 
	rf_write_register(RF_RX_ADDR_P0, (uint8_t*)(&owpdata), addr_width);
	rf_write_register(RF_TX_ADDR, (uint8_t*)(&owpdata), addr_width);
  
	setup=32;
	rf_write_register(RF_RX_PW_P0,&setup,1);
}


unsigned char available(uint8_t* pipe_num)
{
	unsigned char buffer;
	uint8_t status = rf_get_status();

	uint8_t result = ( status & _BV(RX_DR) );

	if (result)
	{
	// If the caller wants the pipe number, include that
	//  if ( pipe_num )
		*pipe_num = ( status >> RX_P_NO ) & 7;
		buffer=64;// _BV(RX_DR);
		rf_write_register(STATUS,&buffer,1 ); 
	}

  return result;
}

void startListening(void){
	// Restore the pipe0 adddress, if exists
	if (pipe0_reading_address[0] > 0)
	{
    	rf_write_register(RF_RX_ADDR_P0, (uint8_t*)pipe0_reading_address, addr_width);	
  	}
  	else
  	{
		closeReadingPipe(0);
	}

	rf_irq_clear_all(); //clear interrupts again
	rf_set_as_rx(true); //resume normal operation as an RX
}

void closeReadingPipe( uint8_t pipe ){
	uint8_t config;
	
	rf_read_register(RF_EN_RXADDR, &config, 1);
	config | RF_EN_RXADDR  & ~_BV(RF_RX_ADDR_P0+pipe);
	rf_write_register(RF_CONFIG,&config ,1);
}

void stopListening(void){
	rf_irq_clear_all(); //clear all interrupts in the 24L01
	rf_set_as_tx(); //change the device to a TX
}

void whatHappened(bool *tx_ok,bool *tx_fail,bool *rx_ready){

  // Read the status & reset the status in one easy call
  uint8_t whstatus;
  whstatus = rf_write_register(STATUS, (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT)), 1 );

  // Report to the user what happened
  *tx_ok = whstatus & _BV(TX_DS);
  *tx_fail = whstatus & _BV(MAX_RT);
  *rx_ready = whstatus & _BV(RX_DR);

}

uint8_t getDynamicPayloadSize(){

	uint8_t dps;
	rf_spi_execute_command(RF_R_RX_PL_WID, (uint8_t*)&dps, 1, true);
	return dps;
}

void enableDynamicPayloads(){

	rf_write_register(FEATURE,rf_read_register_1_byte(FEATURE) | _BV(EN_DPL), 1);
	rf_write_register(DYNPD,rf_read_register_1_byte(DYNPD) | _BV(DPL_P5) | _BV(DPL_P4) | _BV(DPL_P3) | _BV(DPL_P2) | _BV(DPL_P1) | _BV(DPL_P0), 1);
}

bool testRPD(){

	return ( rf_read_register_1_byte(RF_RPD) & 1 ) ;
}

bool testCarrier(){
	return ( rf_read_register_1_byte(CD) & 1 );
}

rf24_pa_dbm_e getPALevel(){
	uint8_t power;
	rf24_pa_dbm_e result = RF24_PA_ERROR ;

	rf_read_register(RF_RF_SETUP, &power, 1) ;
	power = power & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
	if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
	{
		result = RF24_PA_MAX ;
	}
	else if ( power == _BV(RF_PWR_HIGH) )
	{
		result = RF24_PA_HIGH ;
	}
	else if ( power == _BV(RF_PWR_LOW) )
	{
		result = RF24_PA_LOW ;
	}
	else
	{
		result = RF24_PA_MIN ;
	}

  return result ;
}

rf24_datarate_e getDataRate(void)
{
	uint8_t dr;
	rf24_datarate_e result ;
	rf_read_register(RF_RF_SETUP, &dr, 1) ;
	dr = dr & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  return result ;
}

rf24_crclength_e getCRCLength(void)
{
	uint8_t config;
	rf24_crclength_e result = RF24_CRC_DISABLED;
	rf_read_register(RF_CONFIG, &config, 1) ;
	config = config & (_BV(RF_CRCO) | _BV(RF_EN_CRC));

  if ( config & _BV(RF_EN_CRC ) )
  {
    if ( config & _BV(RF_CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}

#endif