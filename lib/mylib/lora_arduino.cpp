/*! \file SX1278.cpp
 *  \brief Library for managing Semtech modules
 *
 *  Copyright (C) 2015 Wireless Open Source
 *  http://wirelessopensource.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.

 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:		1.0
 *  Design:			David Gascón
 *  Implementation:	Covadonga Albiñana, Yuri Carmona
 *  SX1278 adapting:		Pedro del Molino
 */

#include "lora_arduino.hpp"
#include "timer.hpp"
#include "system_functions.hpp"
//#include <cmath>

SX1278::SX1278()
{
	// Initialize class variables
	_bandwidth = BW_125;
	_codingRate = CR_5;
	_spreadingFactor = SF_7;
	_channel = CH_1;
	_header = HEADER_ON;
	_CRC = CRC_ON;
	_modem = LORA;
	_power = 15;
	_packetNumber = 0;
	_reception = CORRECT_PACKET;
	_retries = 0;
	_maxRetries = 3;
	packet_sent.retry = _retries;
};

// PRIVATE FUNCTION //
void *memset(void *s, int c, int n)
{
    unsigned char* p = (unsigned char*)s;
    while(n--)
        *p++ = (unsigned char)c;
    return s;
}
uint16_t strlen(char* s){
	uint16_t res = 0;
	while(*s++ != '\0') res++;
	return res;
}
float max(float f1, float f2){
	return (f1 > f2) ? f1 : f2;
}


/*
 Function: Sets the module ON.
 Returns: Nothing
*/
uint8_t SX1278::ON()
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'ON'");
	#endif

	// Powering the module
  unselect_chip();

	// Set Maximum Over Current Protection
	state = setMaxCurrent(0x1B);

	if( state == 0 )
	{
		#if (SX1278_debug_mode > 1)
			Serial.println("## Setting ON with maximum current supply ##");
			Serial.println();
		#endif
	}
	else
	{
		return 1;
	}

	// set LoRa mode
	state = setLORA();

	return state;
}

/*
 Function: Sets the module OFF.
 Returns: Nothing
*/
void SX1278::OFF()
{
	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'OFF'");
	#endif

  // Powering the module
  select_chip();

	#if (SX1278_debug_mode > 1)
		Serial.println("## Setting OFF ##");
		Serial.println();
	#endif
}

/*
 Function: Reads the indicated register.
 Returns: The content of the register
 Parameters:
   address: address register to read from
*/
uint8_t SX1278::readRegister(uint8_t address)
{
  uint8_t value = spi_read8(address);

  #if (SX1278_debug_mode > 2)
    Serial.print("## Reading:  ##\t");
		Serial.print("Register ");
		Serial.print(address, HEX);
		Serial.print(":  ");
		Serial.print(value, HEX);
		Serial.println();
	#endif

  return value;
}

/*
 Function: Writes on the indicated register.
 Returns: Nothing
 Parameters:
   address: address register to write in
   data : value to write in the register
*/
void SX1278::writeRegister(uint8_t address, uint8_t data)
{
	spi_write8(address, data);

  #if (SX1278_debug_mode > 2)
    Serial.print("## Writing:  ##\t");
		Serial.print("Register ");
		bitClear(address, 7);
		Serial.print(address, HEX);
		Serial.print(":  ");
		Serial.print(data, HEX);
		Serial.println();
	#endif

}

/*
 * Function: Clears the interruption flags
 *
 * LoRa Configuration registers are accessed through the SPI interface.
 * Registers are readable in all device mode including Sleep. However, they
 * should be written only in Sleep and Stand-by modes.
 *
 * Returns: Nothing
*/
void SX1278::clearFlags()
{
    uint8_t st0;

	// Save the previous status
	st0 = readRegister(REG_OP_MODE);

	if( _modem == LORA )
	{
		/// LoRa mode
		// Stdby mode to write in registers
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
		// LoRa mode flags register
		writeRegister(REG_IRQ_FLAGS, 0xFF);
		// Getting back to previous status
		writeRegister(REG_OP_MODE, st0);

		#if (SX1278_debug_mode > 1)
			Serial.println("## LoRa flags cleared ##");
		#endif
	}
	else
	{
		/// FSK mode
		// Stdby mode to write in registers
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
		// FSK mode flags1 register
		writeRegister(REG_IRQ_FLAGS1, 0xFF);
		// FSK mode flags2 register
		writeRegister(REG_IRQ_FLAGS2, 0xFF);
		// Getting back to previous status
		writeRegister(REG_OP_MODE, st0);

		#if (SX1278_debug_mode > 1)
			Serial.println("## FSK flags cleared ##");
		#endif
	}
}

/*
 Function: Sets the module in LoRa mode.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setLORA()
{
    uint8_t state = 2;
    uint8_t st0;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setLORA'");
	#endif

	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);    // Sleep mode (mandatory to set LoRa mode)
	writeRegister(REG_OP_MODE, LORA_SLEEP_MODE);    // LoRa sleep mode
	writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode

	writeRegister(REG_MAX_PAYLOAD_LENGTH,MAX_LENGTH);

	// Set RegModemConfig1 to Default values
	writeRegister(REG_MODEM_CONFIG1, 0x72);
	// Set RegModemConfig2 to Default values
	writeRegister(REG_MODEM_CONFIG2, 0x70);
	// Set RegModemConfig2 to Default values
	writeRegister(REG_MODEM_CONFIG3, 0x00);

	//wait_with_timer2(100);

	st0 = readRegister(REG_OP_MODE);	// Reading config mode
	if( st0 == LORA_STANDBY_MODE )
	{ // LoRa mode
		_modem = LORA;
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.println("## LoRa set with success ##");
			Serial.println();
		#endif
	}
	else
	{ // FSK mode
		_modem = FSK;
		state = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** There has been an error while setting LoRa **");
			Serial.println();
		#endif
	}
	return state;
}

/*
 Function: Sets the module in FSK mode.
 Returns:   Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setFSK()
{
	uint8_t state = 2;
    uint8_t st0;
    uint8_t config1;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setFSK'");
	#endif

	writeRegister(REG_OP_MODE, FSK_SLEEP_MODE);	// Sleep mode (mandatory to change mode)
	writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// FSK standby mode
	config1 = readRegister(REG_PACKET_CONFIG1);
	config1 = config1 & 0b01111101;		// clears bits 8 and 1 from REG_PACKET_CONFIG1
	config1 = config1 | 0b00000100;		// sets bit 2 from REG_PACKET_CONFIG1
	writeRegister(REG_PACKET_CONFIG1,config1);	// AddressFiltering = NodeAddress + BroadcastAddress
	writeRegister(REG_FIFO_THRESH, 0x80);	// condition to start packet tx
	config1 = readRegister(REG_SYNC_CONFIG);
	config1 = config1 & 0b00111111;
	writeRegister(REG_SYNC_CONFIG,config1);

	//wait_with_timer2(100);

	st0 = readRegister(REG_OP_MODE);	// Reading config mode
	if( st0 == FSK_STANDBY_MODE )
	{ // FSK mode
		_modem = FSK;
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.println("## FSK set with success ##");
			Serial.println();
		#endif
	}
	else
	{ // LoRa mode
		_modem = LORA;
		state = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** There has been an error while setting FSK **");
			Serial.println();
		#endif
	}
	return state;
}

/*
 Function: Gets the bandwidth, coding rate and spreading factor of the LoRa modulation.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getMode()
{
	uint8_t st0;
	int8_t state = 2;
	uint8_t value = 0x00;

	#if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getMode'");
	#endif

	// Save the previous status
	st0 = readRegister(REG_OP_MODE);
	// Setting LoRa mode
	if( _modem == FSK )
	{
		setLORA();
	}
	value = readRegister(REG_MODEM_CONFIG1);
	_bandwidth = (value >> 4);   				// Storing 4 MSB from REG_MODEM_CONFIG1 (=_bandwidth)
	_codingRate = (value >> 1) & 0x07;  		// Storing first, second and third bits from
	value = readRegister(REG_MODEM_CONFIG2);	// REG_MODEM_CONFIG1 (=_codingRate)
	_spreadingFactor = (value >> 4) & 0x0F; 	// Storing 4 MSB from REG_MODEM_CONFIG2 (=_spreadingFactor)
	state = 1;

	if( isBW(_bandwidth) )		// Checking available values for:
	{								//		_bandwidth
		if( isCR(_codingRate) )		//		_codingRate
		{							//		_spreadingFactor
			if( isSF(_spreadingFactor) )
			{
				state = 0;
			}
		}
	}

	#if (SX1278_debug_mode > 1)
	  Serial.println("## Parameters from configuration mode are:");
	  Serial.print("Bandwidth: ");
	  Serial.print(_bandwidth, HEX);
	  Serial.println();
	  Serial.print("\t Coding Rate: ");
	  Serial.print(_codingRate, HEX);
	  Serial.println();
	  Serial.print("\t Spreading Factor: ");
	  Serial.print(_spreadingFactor, HEX);
	  Serial.println(" ##");
	  Serial.println();
	#endif

	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}

/*
 Function: Indicates if module is configured in implicit or explicit header mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1278::getHeader()
{
	int8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getHeader'");
	#endif

	// take out bit 2 from REG_MODEM_CONFIG1 indicates ImplicitHeaderModeOn
	if( bitRead(REG_MODEM_CONFIG1, 0) == 0 )
	{ // explicit header mode (ON)
		_header = HEADER_ON;
		state = 1;
	}
	else
	{ // implicit header mode (OFF)
		_header = HEADER_OFF;
		state = 1;
	}

	state = 0;

	if( _modem == FSK )
	{ // header is not available in FSK mode
		#if (SX1278_debug_mode > 1)
			Serial.println("## Notice that FSK mode packets hasn't header ##");
			Serial.println();
		#endif
	}
	else
	{ // header in LoRa mode
		#if (SX1278_debug_mode > 1)
			Serial.print("## Header is ");
			if( _header == HEADER_ON )
			{
				Serial.println("in explicit header mode ##");
			}
			else
			{
				Serial.println("in implicit header mode ##");
			}
			Serial.println();
		#endif
	}
	return state;
}

/*
 Function: Sets the module in explicit header mode (header is sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1278::setHeaderON()
{
  int8_t state = 2;
  uint8_t config1;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setHeaderON'");
  #endif

  if( _modem == FSK )
  {
	  state = -1;		// header is not available in FSK mode
	  #if (SX1278_debug_mode > 1)
		  Serial.println("## FSK mode packets hasn't header ##");
		  Serial.println();
	  #endif
  }
  else
  {
	config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the header bit
	if( _spreadingFactor == 6 )
	{
		state = -1;		// Mandatory headerOFF with SF = 6
		#if (SX1278_debug_mode > 1)
			Serial.println("## Mandatory implicit header mode with spreading factor = 6 ##");
		#endif
	}
	else
	{
		config1 = config1 & 0b11111110;			// clears bit 2 from config1 = headerON
		writeRegister(REG_MODEM_CONFIG1,config1);	// Update config1
	}
	if( _spreadingFactor != 6 )
	{ // checking headerON taking out bit 2 from REG_MODEM_CONFIG1
		config1 = readRegister(REG_MODEM_CONFIG1);
		if( bitRead(config1, 0) == HEADER_ON )
		{
			state = 0;
			_header = HEADER_ON;
			#if (SX1278_debug_mode > 1)
				Serial.println("## Header has been activated ##");
				Serial.println();
			#endif
		}
		else
		{
			state = 1;
		}
	}
  }
  return state;
}

/*
 Function: Sets the module in implicit header mode (header is not sent).
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1278::setHeaderOFF()
{
	uint8_t state = 2;
	uint8_t config1;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setHeaderOFF'");
	#endif

	if( _modem == FSK )
	{
		// header is not available in FSK mode
		state = -1;
		#if (SX1278_debug_mode > 1)
			Serial.println("## Notice that FSK mode packets hasn't header ##");
			Serial.println();
		#endif
	}
	else
	{
		// Read config1 to modify only the header bit
		config1 = readRegister(REG_MODEM_CONFIG1);

		// sets bit 2 from REG_MODEM_CONFIG1 = headerOFF
		config1 = config1 | 0b00000001;
		// Update config1
		writeRegister(REG_MODEM_CONFIG1,config1);

		// check register
		config1 = readRegister(REG_MODEM_CONFIG1);
		if( bitRead(config1, 2) == HEADER_OFF )
		{
			// checking headerOFF taking out bit 2 from REG_MODEM_CONFIG1
			state = 0;
			_header = HEADER_OFF;

			#if (SX1278_debug_mode > 1)
			    Serial.println("## Header has been desactivated ##");
			    Serial.println();
			#endif
		}
		else
		{
			state = 1;
			#if (SX1278_debug_mode > 1)
				Serial.println("** Header hasn't been desactivated ##");
				Serial.println();
			#endif
		}
	}
	return state;
}

/*
 Function: Indicates if module is configured with or without checking CRC.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1278::getCRC()
{
	int8_t state = 2;
	uint8_t value;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getCRC'");
	#endif

	if( _modem == LORA )
	{ // LoRa mode

		// take out bit 2 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
		value = readRegister(REG_MODEM_CONFIG2);
		if( bitRead(value, 2) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			#if (SX1278_debug_mode > 1)
				Serial.println("## CRC is desactivated ##");
				Serial.println();
			#endif
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			#if (SX1278_debug_mode > 1)
				Serial.println("## CRC is activated ##");
				Serial.println();
			#endif
			state = 0;
		}
	}
	else
	{ // FSK mode

		// take out bit 2 from REG_PACKET_CONFIG1 indicates CrcOn
		value = readRegister(REG_PACKET_CONFIG1);
		if( bitRead(value, 4) == CRC_OFF )
		{ // CRCoff
			_CRC = CRC_OFF;
			#if (SX1278_debug_mode > 1)
				Serial.println("## CRC is desactivated ##");
				Serial.println();
			#endif
			state = 0;
		}
		else
		{ // CRCon
			_CRC = CRC_ON;
			#if (SX1278_debug_mode > 1)
				Serial.println("## CRC is activated ##");
				Serial.println();
			#endif
			state = 0;
		}
	}
	if( state != 0 )
	{
		state = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** There has been an error while getting configured CRC **");
			Serial.println();
		#endif
	}
	return state;
}

/*
 Function: Sets the module with CRC on.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1278::setCRC_ON()
{
  uint8_t state = 2;
  uint8_t config;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setCRC_ON'");
  #endif

  if( _modem == LORA )
  { // LORA mode
	config = readRegister(REG_MODEM_CONFIG2);	// Save config to modify only the CRC bit
	config = config | 0b00000100;				// sets bit 2 from REG_MODEM_CONFIG2 = CRC_ON
	writeRegister(REG_MODEM_CONFIG2,config);

	state = 1;

	config = readRegister(REG_MODEM_CONFIG2);
	if( bitRead(config, 2) == CRC_ON )
	{ // take out bit 1 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_ON;
		#if (SX1278_debug_mode > 1)
			Serial.println("## CRC has been activated ##");
			Serial.println();
		#endif
	}
  }
  else
  { // FSK mode
	config = readRegister(REG_PACKET_CONFIG1);	// Save config to modify only the CRC bit
	config = config | 0b00010000;				// set bit 4 from REG_PACKET_CONFIG1 = CRC_ON
	writeRegister(REG_PACKET_CONFIG1,config);

	state = 1;

	config = readRegister(REG_PACKET_CONFIG1);
	if( bitRead(config, 4) == CRC_ON )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates CrcOn
		state = 0;
		_CRC = CRC_ON;
		#if (SX1278_debug_mode > 1)
			Serial.println("## CRC has been activated ##");
			Serial.println();
		#endif
	}
  }
  if( state != 0 )
  {
	  state = 1;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** There has been an error while setting CRC ON **");
		  Serial.println();
	  #endif
  }
  return state;
}

/*
 Function: Sets the module with CRC off.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1278::setCRC_OFF()
{
  int8_t state = 2;
  uint8_t config;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setCRC_OFF'");
  #endif

  if( _modem == LORA )
  { // LORA mode
  	config = readRegister(REG_MODEM_CONFIG2);	// Save config1 to modify only the CRC bit
	config = config & 0b11111011;				// clears bit 1 from config1 = CRC_OFF
	writeRegister(REG_MODEM_CONFIG2,config);

	config = readRegister(REG_MODEM_CONFIG2);
	if( (bitRead(config, 2)) == CRC_OFF )
	{ // take out bit 1 from REG_MODEM_CONFIG2 indicates RxPayloadCrcOn
	  state = 0;
	  _CRC = CRC_OFF;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("## CRC has been desactivated ##");
		  Serial.println();
	  #endif
	}
  }
  else
  { // FSK mode
	config = readRegister(REG_PACKET_CONFIG1);	// Save config1 to modify only the CRC bit
	config = config & 0b11101111;				// clears bit 4 from config1 = CRC_OFF
	writeRegister(REG_PACKET_CONFIG1,config);

	config = readRegister(REG_PACKET_CONFIG1);
	if( bitRead(config, 4) == CRC_OFF )
	{ // take out bit 4 from REG_PACKET_CONFIG1 indicates RxPayloadCrcOn
		state = 0;
		_CRC = CRC_OFF;
		#if (SX1278_debug_mode > 1)
		    Serial.println("## CRC has been desactivated ##");
		    Serial.println();
	    #endif
	}
  }
  if( state != 0 )
  {
	  state = 1;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** There has been an error while setting CRC OFF **");
		  Serial.println();
	  #endif
  }
  return state;
}

/*
 Function: Checks if SF is a valid value.
 Returns: bool that's 'true' if the SF value exists and
		  it's 'false' if the SF value does not exist.
 Parameters:
   spr: spreading factor value to check.
*/
bool	SX1278::isSF(uint8_t spr)
{
  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'isSF'");
  #endif

  // Checking available values for _spreadingFactor
  switch(spr)
  {
	  case SF_6:
	  case SF_7:
	  case SF_8:
	  case SF_9:
	  case SF_10:
	  case SF_11:
	  case SF_12:	return true;
					break;

	  default:		return false;
  }
  #if (SX1278_debug_mode > 1)
	  Serial.println("## Finished 'isSF' ##");
	  Serial.println();
  #endif
}

/*
 Function: Gets the SF within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1278::getSF()
{
  int8_t state = 2;
  uint8_t config2;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getSF'");
  #endif

  if( _modem == FSK )
  {
	  state = -1;		// SF is not available in FSK mode
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** FSK mode hasn't spreading factor **");
		  Serial.println();
	  #endif
  }
  else
  {
	// take out bits 7-4 from REG_MODEM_CONFIG2 indicates _spreadingFactor
	config2 = (readRegister(REG_MODEM_CONFIG2)) >> 4;
	_spreadingFactor = config2;
	state = 1;

	if( (config2 == _spreadingFactor) && isSF(_spreadingFactor) )
	{
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.print("## Spreading factor is ");
			Serial.print(_spreadingFactor, HEX);
			Serial.println(" ##");
			Serial.println();
		#endif
	}
  }
  return state;
}

/*
 Function: Sets the indicated SF in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   spr: spreading factor value to set in LoRa modem configuration.
*/
uint8_t	SX1278::setSF(uint8_t spr)
{
	uint8_t st0;
	int8_t state = 2;
	uint8_t config2;
	uint8_t config3;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setSF'");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if( _modem == FSK )
	{
		/// FSK mode
		#if (SX1278_debug_mode > 1)
			Serial.print("## Notice that FSK hasn't Spreading Factor parameter, ");
			Serial.println("so you are configuring it in LoRa mode ##");
		#endif
		state = setLORA();				// Setting LoRa mode
	}
	else
	{
		/// LoRa mode
		// LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);

		// Read config2 to modify SF value (bits 7-4)
		config2 = (readRegister(REG_MODEM_CONFIG2));
		// Read config3 to modify only the LowDataRateOptimize
		config3 = (readRegister(REG_MODEM_CONFIG3));

		switch(spr)
		{
			case SF_6:
					config2 = config2 & 0b01101111;	// clears bits 7 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 0b01100000;	// sets bits 6 & 5 from REG_MODEM_CONFIG2
					break;
			case SF_7:
					config2 = config2 & 0b01111111;	// clears bits 7 from REG_MODEM_CONFIG2
					config2 = config2 | 0b01110000;	// sets bits 6, 5 & 4
					break;

			case SF_8:
					config2 = config2 & 0b10001111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 0b10000000;	// sets bit 7 from REG_MODEM_CONFIG2
					break;

			case SF_9:
					config2 = config2 & 0b10011111;	// clears bits 6, 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 0b10010000;	// sets bits 7 & 4 from REG_MODEM_CONFIG2
					break;

			case SF_10:	config2 = config2 & 0b10101111;	// clears bits 6 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 0b10100000;	// sets bits 7 & 5 from REG_MODEM_CONFIG2
					break;

			case SF_11:
					config2 = config2 & 0b10111111;	// clears bit 6 from REG_MODEM_CONFIG2
					config2 = config2 | 0b10110000;	// sets bits 7, 5 & 4 from REG_MODEM_CONFIG2
					getBW();
					if( _bandwidth <= BW_125 )
					{ // LowDataRateOptimize (Mandatory with SF_11 if BW_125)
						config3 = config3 | 0b00001000;
					}
					break;

			case SF_12:
					config2 = config2 & 0b11001111;	// clears bits 5 & 4 from REG_MODEM_CONFIG2
					config2 = config2 | 0b11000000;	// sets bits 7 & 6 from REG_MODEM_CONFIG2

					getBW();
					if( _bandwidth <= BW_125 )
					{ // LowDataRateOptimize (Mandatory with SF_12 if BW_125)
						config3 = config3 | 0b00001000;
					}
					break;
	}

	// Check if it is neccesary to set special settings for SF=6
	if( spr == SF_6 )
	{
		// Mandatory headerOFF with SF = 6 (Implicit mode)
		setHeaderOFF();

		// Set the bit field DetectionOptimize of
		// register RegLoRaDetectOptimize to value "0b101".
		writeRegister(REG_DETECT_OPTIMIZE, 0x05);

		// Write 0x0C in the register RegDetectionThreshold.
		writeRegister(REG_DETECTION_THRESHOLD, 0x0C);
	}
	else
	{
		// LoRa detection Optimize: 0x03 --> SF7 to SF12
		writeRegister(REG_DETECT_OPTIMIZE, 0x03);

		// LoRa detection threshold: 0x0A --> SF7 to SF12
		writeRegister(REG_DETECTION_THRESHOLD, 0x0A);
	}

	// sets bit 1-0 of REG_MODEM_CONFIG2 (SymbTimout) and bit 2 of REG_MODEM_CONFIG3 (AgcAutoOn) for any SF value
	config2 = config2 | 0b00000011;
	config3 = config3 | 0b00000100;

	// Update 'config2' and 'config3'
	writeRegister(REG_MODEM_CONFIG2, config2);
	writeRegister(REG_MODEM_CONFIG3, config3);

	// Read 'config2' and 'config3' to check update
	config2 = (readRegister(REG_MODEM_CONFIG2));
	config3 = (readRegister(REG_MODEM_CONFIG3));

	// (config2 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG2 (=_spreadingFactor)
	// bitRead(config3, 3) ---> take out bits 1 from config3 (=LowDataRateOptimize)
	switch(spr)
	{
		case SF_6:	if(		((config2 >> 4) == spr)
						&& 	(bitRead(config3, 2) == 1)
						&& 	(_header == HEADER_OFF))
					{
						state = 0;
					}
					break;
		case SF_7:	if(		((config2 >> 4) == 0x07)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_8:	if(		((config2 >> 4) == 0x08)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_9:	if(		((config2 >> 4) == 0x09)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_10:	if(		((config2 >> 4) == 0x0A)
						 && (bitRead(config3, 2) == 1))
					{
						state = 0;
					}
					break;
		case SF_11:	getBW();
				if(		((config2 >> 4) == 0x0B)
						 && (bitRead(config3, 2) == 1)
						 && (_bandwidth > BW_125))
					{
						state = 0;
					}
				else if		((_bandwidth <= BW_125)
						 && (bitRead(config3, 3) == 1))
					{
						state = 0;
					}
					break;
		case SF_12:	getBW();
				if(		((config2 >> 4) == 0x0C)
						 && (bitRead(config3, 2) == 1)
						 && (_bandwidth > BW_125))
					{
						state = 0;
					}
				else if		((_bandwidth <= BW_125)
						 && (bitRead(config3, 3) == 1))
					{
						state = 0;
					}
					break;
		default:	state = 1;
	}
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status

  if( isSF(spr) )
  { // Checking available value for _spreadingFactor
		state = 0;
		_spreadingFactor = spr;
		#if (SX1278_debug_mode > 1)
		    Serial.print("## Spreading factor ");
		    Serial.print(_spreadingFactor, DEC);
		    Serial.println(" has been successfully set ##");
				if((spr == SF_11 || spr == SF_12) && _bandwidth <= BW_125){
				    Serial.println("## Low Data Rate Optimization has been successfully set ##");
				}
		    Serial.println();
		#endif
  }
  else
  {
	  if( state != 0 )
	  {
		  #if (SX1278_debug_mode > 1)
		      Serial.print("** There has been an error while setting the spreading factor **");
		      Serial.println();
		  #endif
	  }
  }
  return state;
}

/*
 Function: Checks if BW is a valid value.
 Returns: bool that's 'true' if the BW value exists and
		  it's 'false' if the BW value does not exist.
 Parameters:
   band: bandwidth value to check.
*/
bool	SX1278::isBW(uint16_t band)
{
  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'isBW'");
  #endif

  // Checking available values for _bandwidth
  switch(band)
  {
	  case BW_7_8:
	  case BW_10_4:
	  case BW_15_6:
	  case BW_20_8:
	  case BW_31_2:
	  case BW_41_7:
	  case BW_62_5:
	  case BW_125:
	  case BW_250:
	  case BW_500:	return true;
					break;

	  default:		return false;
  }
  #if (SX1278_debug_mode > 1)
	  Serial.println("## Finished 'isBW' ##");
	  Serial.println();
  #endif
}

/*
 Function: Gets the BW within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1278::getBW()
{
  uint8_t state = 2;
  uint8_t config1;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getBW'");
  #endif

  if( _modem == FSK )
  {
	  state = -1;		// BW is not available in FSK mode
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** FSK mode hasn't bandwidth **");
		  Serial.println();
	  #endif
  }
  else
  {
	  // take out bits 7-4 from REG_MODEM_CONFIG1 indicates _bandwidth
	  config1 = (readRegister(REG_MODEM_CONFIG1)) >> 4;
	  _bandwidth = config1;

	  if( (config1 == _bandwidth) && isBW(_bandwidth) )
	  {
		  state = 0;
		  #if (SX1278_debug_mode > 1)
			  Serial.print("## Bandwidth is ");
			  Serial.print(_bandwidth, HEX);
			  Serial.println(" ##");
			  Serial.println();
		  #endif
	  }
	  else
	  {
		  state = 1;
		  #if (SX1278_debug_mode > 1)
			  Serial.print("** There has been an error while getting bandwidth **");
			  Serial.println();
		  #endif
	  }
  }
  return state;
}

/*
 Function: Sets the indicated BW in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   band: bandwith value to set in LoRa modem configuration.
*/
int8_t	SX1278::setBW(uint16_t band)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t config1;
  uint8_t config3;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setBW'");
  #endif

  st0 = readRegister(REG_OP_MODE);	// Save the previous status

  if( _modem == FSK )
  {
	  #if (SX1278_debug_mode > 1)
		  Serial.print("## Notice that FSK hasn't Bandwidth parameter, ");
		  Serial.println("so you are configuring it in LoRa mode ##");
	  #endif
	  state = setLORA();
  }
  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// LoRa standby mode
  config1 = (readRegister(REG_MODEM_CONFIG1));	// Save config1 to modify the BW
  config3 = (readRegister(REG_MODEM_CONFIG3));	// Save config3 to modify the Low Data Rate Optimization
  switch(band)
  {
	case BW_7_8:  	config1 = config1 & 0b00001111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b00000000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_10_4: config1 = config1 & 0b00011111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b00010000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_15_6: config1 = config1 & 0b00101111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b00100000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_20_8: config1 = config1 & 0b00111111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b00110000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_31_2: config1 = config1 & 0b01001111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b01000000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_41_7: config1 = config1 & 0b01011111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b01010000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_62_5: config1 = config1 & 0b01101111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b01100000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_125:  config1 = config1 & 0b01111111;	// clears bits 7-4 from REG_MODEM_CONFIG1
			config1 = config1 | 0b01110000;	//sets bit 7-4 from REG_MODEM_CONFIG1
			getSF();
			if( _spreadingFactor == 11 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_11)
				config3 = config3 | 0b00001000;
			}
			if( _spreadingFactor == 12 )
			{ // LowDataRateOptimize (Mandatory with BW_125 if SF_12)
				config3 = config3 | 0b00001000;
			}
			break;
	  case BW_250:  config1 = config1 & 0b10001111;	// clears bit 7 from REG_MODEM_CONFIG1
			config1 = config1 | 0b10000000;	// sets bit 6 from REG_MODEM_CONFIG1
			break;
	  case BW_500:  config1 = config1 & 0b10011111;	//clears bit 6 from REG_MODEM_CONFIG1
			config1 = config1 | 0b10010000;	//sets bit 7 from REG_MODEM_CONFIG1
			break;

  }
  writeRegister(REG_MODEM_CONFIG1,config1);		// Update config1
  writeRegister(REG_MODEM_CONFIG3,config3);		// Update config3

  config1 = (readRegister(REG_MODEM_CONFIG1));
  config3 = (readRegister(REG_MODEM_CONFIG3));
  // (config1 >> 4) ---> take out bits 7-4 from REG_MODEM_CONFIG1 (=_bandwidth)
  switch(band)
  {
	   case BW_7_8: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_10_4: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_15_6: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_20_8: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_31_2: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_41_7: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_62_5: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_125: if( (config1 >> 4) == BW_125 )
					{
						state = 0;
						getSF();
						if( _spreadingFactor == 11 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
						if( _spreadingFactor == 12 )
						{
							if( bitRead(config3, 3) == 1 )
							{ // LowDataRateOptimize
								state = 0;
							}
							else
							{
								state = 1;
							}
						}
					}
					break;
	   case BW_250: if( (config1 >> 4) == BW_250 )
					{
						state = 0;
					}
					break;
	   case BW_500: if( (config1 >> 4) == BW_500 )
					{
						state = 0;
					}
					break;
  }

  if( not isBW(band) )
  {
	  state = 1;
	  #if (SX1278_debug_mode > 1)
		  Serial.print("** Bandwidth ");
		  Serial.print(band, HEX);
		  Serial.println(" is not a correct value **");
		  Serial.println();
	  #endif
  }
  else
  {
	  _bandwidth = band;
	  #if (SX1278_debug_mode > 1)
		  Serial.print("## Bandwidth ");
		  Serial.print(band, HEX);
		  Serial.println(" has been successfully set ##");
		  Serial.println();
	  #endif
  }
  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}

/*
 Function: Checks if CR is a valid value.
 Returns: bool that's 'true' if the CR value exists and
		  it's 'false' if the CR value does not exist.
 Parameters:
   cod: coding rate value to check.
*/
bool	SX1278::isCR(uint8_t cod)
{
  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'isCR'");
  #endif

  // Checking available values for _codingRate
  switch(cod)
  {
	  case CR_5:
	  case CR_6:
	  case CR_7:
	  case CR_8:	return true;
					break;

	  default:		return false;
  }
  #if (SX1278_debug_mode > 1)
	  Serial.println("## Finished 'isCR' ##");
	  Serial.println();
  #endif
}

/*
 Function: Indicates the CR within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t	SX1278::getCR()
{
  int8_t state = 2;
  uint8_t config1;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getCR'");
  #endif

  if( _modem == FSK )
  {
	  state = -1;		// CR is not available in FSK mode
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** FSK mode hasn't coding rate **");
		  Serial.println();
	  #endif
  }
  else
  {
	// take out bits 7-3 from REG_MODEM_CONFIG1 indicates _bandwidth & _codingRate
	config1 = (readRegister(REG_MODEM_CONFIG1)) >> 1;
	config1 = config1 & 0b00000111;	// clears bits 7-4 ---> clears _bandwidth
	_codingRate = config1;
	state = 1;

	if( (config1 == _codingRate) && isCR(_codingRate) )
	{
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.print("## Coding rate is ");
			Serial.print(_codingRate, HEX);
			Serial.println(" ##");
			Serial.println();
		#endif
	}
  }
  return state;
}

/*
 Function: Sets the indicated CR in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   cod: coding rate value to set in LoRa modem configuration.
*/
int8_t	SX1278::setCR(uint8_t cod)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t config1;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setCR'");
  #endif

  st0 = readRegister(REG_OP_MODE);		// Save the previous status

  if( _modem == FSK )
  {
	  #if (SX1278_debug_mode > 1)
		  Serial.print("## Notice that FSK hasn't Coding Rate parameter, ");
		  Serial.println("so you are configuring it in LoRa mode ##");
	  #endif
	  state = setLORA();
  }
  else
  {
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);		// Set Standby mode to write in registers

	  config1 = readRegister(REG_MODEM_CONFIG1);	// Save config1 to modify only the CR
	  switch(cod)
	  {
		 case CR_5: config1 = config1 & 0b11110011;	// clears bits 3 & 2 from REG_MODEM_CONFIG1
			    config1 = config1 | 0b00000010;	// sets bit 1 from REG_MODEM_CONFIG1
			    break;
		 case CR_6: config1 = config1 & 0b11110101;	// clears bits 3 & 1 from REG_MODEM_CONFIG1
			    config1 = config1 | 0b00000100;	// sets bit 2 from REG_MODEM_CONFIG1
			    break;
		 case CR_7: config1 = config1 & 0b11110111;	// clears bit 3 from REG_MODEM_CONFIG1
			    config1 = config1 | 0b00000110;	// sets bits 2 & 1 from REG_MODEM_CONFIG1
			    break;
		 case CR_8: config1 = config1 & 0b11111001;	// clears bits 2 & 1 from REG_MODEM_CONFIG1
			    config1 = config1 | 0b00001000;	// sets bit 3 from REG_MODEM_CONFIG1
			    break;
	  }
	  writeRegister(REG_MODEM_CONFIG1, config1);		// Update config1

	  config1 = readRegister(REG_MODEM_CONFIG1);
	  // ((config1 >> 3) & 0b0000111) ---> take out bits 5-3 from REG_MODEM_CONFIG1 (=_codingRate)
	  switch(cod)
	  {
		 case CR_5: if( ((config1 >> 1) & 0b0000111) == 0x01 )
					{
						state = 0;
					}
					break;
		 case CR_6: if( ((config1 >> 1) & 0b0000111) == 0x02 )
					{
						state = 0;
					}
					break;
		 case CR_7: if( ((config1 >> 1) & 0b0000111) == 0x03 )
					{
						state = 0;
					}
					break;
		 case CR_8: if( ((config1 >> 1) & 0b0000111) == 0x04 )
					{
						state = 0;
					}
					break;
	  }
  }

  if( isCR(cod) )
  {
	  _codingRate = cod;
	  #if (SX1278_debug_mode > 1)
		  Serial.print("## Coding Rate ");
		  Serial.print(cod, HEX);
		  Serial.println(" has been successfully set ##");
		  Serial.println();
	  #endif
  }
  else
  {
	  state = 1;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("** There has been an error while configuring Coding Rate parameter **");
		  Serial.println();
	  #endif
  }
  writeRegister(REG_OP_MODE,st0);	// Getting back to previous status
  return state;
}

/*
 Function: Checks if channel is a valid value.
 Returns: bool that's 'true' if the CR value exists and
		  it's 'false' if the CR value does not exist.
 Parameters:
   ch: frequency channel value to check.
*/
bool	SX1278::isChannel(uint32_t ch)
{
  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'isChannel'");
  #endif

  // Checking available values for _channel
  switch(ch)
  {
	  case CH_1_BW_500:
	  case CH_2_BW_500: // same value that CH_7_BW_125
	  case CH_3_BW_500:

	  case CH_1_BW_250:
	  case CH_2_BW_250:
	  case CH_3_BW_250:
	  case CH_4_BW_250:
	  case CH_5_BW_250:
	  case CH_6_BW_250:

	  case CH_1_BW_125:
	  case CH_2_BW_125:
	  case CH_3_BW_125:
	  case CH_4_BW_125:
	  case CH_5_BW_125:
	  case CH_6_BW_125:
	  case CH_8_BW_125:
	  case CH_9_BW_125:
	  case CH_10_BW_125:
	  case CH_11_BW_125:
	  case CH_12_BW_125:
	  case CH_13_BW_125:

	  case CH_1:
	  case CH_2:
	  case CH_3:
	  case CH_4:
	  case CH_5:
	  case CH_6:
	  case CH_7:
	  case CH_8:
	  case CH_9:
	  case CH_10:
	  case CH_11:
	  case CH_12:
	  case CH_13:
	  case CH_14:
	  case CH_15:
	  case CH_16:
	  case CH_17:
	  case CH_18:
	  case CH_19:
	  case CH_20:
	  case CH_21:
	  case CH_22:
	  case CH_23:
	  case CH_24:		return true;
				break;

	  default:		return false;
  }
  #if (SX1278_debug_mode > 1)
	  Serial.println("## Finished 'isChannel' ##");
	  Serial.println();
  #endif
}

/*
 Function: Indicates the frequency channel within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getChannel()
{
  uint8_t state = 2;
  uint32_t ch;
  uint8_t freq3;
  uint8_t freq2;
  uint8_t freq1;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getChannel'");
  #endif

  freq3 = readRegister(REG_FRF_MSB);	// frequency channel MSB
  freq2 = readRegister(REG_FRF_MID);	// frequency channel MID
  freq1 = readRegister(REG_FRF_LSB);	// frequency channel LSB
  ch = ((uint32_t)freq3 << 16) + ((uint32_t)freq2 << 8) + (uint32_t)freq1;
  _channel = ch;						// frequency channel

  if( (_channel == ch) && isChannel(_channel) )
  {
	  state = 0;
	  #if (SX1278_debug_mode > 1)
		  Serial.print("## Frequency channel is ");
		  Serial.print(_channel, HEX);
		  Serial.println(" ##");
		  Serial.println();
	  #endif
  }
  else
  {
	  state = 1;
  }
  return state;
}

/*
 Function: Sets the indicated channel in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   ch: frequency channel value to set in configuration.
*/
int8_t SX1278::setChannel(uint32_t ch)
{
  uint8_t st0;
  int8_t state = 2;
  unsigned int freq3;
  unsigned int freq2;
  uint8_t freq1;
  uint32_t freq;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setChannel'");
  #endif

  st0 = readRegister(REG_OP_MODE);	// Save the previous status
  if( _modem == LORA )
  {
	  // LoRa Stdby mode in order to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  {
	  // FSK Stdby mode in order to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  freq3 = ((ch >> 16) & 0x0FF);		// frequency channel MSB
  freq2 = ((ch >> 8) & 0x0FF);		// frequency channel MIB
  freq1 = (ch & 0xFF);				// frequency channel LSB

  writeRegister(REG_FRF_MSB, freq3);
  writeRegister(REG_FRF_MID, freq2);
  writeRegister(REG_FRF_LSB, freq1);

  // storing MSB in freq channel value
  freq3 = (readRegister(REG_FRF_MSB));
  freq = (freq3 << 8) & 0xFFFFFF;

  // storing MID in freq channel value
  freq2 = (readRegister(REG_FRF_MID));
  freq = (freq << 8) + ((freq2 << 8) & 0xFFFFFF);

  // storing LSB in freq channel value
  freq = freq + ((readRegister(REG_FRF_LSB)) & 0xFFFFFF);

  if( freq == ch )
  {
    state = 0;
    _channel = ch;
    #if (SX1278_debug_mode > 1)
		Serial.print("## Frequency channel ");
		Serial.print(ch, HEX);
		Serial.println(" has been successfully set ##");
		Serial.println();
	#endif
  }
  else
  {
    state = 1;
  }

  if( not isChannel(ch) )
  {
	 state = -1;
	 #if (SX1278_debug_mode > 1)
		 Serial.print("** Frequency channel ");
		 Serial.print(ch, HEX);
		 Serial.println("is not a correct value **");
		 Serial.println();
	 #endif
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}

/*
 Function: Gets the signal power within the module is configured.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getPower()
{
  uint8_t state = 2;
  uint8_t value = 0x00;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getPower'");
  #endif

  value = readRegister(REG_PA_CONFIG);
  state = 1;

  value = value & 0b00001111;
  // Pout= 17-(15-OutputPower) = OutputPower+2
  value = value + 2;

  _power = value;
  if( (value >= 2) && (value <= 20) )
  {
	    state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.print("## Output power is ");
			Serial.print(_power, HEX);
			Serial.println(" ##");
			Serial.println();
		#endif
  }

  return state;
}

/*
 Function: Sets the signal power indicated in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   p: power option to set in configuration.
*/
int8_t SX1278::setPower(char p)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t value = 0x00;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setPower'");
  #endif

  st0 = readRegister(REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  writeRegister(REG_PA_DAC, 0x84);

  switch (p)
  {

    // M = max
    // H = high
    // I = intermediate
    // L = low

    case 'M':  _power = 0xFF; //20dbm
	       writeRegister(REG_PA_DAC, 0x87);
               break;

    case 'H':  _power = 0xFC; //14dbm
               break;

    case 'I':  _power = 0xF6; //8dbm
               break;

    case 'L':  _power = 0xF0; //2dbm
               break;
    default:   state = -1;
               break;
  }

  writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("## Output power has been successfully set ##");
		  Serial.println();
	  #endif
  }
  else
  {
	  state = 1;
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}

/*
 Function: Sets the signal power indicated as input to the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   pow: power option to set in configuration. The input value range is from
   0 to 14 dBm.
*/
int8_t SX1278::setPowerNum(uint8_t pow)
{
  uint8_t st0;
  int8_t state = 2;
  uint8_t value = 0x00;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'setPower'");
  #endif

  st0 = readRegister(REG_OP_MODE);	  // Save the previous status
  if( _modem == LORA )
  { // LoRa Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
  }
  else
  { // FSK Stdby mode to write in registers
	  writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
  }

  if ( (pow >= 2) && (pow <= 20) )
  { // Pout= 17-(15-OutputPower) = OutputPower+2
	  if ( pow <= 17 ) {
		writeRegister(REG_PA_DAC, 0x84);
	  	pow = pow - 2;
	  } else { // Power > 17dbm -> Power = 20dbm
		writeRegister(REG_PA_DAC, 0x87);
		pow = 15;
	  }
	  _power = pow;
  }
  else
  {
	  state = -1;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("## Power value is not valid ##");
		  Serial.println();
	  #endif
  }

  writeRegister(REG_PA_CONFIG, _power);	// Setting output power value
  value = readRegister(REG_PA_CONFIG);

  if( value == _power )
  {
	  state = 0;
	  #if (SX1278_debug_mode > 1)
		  Serial.println("## Output power has been successfully set ##");
		  Serial.println();
	  #endif
  }
  else
  {
	  state = 1;
  }

  writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  return state;
}


/*
 Function: Gets the preamble length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getPreambleLength()
{
	int8_t state = 2;
	uint8_t p_length;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getPreambleLength'");
	#endif

	state = 1;
	if( _modem == LORA )
  	{ // LORA mode
  		p_length = readRegister(REG_PREAMBLE_MSB_LORA);
  		// Saving MSB preamble length in LoRa mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_LORA);
  		// Saving LSB preamble length in LoRa mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
		#if (SX1278_debug_mode > 1)
			Serial.print("## Preamble length configured is ");
			Serial.print(_preamblelength, HEX);
			Serial.print(" ##");
			Serial.println();
		#endif
	}
	else
	{ // FSK mode
		p_length = readRegister(REG_PREAMBLE_MSB_FSK);
		// Saving MSB preamble length in FSK mode
		_preamblelength = (p_length << 8) & 0xFFFF;
		p_length = readRegister(REG_PREAMBLE_LSB_FSK);
		// Saving LSB preamble length in FSK mode
		_preamblelength = _preamblelength + (p_length & 0xFFFF);
		#if (SX1278_debug_mode > 1)
			Serial.print("## Preamble length configured is ");
			Serial.print(_preamblelength, HEX);
			Serial.print(" ##");
			Serial.println();
		#endif
	}
	state = 0;
	return state;
}

/*
 Function: Sets the preamble length in the module
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   l: length value to set as preamble length.
*/
uint8_t SX1278::setPreambleLength(uint16_t l)
{
	uint8_t st0;
	uint8_t p_length;
	int8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPreambleLength'");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	state = 1;
	if( _modem == LORA )
  	{ // LoRa mode
  		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set Standby mode to write in registers
  		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_MSB_LORA, p_length);
		p_length = (l & 0x0FF);
		// Storing LSB preamble length in LoRa mode
		writeRegister(REG_PREAMBLE_LSB_LORA, p_length);
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    // Set Standby mode to write in registers
		p_length = ((l >> 8) & 0x0FF);
  		// Storing MSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_MSB_FSK, p_length);
		p_length = (l & 0x0FF);
  		// Storing LSB preamble length in FSK mode
		writeRegister(REG_PREAMBLE_LSB_FSK, p_length);
	}

	state = 0;
	#if (SX1278_debug_mode > 1)
		Serial.print("## Preamble length ");
		Serial.print(l, HEX);
		Serial.println(" has been successfully set ##");
		Serial.println();
	#endif

	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}

/*
 Function: Gets the payload length from the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getPayloadLength()
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getPayloadLength'");
	#endif

	if( _modem == LORA )
  	{ // LORA mode
  		// Saving payload length in LoRa mode
		_payloadlength = readRegister(REG_PAYLOAD_LENGTH_LORA);
		state = 1;
	}
	else
	{ // FSK mode
  		// Saving payload length in FSK mode
		_payloadlength = readRegister(REG_PAYLOAD_LENGTH_FSK);
		state = 1;
	}

	#if (SX1278_debug_mode > 1)
		Serial.print("## Payload length configured is ");
		Serial.print(_payloadlength, HEX);
		Serial.println(" ##");
		Serial.println();
	#endif

	state = 0;
	return state;
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1278::setPacketLength()
{
	uint16_t length;

	length = _payloadlength + OFFSET_PAYLOADLENGTH;
	return setPacketLength(length);
}

/*
 Function: Sets the packet length in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   l: length value to set as payload length.
*/
int8_t SX1278::setPacketLength(uint8_t l)
{
	uint8_t st0;
	uint8_t value = 0x00;
	int8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPacketLength'");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	//----
	//	truncPayload(l);
	packet_sent.length = l;
	//
	if( _modem == LORA )
  	{ // LORA mode
  		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);    // Set LoRa Standby mode to write in registers
		writeRegister(REG_PAYLOAD_LENGTH_LORA, packet_sent.length);
		// Storing payload length in LoRa mode
		value = readRegister(REG_PAYLOAD_LENGTH_LORA);
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);    //  Set FSK Standby mode to write in registers
		writeRegister(REG_PAYLOAD_LENGTH_FSK, packet_sent.length);
		// Storing payload length in FSK mode
		value = readRegister(REG_PAYLOAD_LENGTH_FSK);
	}

	if( packet_sent.length == value )
	{
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.print("## Packet length ");
			Serial.print(packet_sent.length, DEC);
			Serial.println(" has been successfully set ##");
			Serial.println();
		#endif
	}
	else
	{
		state = 1;
	}

	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
  	//wait_with_timer2(250);
	return state;
}

/*
 Function: Gets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getNodeAddress()
{
	uint8_t st0 = 0;
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getNodeAddress'");
	#endif

	if( _modem == LORA )
	{
		// Nothing to read
		// node address is stored in _nodeAddress attribute
		state = 0;
	}
	else
	{
		// FSK mode
		st0 = readRegister(REG_OP_MODE);	// Save the previous status

		// Allowing access to FSK registers while in LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);

		// Read node address
		_nodeAddress = readRegister(REG_NODE_ADRS);

		// Getting back to previous status
		writeRegister(REG_OP_MODE, st0);

		// update state
		state = 0;
	}

	#if (SX1278_debug_mode > 1)
		Serial.print("## Node address configured is ");
		Serial.print(_nodeAddress, DEC);
		Serial.println(" ##");
		Serial.println();
	#endif
	return state;
}

/*
 Function: Sets the node address in the module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
 Parameters:
   addr: address value to set as node address.
*/
int8_t SX1278::setNodeAddress(uint8_t addr)
{
	uint8_t st0;
	uint8_t value;
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setNodeAddress'");
	#endif

	// check address value is within valid range
	if( addr > 255 )
	{
		state = -1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Node address must be less than 255 **");
			Serial.println();
		#endif
	}
	else
	{
		// Saving node address
		_nodeAddress = addr;
		st0 = readRegister(REG_OP_MODE);	  // Save the previous status

		// in LoRa mode
		state = 0;

		if( _modem == LORA )
		{
			// in LoRa mode, address is SW controlled
			// set status to success
			state = 0;
		}
		else if( _modem == FSK )
		{
			//Set FSK Standby mode to write in registers
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);

			// Storing node and broadcast address
			writeRegister(REG_NODE_ADRS, addr);
			writeRegister(REG_BROADCAST_ADRS, BROADCAST_0);

			value = readRegister(REG_NODE_ADRS);
			writeRegister(REG_OP_MODE, st0);		// Getting back to previous status

			if( value == _nodeAddress )
			{
				state = 0;
				#if (SX1278_debug_mode > 1)
					Serial.print("## Node address ");
					Serial.print(_nodeAddress, DEC);
					Serial.println(" has been successfully set ##");
					Serial.println();
				#endif
			}
			else
			{
				state = 1;
				#if (SX1278_debug_mode > 1)
					Serial.println("** There has been an error while setting address ##");
					Serial.println();
				#endif
			}
		}
	}
	return state;
}

/*
 Function: Gets the SNR value in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int8_t SX1278::getSNR()
{	// getSNR exists only in LoRa mode
  int8_t state = 2;
  uint8_t value;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getSNR'");
  #endif

  if( _modem == LORA )
  { // LoRa mode
	  state = 1;
	  value = readRegister(REG_PKT_SNR_VALUE);
	  if( value & 0x80 ) // The SNR sign bit is 1
	  {
		  // Invert and divide by 4
		  value = ( ( ~value + 1 ) & 0xFF ) >> 2;
          _SNR = -value;
      }
      else
      {
		  // Divide by 4
		  _SNR = ( value & 0xFF ) >> 2;
	  }
	  state = 0;
	  #if (SX1278_debug_mode > 0)
		  Serial.print("## SNR value is ");
		  Serial.print(_SNR, DEC);
		  Serial.println(" ##");
		  Serial.println();
	  #endif
  }
  else
  { // forbidden command if FSK mode
	state = -1;
	#if (SX1278_debug_mode > 0)
		Serial.println("** SNR does not exist in FSK mode **");
		Serial.println();
	#endif
  }
  return state;
}

/*
 Function: Gets the current value of RSSI.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getRSSI()
{
	uint8_t state = 2;
	int rssi_mean = 0;
	int total = 5;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getRSSI'");
	#endif

	if( _modem == LORA )
	{
		/// LoRa mode
		// get mean value of RSSI
		for(int i = 0; i < total; i++)
		{
			_RSSI = -OFFSET_RSSI + readRegister(REG_RSSI_VALUE_LORA);
			rssi_mean += _RSSI;
		}
		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;
		#if (SX1278_debug_mode > 0)
			Serial.print("## RSSI value is ");
			Serial.print(_RSSI, DEC);
			Serial.println(" ##");
			Serial.println();
		#endif
	}
	else
	{
		/// FSK mode
		// get mean value of RSSI
		for(int i = 0; i < total; i++)
		{
			_RSSI = (readRegister(REG_RSSI_VALUE_FSK) >> 1);
			rssi_mean += _RSSI;
		}
		rssi_mean = rssi_mean / total;
		_RSSI = rssi_mean;

		state = 0;
		#if (SX1278_debug_mode > 0)
			Serial.print("## RSSI value is ");
			Serial.print(_RSSI);
			Serial.println(" ##");
			Serial.println();
		#endif
	}

	return state;
}

/*
 Function: Gets the RSSI of the last packet received in LoRa mode.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden command for this protocol
*/
int16_t SX1278::getRSSIpacket()
{	// RSSIpacket only exists in LoRa
  int8_t state = 2;

  #if (SX1278_debug_mode > 1)
	  Serial.println();
	  Serial.println("Starting 'getRSSIpacket'");
  #endif

  state = 1;
  if( _modem == LORA )
  { // LoRa mode
	  state = getSNR();
	  if( state == 0 )
	  {
		  if( _SNR < 0 )
		  {
			  _RSSIpacket = -NOISE_ABSOLUTE_ZERO + 10.0 * SignalBwLog[_bandwidth] + NOISE_FIGURE + ( double )_SNR;
			  state = 0;
		  }
		  else
		  {
			  _RSSIpacket = readRegister(REG_PKT_RSSI_VALUE);
			  _RSSIpacket = -OFFSET_RSSI + ( double )_RSSIpacket;
			  state = 0;
		  }
	  #if (SX1278_debug_mode > 0)
		  Serial.print("## RSSI packet value is ");
		  Serial.print(_RSSIpacket, DEC);
  		  Serial.println(" ##");
		  Serial.println();
	  #endif
	  }
  }
  else
  { // RSSI packet doesn't exist in FSK mode
	state = -1;
	#if (SX1278_debug_mode > 0)
		Serial.println("** RSSI packet does not exist in FSK mode **");
		Serial.println();
	#endif
  }
  return state;
}

/*
 Function: It sets the maximum number of retries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 -->
*/
uint8_t SX1278::setRetries(uint8_t ret)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setRetries'");
	#endif

	state = 1;
	if( ret > MAX_RETRIES )
	{
		state = -1;
		#if (SX1278_debug_mode > 1)
			Serial.print("** Retries value can't be greater than ");
			Serial.print(MAX_RETRIES, DEC);
			Serial.println(" **");
			Serial.println();
		#endif
	}
	else
	{
		_maxRetries = ret;
		state = 0;
		#if (SX1278_debug_mode > 1)
			Serial.print("## Maximum retries value = ");
			Serial.print(_maxRetries, DEC);
			Serial.println(" ##");
			Serial.println();
		#endif
	}
	return state;
}

/*
 Function: Gets the current supply limit of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   rate: value to compute the maximum current supply. Maximum current is 45+5*'rate' [mA]
*/
uint8_t SX1278::getMaxCurrent()
{
	int8_t state = 2;
	uint8_t value;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getMaxCurrent'");
	#endif

	state = 1;
	_maxCurrent = readRegister(REG_OCP);

	// extract only the OcpTrim value from the OCP register
	_maxCurrent &= 0b00011111;

	if( _maxCurrent <= 15 )
	{
		value = (45 + (5 * _maxCurrent));
	}
	else if( _maxCurrent <= 27 )
	{
		value = (-30 + (10 * _maxCurrent));
	}
	else
	{
		value = 240;
	}

	_maxCurrent = value;
	#if (SX1278_debug_mode > 1)
		Serial.print("## Maximum current supply configured is ");
		Serial.print(value, DEC);
		Serial.println(" mA ##");
		Serial.println();
	#endif
	state = 0;
	return state;
}

/*
 Function: Limits the current supply of the power amplifier, protecting battery chemistries.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   rate: value to compute the maximum current supply. Range: 0x00 to 0x1B. The
   Maximum current is:
	Imax = 45+5*OcpTrim [mA] 	if OcpTrim <= 15 (120 mA) /
	Imax = -30+10*OcpTrim [mA] 	if 15 < OcpTrim <= 27 (130 to 240 mA)
	Imax = 240mA 				for higher settings
*/
int8_t SX1278::setMaxCurrent(uint8_t rate)
{
	int8_t state = 2;
	uint8_t st0;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setMaxCurrent'");
	#endif

	// Maximum rate value = 0x1B, because maximum current supply = 240 mA
	if (rate > 0x1B)
	{
		state = -1;
		#if (SX1278_debug_mode > 1)
			Serial.print("** Maximum current supply is 240 mA, ");
			Serial.println("so maximum parameter value must be 27 (DEC) or 0x1B (HEX) **");
			Serial.println();
		#endif
	}
	else
	{
		// Enable Over Current Protection
		rate |= 0b00100000;

		state = 1;
		st0 = readRegister(REG_OP_MODE);	// Save the previous status
		if( _modem == LORA )
		{ // LoRa mode
			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Set LoRa Standby mode to write in registers
		}
		else
		{ // FSK mode
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Set FSK Standby mode to write in registers
		}
		writeRegister(REG_OCP, rate);		// Modifying maximum current supply
		writeRegister(REG_OP_MODE, st0);		// Getting back to previous status
		state = 0;
	}
	return state;
}

/*
 Function: Gets the content of different registers.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getRegs()
{
	int8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getRegs'");
	#endif

	state_f = 1;
	state = getMode();			// Stores the BW, CR and SF.
	if( state == 0 )
	{
		state = getPower();		// Stores the power.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting mode **");
		#endif
	}
 	if( state == 0 )
	{
		state = getChannel();	// Stores the channel.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting power **");
		#endif
	}
	if( state == 0 )
	{
		state = getCRC();		// Stores the CRC configuration.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting channel **");
		#endif
	}
	if( state == 0 )
	{
		state = getHeader();	// Stores the header configuration.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting CRC **");
		#endif
	}
	if( state == 0 )
	{
		state = getPreambleLength();	// Stores the preamble length.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting header **");
		#endif
	}
	if( state == 0 )
	{
		state = getPayloadLength();		// Stores the payload length.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting preamble length **");
		#endif
	}
	if( state == 0 )
	{
		state = getNodeAddress();		// Stores the node address.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting payload length **");
		#endif
	}
	if( state == 0 )
	{
		state = getMaxCurrent();		// Stores the maximum current supply.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting node address **");
		#endif
	}
	if( state == 0 )
	{
		state_f = getTemp();		// Stores the module temperature.
	}
	else
	{
		state_f = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting maximum current supply **");
		#endif
	}
	if( state_f != 0 )
	{
		#if (SX1278_debug_mode > 1)
			Serial.println("** Error getting temperature **");
			Serial.println();
		#endif
	}
	return state_f;
}

/*
 Function: It truncs the payload length if it is greater than 0xFF.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::truncPayload(uint16_t length16)
{
	uint8_t state = 2;

	state = 1;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'truncPayload'");
	#endif

	if( length16 > MAX_PAYLOAD )
	{
		_payloadlength = MAX_PAYLOAD;
	}
	else
	{
		_payloadlength = (length16 & 0xFF);
	}
	state = 0;

	return state;
}

/*
 Function: It sets an ACK in FIFO in order to send it.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setACK()
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setACK'");
	#endif

	clearFlags();	// Initializing flags

	if( _modem == LORA )
	{ // LoRa mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
	}

	// Setting ACK length in order to send it
	state = setPacketLength(ACK_LENGTH);
	if( state == 0 )
	{
		// Setting ACK
		memset( &ACK, 0x00, sizeof(ACK) );
		ACK.dst = packet_received.src; // ACK destination is packet source
		ACK.src = packet_received.dst; // ACK source is packet destination
		ACK.packnum = packet_received.packnum; // packet number that has been correctly received
		ACK.length = 0;		  // length = 0 to show that's an ACK
		ACK.data[0] = _reception;	// CRC of the received packet

		// Setting address pointer in FIFO data buffer
		writeRegister(REG_FIFO_ADDR_PTR, 0x00);
		writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);

		state = 1;

		// Writing ACK to send in FIFO
		writeRegister(REG_FIFO, ACK.dst); 		// Writing the destination in FIFO
		writeRegister(REG_FIFO, ACK.src);		// Writing the source in FIFO
		writeRegister(REG_FIFO, ACK.packnum);	// Writing the packet number in FIFO
		writeRegister(REG_FIFO, ACK.length); 	// Writing the packet length in FIFO
		writeRegister(REG_FIFO, ACK.data[0]);	// Writing the ACK in FIFO

		#if (SX1278_debug_mode > 0)
			Serial.println("## ACK set and written in FIFO ##");
			// Print the complete ACK if debug_mode
			Serial.println("## ACK to send:");
			Serial.print(ACK.dst, HEX);			 	// Printing destination
			Serial.print("|");
			Serial.print(ACK.src, HEX);			 	// Printing source
			Serial.print("|");
			Serial.print(ACK.packnum, HEX);			// Printing ACK number
			Serial.print("|");
			Serial.print(ACK.length, HEX);				// Printing ACK length
			Serial.print("|");
			Serial.print(ACK.data[0], HEX);			// Printing ACK payload
			Serial.println(" ##");
			Serial.println();
		#endif

		state = 0;
		_reception = CORRECT_PACKET;		// Updating value to next packet

		wait_with_timer2(500);
	}
	return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receive()
{
	uint8_t state = 1;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'receive'");
	#endif

	// Initializing packet_received struct
	memset( &packet_received, 0x00, sizeof(packet_received) );

	// Setting Testmode
	writeRegister(0x31,0x43);
	// Set LowPnTxPllOff
	writeRegister(REG_PA_RAMP, 0x09);
	// Set LNA gain: Highest gain. LnaBoost:Improved sensitivity
	writeRegister(REG_LNA, 0x23);
	// Setting address pointer in FIFO data buffer
	writeRegister(REG_FIFO_ADDR_PTR, 0x00);
	// change RegSymbTimeoutLsb
	writeRegister(REG_SYMB_TIMEOUT_LSB, 0xFF);
	// Setting current value of reception buffer pointer
	writeRegister(REG_FIFO_RX_BYTE_ADDR, 0x00);

	// Proceed depending on the protocol selected
	if( _modem == LORA )
	{
		/// LoRa mode
		// With MAX_LENGTH gets all packets with length < MAX_LENGTH
		state = setPacketLength(MAX_LENGTH);
		// Set LORA mode - Rx
		writeRegister(REG_OP_MODE, LORA_RX_MODE);

		#if (SX1278_debug_mode > 1)
			Serial.println("## Receiving LoRa mode activated with success ##");
			Serial.println(millis());
		#endif
	}
	else
	{
		/// FSK mode
		state = setPacketLength();
		// FSK mode - Rx
		writeRegister(REG_OP_MODE, FSK_RX_MODE);
		#if (SX1278_debug_mode > 1)
			Serial.println("## Receiving FSK mode activated with success ##");
			Serial.println();
		#endif
	}

	#if (SX1278_debug_mode > 1)
		//showRxRegisters();
	#endif

	return state;
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketMAXTimeout()
{
	return receivePacketTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketTimeout()
{
	setTimeout();
	return receivePacketTimeout(_sendTime);
}

/*
 Function: Configures the module to receive information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketTimeout(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'receivePacketTimeout'");
	#endif

	// set RX mode
	state = receive();

	// if RX mode is set correctly then wait for data
	if( state == 0 )
	{
		// Wait for a new packet for 'wait' time
		if( availableData(wait) )
		{
			// If packet received, getPacket
			state_f = getPacket();
		}
		else
		{
			state_f = 1;
		}
	}
	else
	{
		state_f = state;
	}
	return state_f;
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketMAXTimeoutACK()
{
	return receivePacketTimeoutACK(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketTimeoutACK()
{
	setTimeout();
	return receivePacketTimeoutACK(_sendTime);
}

/*
 Function: Configures the module to receive information and send an ACK.
 Returns: Integer that determines if there has been any error
   state = 4  --> The command has been executed but the packet received is incorrect
   state = 3  --> The command has been executed but there is no packet received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receivePacketTimeoutACK(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;


	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'receivePacketTimeoutACK'");
	#endif

	// set RX mode
	state = receive();

	// if RX mode is set correctly then wait for data
	if( state == 0 )
	{
		// Wait for a new packet for 'wait' time
		if( availableData(wait) )
		{
			// If packet received, getPacket
			state = getPacket();
		}
		else
		{
			state = 1;
			state_f = 3;  // There is no packet received
		}
	}
	else
	{
		state = 1;
		state_f = 1; // There has been an error with the 'receive' function
	}


	if( (state == 0) || (state == 3) )
	{
		if( _reception == INCORRECT_PACKET )
		{
			state_f = 4;  // The packet has been incorrectly received
		}
		else
		{
			state_f = 1;  // The packet has been correctly received
		}
		state = setACK();
		if( state == 0 )
		{
			state = sendWithTimeout();
			if( state == 0 )
			{
			state_f = 0;
			#if (SX1278_debug_mode > 1)
				Serial.println("This last packet was an ACK, so ...");
				Serial.println("ACK successfully sent");
				Serial.println();
			#endif
			}
			else
			{
				state_f = 1; // There has been an error with the 'sendWithTimeout' function
			}
		}
		else
		{
			state_f = 1; // There has been an error with the 'setACK' function
		}
	}
	else
	{
		state_f = 1;
	}
	return state_f;
}

/*
 Function: Configures the module to receive all the information on air, before MAX_TIMEOUT expires.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t	SX1278::receiveAll()
{
	return receiveAll(MAX_TIMEOUT);
}

/*
 Function: Configures the module to receive all the information on air.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::receiveAll(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t config1;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'receiveAll'");
	#endif

	if( _modem == FSK )
	{
		/// FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);		// Setting standby FSK mode
		config1 = readRegister(REG_PACKET_CONFIG1);
		config1 = config1 & 0b11111001;			// clears bits 2-1 from REG_PACKET_CONFIG1
		writeRegister(REG_PACKET_CONFIG1, config1);		// AddressFiltering = None
	}

	#if (SX1278_debug_mode > 1)
		Serial.println("## Address filtering desactivated ##");
		Serial.println();
	#endif

	// Setting Rx mode
	state = receive();

	if( state == 0 )
	{
		// Getting all packets received in wait
		state = getPacket(wait);
	}
	return state;
}

/*
 Function: If a packet is received, checks its destination.
 Returns: bool that's 'true' if the packet is for the module and
		  it's 'false' if the packet is not for the module.
*/
bool	SX1278::availableData()
{
	return availableData(MAX_TIMEOUT);
}

/*
 Function: If a packet is received, checks its destination.
 Returns: bool that's 'true' if the packet is for the module and
		  it's 'false' if the packet is not for the module.
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
bool	SX1278::availableData(uint32_t wait)
{
	uint8_t value;
	uint8_t header = 0;
	bool forme = false;
	unsigned long previous;

	// update attribute
	_hreceived = false;

	#if (SX1278_debug_mode > 0)
		Serial.println();
		Serial.println("Starting 'availableData'");
	#endif

	previous = millis();

	if( _modem == LORA )
	{
		/// LoRa mode
		// read REG_IRQ_FLAGS
		value = readRegister(REG_IRQ_FLAGS);

		// Wait to ValidHeader interrupt in REG_IRQ_FLAGS
		while( (bitRead(value, 4) == 0) && (millis()-previous < (unsigned long)wait) )
		{
			// read REG_IRQ_FLAGS
			value = readRegister(REG_IRQ_FLAGS);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}

		// Check if ValidHeader was received
		if( bitRead(value, 4) == 1 )
		{
			#if (SX1278_debug_mode > 0)
				Serial.println("## Valid Header received in LoRa mode ##");
			#endif
			_hreceived = true;
			while( (header == 0) && (millis()-previous < (unsigned long)wait) )
			{
				// Wait for the increment of the RX buffer pointer
				header = readRegister(REG_FIFO_RX_BYTE_ADDR);

				// Condition to avoid an overflow (DO NOT REMOVE)
				if( millis() < previous )
				{
					previous = millis();
				}
			}

			// If packet received: Read first byte of the received packet
			if( header != 0 )
			{
				_destination = readRegister(REG_FIFO);
			}
		}
		else
		{
			forme = false;
			_hreceived = false;
			#if (SX1278_debug_mode > 0)
				Serial.println("** The timeout has expired **");
				Serial.println();
			#endif
		}
	}
	else
	{
		/// FSK mode
		// read REG_IRQ_FLAGS2
		value = readRegister(REG_IRQ_FLAGS2);
		// Wait to Payload Ready interrupt
		while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
		{
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}// end while (millis)
		if( bitRead(value, 2) == 1 )	// something received
		{
			_hreceived = true;
			#if (SX1278_debug_mode > 0)
				Serial.println("## Valid Preamble detected in FSK mode ##");
			#endif
			// Reading first byte of the received packet
			_destination = readRegister(REG_FIFO);
		}
		else
		{
			forme = false;
			_hreceived = false;
			#if (SX1278_debug_mode > 0)
				Serial.println("** The timeout has expired **");
				Serial.println();
			#endif
		}
	}


	/* We use '_hreceived' because we need to ensure that '_destination' value
	 * is correctly updated and is not the '_destination' value from the
	 * previously packet
	 */
	if( _hreceived == true )
	{
		#if (SX1278_debug_mode > 0)
			Serial.println("## Checking destination ##");
		#endif

		// Checking destination
		if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) )
		{ // LoRa or FSK mode
			forme = true;
			#if (SX1278_debug_mode > 0)
				Serial.println("## Packet received is for me ##");
			#endif
		}
		else
		{
			//forme = false;
			forme = true; // TODO: make this false, its like this for debug purposes!!
			#if (SX1278_debug_mode > 0)
				Serial.print("## Packet received is not for me, destination is: ");
				Serial.print(_destination, HEX);
				Serial.println(" ##");
				Serial.println(millis());
			#endif

			if(!forme){
				// If it is not a correct destination address, then change to
				// STANDBY to minimize power consumption
				if( _modem == LORA )
				{
					// Setting standby LoRa mode
					writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
				}
				else
				{
					// Setting standby FSK mode
					writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
				}
			}
		}
	}
	else
	{
		// If timeout has expired, then change to
		// STANDBY to minimize power consumption
		if( _modem == LORA )
		{
			// Setting standby LoRa mode
//~ 			writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);
		}
		else
		{
			// Setting standby FSK mode
			writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);
		}
	}
	return forme;
}

/*
 Function: It gets and stores a packet if it is received before MAX_TIMEOUT expires.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getPacketMAXTimeout()
{
	return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
int8_t SX1278::getPacket()
{
	return getPacket(MAX_TIMEOUT);
}

/*
 Function: It gets and stores a packet if it is received before ending 'wait' time.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
   state = -1 --> Forbidden parameter value for this function
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
int8_t SX1278::getPacket(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;
	uint8_t value = 0x00;
	unsigned long previous;
	bool p_received = false;

	#if (SX1278_debug_mode > 0)
		Serial.println();
		Serial.println("Starting 'getPacket'");
	#endif

	previous = millis();

	if( _modem == LORA )
	{
		/// LoRa mode
		// read REG_IRQ_FLAGS
		value = readRegister(REG_IRQ_FLAGS);

		// Wait until the packet is received (RxDone flag) or the timeout expires
		while( (bitRead(value, 6) == 0) && (millis()-previous < (unsigned long)wait) )
		{
			value = readRegister(REG_IRQ_FLAGS);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}

		// Check if 'RxDone' is true and 'PayloadCrcError' is correct
		if( (bitRead(value, 6) == 1) && (bitRead(value, 5) == 0) )
		{
			// packet received & CRC correct
			// Checking destination
			if( (_destination == _nodeAddress) || (_destination == BROADCAST_0) )
				_reception = CORRECT_PACKET;
			else _reception = INCORRECT_PACKET;
			p_received = true;	// packet correctly received
			//_reception = CORRECT_PACKET;
			#if (SX1278_debug_mode > 0)
				Serial.println("## Packet correctly received in LoRa mode ##");
			#endif
		}
		else
		{
			if( bitRead(value, 6) != 1 )
			{
				#if (SX1278_debug_mode > 0)
					Serial.println("NOT 'RxDone' flag");
				#endif
			}

			if( _CRC != CRC_ON )
			{
				#if (SX1278_debug_mode > 0)
					Serial.println("NOT 'CRC_ON' enabled");
				#endif
			}

			if( (bitRead(value, 5) == 0) && (_CRC == CRC_ON) )
			{
				// CRC is correct
				_reception = CORRECT_PACKET;
			}
			else
			{
				// CRC incorrect
				p_received = true;	// packet received (incorrect but received!!)
				_reception = INCORRECT_PACKET;
				state = 3;
				#if (SX1278_debug_mode > 0)
					Serial.println("** The CRC is incorrect **");
					Serial.println();
				#endif
			}
		}

	}
	else
	{
		/// FSK mode
		value = readRegister(REG_IRQ_FLAGS2);
		while( (bitRead(value, 2) == 0) && (millis() - previous < wait) )
		{
			value = readRegister(REG_IRQ_FLAGS2);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		} // end while (millis)
		if( bitRead(value, 2) == 1 )
		{ // packet received
 			if( (bitRead(value, 1) == 1) && (_CRC == CRC_ON) )
			{ // CRC correct
				p_received = true;
				#if (SX1278_debug_mode > 0)
					Serial.println("## Packet correctly received in FSK mode ##");
				#endif
			}
			else
			{ // CRC incorrect
				_reception = INCORRECT_PACKET;
				state = 3;
				p_received = false;
				#if (SX1278_debug_mode > 0)
					Serial.println("## Packet incorrectly received in FSK mode ##");
				#endif
			}
		}
		else
		{
			#if (SX1278_debug_mode > 0)
				Serial.println("** The timeout has expired **");
				Serial.println();
			#endif
		}
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
	}

	/* If a new packet was received correctly, now the information must be
	 * filled inside the structures of the class
	 */
	if( p_received == true )
	{
		if(_reception == CORRECT_PACKET){
			// Store the packet
			if( _modem == LORA )
			{
				/// LoRa
				// Setting address pointer in FIFO data buffer
				writeRegister(REG_FIFO_ADDR_PTR, 0x00);
				// Storing first byte of the received packet
				packet_received.dst = readRegister(REG_FIFO);
			}
			else
			{
				/// FSK
				value = readRegister(REG_PACKET_CONFIG1);
				if( (bitRead(value, 2) == 0) && (bitRead(value, 1) == 0) )
				{
					// Storing first byte of the received packet
					packet_received.dst = readRegister(REG_FIFO);
				}
				else
				{
					// Storing first byte of the received packet
					packet_received.dst = _destination;
				}
			}

			// Reading second byte of the received packet
			// Reading third byte of the received packet
			// Reading fourth byte of the received packet
			packet_received.src = readRegister(REG_FIFO);
			packet_received.packnum = readRegister(REG_FIFO);
			packet_received.length = readRegister(REG_FIFO);

			// calculate the payload length
			if( _modem == LORA )
			{
				_payloadlength = packet_received.length - OFFSET_PAYLOADLENGTH;
			}

			// check if length is incorrect
			if( packet_received.length > (MAX_LENGTH + 1) )
			{
				#if (SX1278_debug_mode > 0)
					Serial.println("Corrupted packet, length must be less than 256");
				#endif
			}
			else
			{
				// Store payload in 'data'
				for(unsigned int i = 0; i < _payloadlength; i++)
				{
					packet_received.data[i] = readRegister(REG_FIFO);
				}
				// Store 'retry'
				packet_received.retry = readRegister(REG_FIFO);

				// Print the packet if debug_mode
				#if (SX1278_debug_mode > 1)
					Serial.println("## Packet received:");
					Serial.print(packet_received.dst, HEX);	 	// Printing destination
					Serial.print("|");
					Serial.print(packet_received.src, HEX);		 	// Printing source
					Serial.print("|");
					Serial.print(packet_received.packnum, HEX);			// Printing packet number
					Serial.print("|");
					Serial.print(packet_received.length, HEX);			// Printing packet length
					Serial.print("|");
					for(unsigned int i = 0; i < _payloadlength; i++)
					{
						Serial.print(packet_received.data[i], HEX);		// Printing payload
						Serial.print("|");
					}
					Serial.print(packet_received.retry, HEX);			// Printing number retry
					Serial.println(" ##");
					Serial.println();
				#endif
				state_f = 0;
			}
		}
		else{ // incorrect but in LoRa mode, the packet is stored!!
		#if (SX1278_debug_mode > 1)
			if( _modem == LORA )
			{
				/// LoRa
				// Setting address pointer in FIFO data buffer
				Serial.println("## Packet received:");
				writeRegister(REG_FIFO_ADDR_PTR, 0x00);
				uint8_t sz = readRegister(REG_RX_NB_BYTES);
				for(uint8_t i = 0; i < sz; i++){
					Serial.print(readRegister(REG_FIFO), HEX);
					Serial.print("|");
				}
				Serial.println(" ##");
				Serial.println();
			}
			else{
				// TODO: can it be done in FSK mode??
			}
		#endif
			// if packet was NOT received
			state_f = 1;
			if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) && (state != 3) )
			{
				_retries++;
				#if (SX1278_debug_mode > 0)
					Serial.println("## Retrying to send the last packet ##");
					Serial.println();
				#endif
			}
		}
	}
	else
	{
		// if packet was NOT received
		state_f = 1;
		if( (_reception == INCORRECT_PACKET) && (_retries < _maxRetries) && (state != 3) )
		{
			_retries++;
			#if (SX1278_debug_mode > 0)
				Serial.println("## Retrying to send the last packet ##");
				Serial.println();
			#endif
		}
	}

	// Setting address pointer in FIFO data buffer to 0x00 again
	if( _modem == LORA )
	{
		writeRegister(REG_FIFO_ADDR_PTR, 0x00);
	}

	// Initializing flags
	clearFlags();

	if( wait > MAX_WAIT )
	{
		state_f = -1;
		#if (SX1278_debug_mode > 0)
			Serial.println("** The timeout must be smaller than 12.5 seconds **");
			Serial.println();
		#endif
	}

	return state_f;
}

/*
 Function: It sets the packet destination.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
 Parameters:
   dest: destination value of the packet sent.
*/
int8_t SX1278::setDestination(uint8_t dest)
{
	int8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setDestination'");
	#endif

	state = 1;
	_destination = dest; // Storing destination in a global variable
	packet_sent.dst = dest;	 // Setting destination in packet structure
	packet_sent.src = _nodeAddress; // Setting source in packet structure
	packet_sent.packnum = _packetNumber;	// Setting packet number in packet structure
	_packetNumber++;
	state = 0;

	#if (SX1278_debug_mode > 1)
		Serial.print("## Destination ");
		Serial.print(_destination, HEX);
		Serial.println(" successfully set ##");
		Serial.print("## Source ");
		Serial.print(packet_sent.src, DEC);
		Serial.println(" successfully set ##");
		Serial.print("## Packet number ");
		Serial.print(packet_sent.packnum, DEC);
		Serial.println(" successfully set ##");
		Serial.println();
	#endif
	return state;
}

/*
 Function: It sets the timeout according to the configured mode.
 Link: http://www.semtech.com/images/datasheet/sx1276.pdf
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setTimeout()
{
	uint8_t state = 2;
	uint16_t delay;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setTimeout'");
	#endif

	state = 1;
	if( _modem == LORA )
	{
		// calculate 'delay'
		delay = ((0.1*_sendTime) + 1);

		float Tpacket = timeOnAir();

		// calculate final send/receive timeout adding an offset and a random value
		//_sendTime = (uint16_t) Tpacket + (rand() % delay) + 1000;// TODO: maybe add rand() ??
		_sendTime = (uint16_t) Tpacket + (delay - 1) + 1000;

		#if (SX1278_debug_mode > 2)
			Serial.print("Tsym (ms):");
			Serial.println(Tsym);
			Serial.print("Tpreamble (ms):");
			Serial.println(Tpreamble);
			Serial.print("payloadSymbNb:");
			Serial.println(payloadSymbNb);
			Serial.print("Tpacket:");
			Serial.println(Tpacket);
		#endif

		// update state
		state = 0;
	}
	else
	{
		// update state
		_sendTime = MAX_TIMEOUT;

		// update state
		state = 0;
	}

	#if (SX1278_debug_mode > 1)
		Serial.print("Timeout to send/receive is: ");
		Serial.println(_sendTime, DEC);
	#endif

	return state;
}


/*
 Function: It gets the theoretical value of the time-on-air of the packet
 Link: http://www.semtech.com/images/datasheet/sx1276.pdf
 Returns: Float that determines the time-on-air
*/
float SX1278::timeOnAir()
{
	return timeOnAir( _payloadlength );
}

/*
 Function: It gets the theoretical value of the time-on-air of the packet
 Link: http://www.semtech.com/images/datasheet/sx1276.pdf
 Returns: Float that determines the time-on-air
*/
float SX1278::timeOnAir( uint16_t payloadlength )
{
	float BW;
	float DE = 0;
	uint8_t SF = _spreadingFactor;
	float PL = payloadlength + OFFSET_PAYLOADLENGTH;
	float H = _header;
	float CR = _codingRate;

	// Dara rate optimization enabled if SF is 11 or 12
	if( SF > 10) DE = 1.0;
	else DE = 0.0;

	// payload correction
	if( payloadlength == 0 ) PL = 255;

	// Bandwidth value setting
	if( _bandwidth == BW_125 ) 		BW = 125.0;
	else if( _bandwidth == BW_250 ) BW = 250.0;
	else if( _bandwidth == BW_500 ) BW = 500.0;
	else BW = 125.0;

	// Calculation steps:
	//Tsym = pow(2,SF)/(BW); // ms
	Tsym = (1 << SF) / (BW); // ms
	Tpreamble = (8+4.25)*Tsym;// ms
	float arg1 = (8.0*PL-4.0*SF+28.0+16.0-20.0*H)/(4.0*(SF-2.0*DE));
	if (arg1 < 0)
		payloadSymbNb = 8;
	else{
		uint32_t flr = arg1;
		payloadSymbNb = (flr+1)*(CR+4.0) + 8;
	}
	//float argument1 = ceil( (8.0*PL-4.0*SF+28.0+16.0-20.0*H)/(4.0*(SF-2.0*DE)) )*(CR+4.0);
	//float argument2 = 0;
	//payloadSymbNb = 8 + max(argument1, argument2);
	float Tpayload = payloadSymbNb * Tsym;
	float Tpacket = Tpreamble + Tpayload;

	return Tpacket;
}




/*
 Function: It sets a char array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setPayload(char *payload)
{
	uint8_t state = 2;
	uint8_t state_f = 2;
	uint16_t length16;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPayload'");
	#endif

	state = 1;
	length16 = (uint16_t)strlen(payload);
	state = truncPayload(length16);
	if( state == 0 )
	{
		// fill data field until the end of the string
		for(unsigned int i = 0; i < _payloadlength; i++)
		{
			packet_sent.data[i] = payload[i];
		}
	}
	else
	{
		state_f = state;
	}

	// In the case of FSK mode, the max payload is more restrictive
	if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
	{
		_payloadlength = MAX_PAYLOAD_FSK;
		state = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("In FSK, payload length must be less than 60 bytes.");
			Serial.println();
		#endif
	}

	// Set length with the actual counter value
	// Setting packet length in packet structure
	state_f = setPacketLength();
	return state_f;
}

/*
 Function: It sets an uint8_t array payload packet in a packet struct.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setPayload(uint8_t *payload)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPayload'");
	#endif

	state = 1;
	if( ( _modem == FSK ) && ( _payloadlength > MAX_PAYLOAD_FSK ) )
	{
		_payloadlength = MAX_PAYLOAD_FSK;
		state = 1;
		#if (SX1278_debug_mode > 1)
			Serial.println("In FSK, payload length must be less than 60 bytes.");
			Serial.println();
		#endif
	}
	for(unsigned int i = 0; i < _payloadlength; i++)
	{
		packet_sent.data[i] = payload[i];	// Storing payload in packet structure
	}
	// set length with the actual counter value
    state = setPacketLength();	// Setting packet length in packet structure
	return state;
}

/*
 Function: It sets a packet struct in FIFO in order to send it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setPacket(uint8_t dest, char *payload)
{
	int8_t state = 2;
	uint8_t st0;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPacket'");
	#endif

	// Save the previous status
	st0 = readRegister(REG_OP_MODE);
	// Initializing flags
	clearFlags();

	// Updating incorrect value
	_reception = CORRECT_PACKET;


	if (_retries == 0)
	{
		// Updating these values only if it is the first try
		// Setting destination in packet structure
		state = setDestination(dest);
		if( state == 0 )
		{
			state = setPayload(payload);
		}
	}
	else
	{
		state = setPacketLength();
		packet_sent.retry = _retries;
		#if (SX1278_debug_mode > 0)
			Serial.print("** Retrying to send last packet ");
			Serial.print(_retries, DEC);
			Serial.println(" time **");
		#endif
	}

	// Setting address pointer in FIFO data buffer
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_FIFO_ADDR_PTR, 0x00);
	if( state == 0 )
	{
		state = 1;
		// Writing packet to send in FIFO
		writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
		writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
		writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
		writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
		for( uint16_t i = 0; i < _payloadlength; i++)
		{
			writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
		}
		writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
		state = 0;
		#if (SX1278_debug_mode > 0)
			Serial.println("## Packet set and written in FIFO ##");
			// Print the complete packet if debug_mode
			Serial.print("## Packet to send: ");
			Serial.print(packet_sent.dst, HEX);			 	// Printing destination
			Serial.print("|");
			Serial.print(packet_sent.src, HEX);			 	// Printing source
			Serial.print("|");
			Serial.print(packet_sent.packnum, HEX);			// Printing packet number
			Serial.print("|");
			Serial.print(packet_sent.length, HEX);			// Printing packet length
			Serial.print("|");
			for( uint16_t i = 0; i < _payloadlength; i++)
			{
				Serial.print(packet_sent.data[i], HEX);		// Printing payload
				Serial.print("|");
			}
			Serial.print(packet_sent.retry, HEX);			// Printing retry number
			Serial.println(" ##");
		#endif
	}
	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}

/*
 Function: It sets a packet struct in FIFO in order to sent it.
 Returns:  Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::setPacket(uint8_t dest, uint8_t *payload)
{
	int8_t state = 2;
	uint8_t st0;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'setPacket'");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status
	clearFlags();	// Initializing flags

	if( _modem == LORA )
	{ // LoRa mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Stdby LoRa mode to write in FIFO
	}
	else
	{ // FSK mode
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Stdby FSK mode to write in FIFO
	}

	_reception = CORRECT_PACKET;	// Updating incorrect value to send a packet (old or new)
	if(_retries == 0)
	{ // Sending new packet
		state = setDestination(dest);	// Setting destination in packet structure
		if( state == 0 )
		{
			state = setPayload(payload);
		}
	}
	else
	{
		state = setPacketLength();
		packet_sent.retry = _retries;
		#if (SX1278_debug_mode > 0)
			Serial.print("** Retrying to send last packet ");
			Serial.print(_retries, DEC);
			Serial.println(" time **");
		#endif
	}
	writeRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
	writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
	if( state == 0 )
	{
		state = 1;
		// Writing packet to send in FIFO
		writeRegister(REG_FIFO, packet_sent.dst); 		// Writing the destination in FIFO
		writeRegister(REG_FIFO, packet_sent.src);		// Writing the source in FIFO
		writeRegister(REG_FIFO, packet_sent.packnum);	// Writing the packet number in FIFO
		writeRegister(REG_FIFO, packet_sent.length); 	// Writing the packet length in FIFO
		for(unsigned int i = 0; i < _payloadlength; i++)
		{
			writeRegister(REG_FIFO, packet_sent.data[i]);  // Writing the payload in FIFO
		}
		writeRegister(REG_FIFO, packet_sent.retry);		// Writing the number retry in FIFO
		state = 0;
		#if (SX1278_debug_mode > 0)
			Serial.println("## Packet set and written in FIFO ##");
			// Print the complete packet if debug_mode
			Serial.print("## Packet to send: ");
			Serial.print(packet_sent.dst, HEX);			 	// Printing destination
			Serial.print("|");
			Serial.print(packet_sent.src, HEX);			 	// Printing source
			Serial.print("|");
			Serial.print(packet_sent.packnum, HEX);			// Printing packet number
			Serial.print("|");
			Serial.print(packet_sent.length, HEX);			// Printing packet length
			Serial.print("|");
			for(unsigned int i = 0; i < _payloadlength; i++)
			{
				Serial.print(packet_sent.data[i], HEX);		// Printing payload
				Serial.print("|");
			}
			Serial.print(packet_sent.retry, HEX);			// Printing retry number
			Serial.println(" ##");
		#endif
	}
	writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendWithMAXTimeout()
{
	return sendWithTimeout(MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendWithTimeout()
{
	setTimeout();
	return sendWithTimeout(_sendTime);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendWithTimeout(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t value = 0x00;
	unsigned long previous;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendWithTimeout'");
	#endif

	// wait to TxDone flag
	previous = millis();
	if( _modem == LORA )
	{
		/// LoRa mode
		// Initializing flags
		clearFlags();
		// LORA mode - Tx
		writeRegister(REG_OP_MODE, LORA_TX_MODE);

		value = readRegister(REG_IRQ_FLAGS);

		// Wait until the packet is sent (TX Done flag) or the timeout expires
		while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
		{
			value = readRegister(REG_IRQ_FLAGS);
			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}
		state = 1;
	}
	else
	{
		/// FSK mode
		writeRegister(REG_OP_MODE, FSK_TX_MODE);  // FSK mode - Tx

		value = readRegister(REG_IRQ_FLAGS2);
		// Wait until the packet is sent (Packet Sent flag) or the timeout expires
		while ((bitRead(value, 3) == 0) && (millis() - previous < wait))
		{
			value = readRegister(REG_IRQ_FLAGS2);

			// Condition to avoid an overflow (DO NOT REMOVE)
			if( millis() < previous )
			{
				previous = millis();
			}
		}
		state = 1;
	}
	if( bitRead(value, 3) == 1 )
	{
		state = 0;	// Packet successfully sent
		#if (SX1278_debug_mode > 1)
			Serial.println("## Packet successfully sent ##");
			Serial.println();
		#endif
	}
	else
	{
		if( state == 1 )
		{
			#if (SX1278_debug_mode > 1)
				Serial.println("** Timeout has expired **");
				Serial.println();
			#endif
		}
		else
		{
			#if (SX1278_debug_mode > 1)
				Serial.println("** There has been an error and packet has not been sent **");
				Serial.println();
			#endif
		}
	}

	// Initializing flags
	clearFlags();
	return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeout(uint8_t dest, char *payload)
{
	return sendPacketTimeout(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeout(	uint8_t dest,
											uint8_t *payload,
											uint16_t length16)
{
	return sendPacketTimeout(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeout(uint8_t dest, char *payload)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeout'");
	#endif

	// Setting a packet with 'dest' destination address, 'payload' data field
	// and writing it in FIFO.
	state = setPacket(dest, payload);
	if (state == 0)
	{
		state = sendWithTimeout();	// Sending the packet
	}
	return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeout(	uint8_t dest,
										uint8_t *payload,
										uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeout'");
	#endif

	state = truncPayload(length16);
	if( state == 0 )
	{
		state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
	}												// and writing it in FIFO.
	else
	{
		state_f = state;
	}
	if( state_f == 0 )
	{
		state_f = sendWithTimeout();	// Sending the packet
	}
	return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeout(uint8_t dest, char *payload, uint32_t wait)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeout'");
	#endif

	state = setPacket(dest, payload);	// Setting a packet with 'dest' destination
	if (state == 0)								// and writing it in FIFO.
	{
		state = sendWithTimeout(wait);	// Sending the packet
	}
	return state;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeout(	uint8_t dest,
										uint8_t *payload,
										uint16_t length16,
										uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeout'");
	#endif

	state = truncPayload(length16);
	if( state == 0 )
	{
		state_f = setPacket(dest, payload);	// Setting a packet with 'dest' destination
	}
	else
	{
		state_f = state;
	}
	if( state_f == 0 )								// and writing it in FIFO.
	{
		state_f = sendWithTimeout(wait);	// Sending the packet
	}
	return state_f;
}

/*
 Function: Configures the module to transmit information.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeoutACK(uint8_t dest, char *payload)
{
	return sendPacketTimeoutACK(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeoutACK(uint8_t dest,
											uint8_t *payload,
											uint16_t length16)
{
	return sendPacketTimeoutACK(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACK(uint8_t dest, char *payload)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACK'");
	#endif

	state = sendPacketTimeout(dest, payload);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive();	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = state;
	}
	if( state == 0 )
	{
		if( availableData() )
		{
			state_f = getACK();	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = state;
	}

	return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACK(	uint8_t dest,
											uint8_t *payload,
											uint16_t length16)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACK'");
	#endif

	// Sending packet to 'dest' destination
	state = sendPacketTimeout(dest, payload, length16);

	// Trying to receive the ACK
	if( state == 0 )
	{
		state = receive();	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = state;
	}
	if( state == 0 )
	{
		if( availableData() )
		{
			state_f = getACK();	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = state;
	}

	return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACK(	uint8_t dest,
											char *payload,
											uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACK'");
	#endif

	state = sendPacketTimeout(dest, payload, wait);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive();	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		if( availableData() )
		{
			state_f = getACK();	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = 1;
	}

	return state_f;
}

/*
 Function: Configures the module to transmit information and receive an ACK.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACK(	uint8_t dest,
											uint8_t *payload,
											uint16_t length16,
											uint32_t wait)
{
	uint8_t state = 2;
	uint8_t state_f = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACK'");
	#endif

	state = sendPacketTimeout(dest, payload, length16, wait);	// Sending packet to 'dest' destination
	if( state == 0 )
	{
		state = receive();	// Setting Rx mode to wait an ACK
	}
	else
	{
		state_f = 1;
	}
	if( state == 0 )
	{
		if( availableData() )
		{
			state_f = getACK();	// Getting ACK
		}
		else
		{
			state_f = 9;
		}
	}
	else
	{
		state_f = 1;
	}

	return state_f;
}

/*
 Function: It gets and stores an ACK if it is received.
 Returns:
*/
uint8_t SX1278::getACK()
{
	return getACK(MAX_TIMEOUT);
}

/*
 Function: It gets and stores an ACK if it is received, before ending 'wait' time.
 Returns: Integer that determines if there has been any error
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The ACK has not been received
   state = 1  --> not used (reserved)
   state = 0  --> The ACK has been received with no errors
 Parameters:
   wait: time to wait while there is no a valid header received.
*/
uint8_t SX1278::getACK(uint32_t wait)
{
	uint8_t state = 2;
	uint8_t value = 0x00;
	unsigned long previous;
	bool a_received = false;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getACK'");
	#endif

    previous = millis();

	if( _modem == LORA )
	{ // LoRa mode
	    value = readRegister(REG_IRQ_FLAGS);
		// Wait until the ACK is received (RxDone flag) or the timeout expires
		while ((bitRead(value, 6) == 0) && (millis() - previous < wait))
		{
			value = readRegister(REG_IRQ_FLAGS);
			if( millis() < previous )
			{
				previous = millis();
			}
		}
		if( bitRead(value, 6) == 1 )
		{ // ACK received
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(REG_OP_MODE, LORA_STANDBY_MODE);	// Setting standby LoRa mode
	}
	else
	{ // FSK mode
		value = readRegister(REG_IRQ_FLAGS2);
		// Wait until the packet is received (RxDone flag) or the timeout expires
		while ((bitRead(value, 2) == 0) && (millis() - previous < wait))
		{
			value = readRegister(REG_IRQ_FLAGS2);
			if( millis() < previous )
			{
				previous = millis();
			}
		}
		if( bitRead(value, 2) == 1 )
		{ // ACK received
			a_received = true;
		}
		// Standby para minimizar el consumo
		writeRegister(REG_OP_MODE, FSK_STANDBY_MODE);	// Setting standby FSK mode
	}

	if( a_received )
	{
//----	writeRegister(REG_FIFO_ADDR_PTR, 0x00);  // Setting address pointer in FIFO data buffer
		// Storing the received ACK
		ACK.dst = _destination;
		ACK.src = readRegister(REG_FIFO);
		ACK.packnum = readRegister(REG_FIFO);
		ACK.length = readRegister(REG_FIFO);
		ACK.data[0] = readRegister(REG_FIFO);

		// Checking the received ACK
		if( ACK.dst == packet_sent.src )
		{
			if( ACK.src == packet_sent.dst )
			{
				if( ACK.packnum == packet_sent.packnum )
				{
					if( ACK.length == 0 )
					{
						if( ACK.data[0] == CORRECT_PACKET )
						{
							state = 0;
							#if (SX1278_debug_mode > 0)
							// Printing the received ACK
							Serial.println("## ACK received:");
							Serial.print(ACK.dst, HEX);
							Serial.print("|");
							Serial.print(ACK.src, HEX);
							Serial.print("|");
							Serial.print(ACK.packnum, HEX);
							Serial.print("|");
							Serial.print(ACK.length, HEX);
							Serial.print("|");
							Serial.print(ACK.data[0], HEX);
							Serial.println(" ##");
							Serial.println();
							#endif
						}
						else
						{
							state = 3;
							#if (SX1278_debug_mode > 0)
								Serial.println("** N-ACK received **");
								Serial.println();
							#endif
						}
					}
					else
					{
						state = 4;
						#if (SX1278_debug_mode > 0)
							Serial.println("** ACK length incorrectly received **");
							Serial.println();
						#endif
					}
				}
				else
				{
					state = 5;
					#if (SX1278_debug_mode > 0)
						Serial.println("** ACK number incorrectly received **");
						Serial.println();
					#endif
				}
			}
			else
			{
				state = 6;
				#if (SX1278_debug_mode > 0)
					Serial.println("** ACK source incorrectly received **");
					Serial.println();
				#endif
			}
		}
		else
		{
			state = 7;
			#if (SX1278_debug_mode > 0)
				Serial.println("** ACK destination incorrectly received **");
				Serial.println();
			#endif
		}
	}
	else
	{
		state = 8;
		#if (SX1278_debug_mode > 0)
			Serial.println("** ACK lost **");
			Serial.println();
		#endif
	}
	clearFlags();	// Initializing flags
	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeoutACKRetries(	uint8_t dest,
													char  *payload)
{
	return sendPacketTimeoutACKRetries(dest, payload, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketMAXTimeoutACKRetries(	uint8_t dest,
													uint8_t *payload,
													uint16_t length16)
{
	return sendPacketTimeoutACKRetries(dest, payload, length16, MAX_TIMEOUT);
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACKRetries(uint8_t dest, char *payload)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACKRetries'");
	#endif

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK(dest, payload);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACKRetries(uint8_t dest,
												uint8_t *payload,
												uint16_t length16)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACKRetries'");
	#endif

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK(dest, payload, length16);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACKRetries(uint8_t dest,
												char *payload,
												uint32_t wait)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACKRetries'");
	#endif

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK(dest, payload, wait);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: Configures the module to transmit information with retries in case of error.
 Returns: Integer that determines if there has been any error
   state = 9  --> The ACK lost (no data available)
   state = 8  --> The ACK lost
   state = 7  --> The ACK destination incorrectly received
   state = 6  --> The ACK source incorrectly received
   state = 5  --> The ACK number incorrectly received
   state = 4  --> The ACK length incorrectly received
   state = 3  --> N-ACK received
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::sendPacketTimeoutACKRetries(uint8_t dest,
												uint8_t *payload,
												uint16_t length16,
												uint32_t wait)
{
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'sendPacketTimeoutACKRetries'");
	#endif

	// Sending packet to 'dest' destination and waiting an ACK response.
	state = 1;
	while( (state != 0) && (_retries <= _maxRetries) )
	{
		state = sendPacketTimeoutACK(dest, payload, length16, wait);
		_retries++;
	}
	_retries = 0;

	return state;
}

/*
 Function: It gets the temperature from the measurement block module.
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
uint8_t SX1278::getTemp()
{
	uint8_t st0;
	uint8_t state = 2;

	#if (SX1278_debug_mode > 1)
		Serial.println();
		Serial.println("Starting 'getTemp'");
	#endif

	st0 = readRegister(REG_OP_MODE);	// Save the previous status

	if( _modem == LORA )
	{ // Allowing access to FSK registers while in LoRa standby mode
		writeRegister(REG_OP_MODE, LORA_STANDBY_FSK_REGS_MODE);
	}

	state = 1;
	// Saving temperature value
	_temp = readRegister(REG_TEMP);
	if( _temp & 0x80 ) // The SNR sign bit is 1
	{
		// Invert and divide by 4
		_temp = ( ( ~_temp + 1 ) & 0xFF );
    }
    else
    {
		// Divide by 4
		_temp = ( _temp & 0xFF );
	}


	#if (SX1278_debug_mode > 1)
		Serial.print("## Temperature is: ");
		Serial.print(_temp);
		Serial.println(" ##");
		Serial.println();
	#endif

	if( _modem == LORA )
	{
		writeRegister(REG_OP_MODE, st0);	// Getting back to previous status
	}

	state = 0;
	return state;
}

/*
 Function: It prints the registers related to RX
 Returns: Integer that determines if there has been any error
   state = 2  --> The command has not been executed
   state = 1  --> There has been an error while executing the command
   state = 0  --> The command has been executed with no errors
*/
void SX1278::showRxRegisters()
{
	Serial.println("\n--- Show RX register ---");

	// variable
	uint8_t reg;

	for(int i = 0x00; i < 0x80; i++)
	{
		reg = readRegister(i);
		Serial.print("Reg 0x");
		Serial.print(i, HEX);
		Serial.print(":");
		Serial.print(reg, HEX);
		Serial.println();
		wait_with_timer2(100);
	}

	Serial.println("------------------------");

}

/*
 Function: It sets the CAD mode to search Channel Activity Detection
 Returns: Integer that determines if there has been any error
   state = true   --> Channel Activity Detected
   state = false  --> Channel Activity NOT Detected
*/
bool SX1278::cadDetected()
{
	uint8_t val = 0;

	// get actual time
	unsigned long time = millis();

	// set LNA
	sx1278.writeRegister(REG_LNA,0x23);
	sx1278.clearFlags();

    sx1278.getRSSI();

	#if (SX1278_debug_mode > 1)
		Serial.print("Inside CAD DETECTION -> RSSI: ");
		Serial.println(sx1278._RSSI);
	#endif

	if( _modem == LORA )
	{
		#if (SX1278_debug_mode > 1)
			Serial.println("Set CAD mode");
		#endif

		// Setting LoRa CAD mode
		sx1278.writeRegister(REG_OP_MODE,0x87);
	}

	// Wait for IRQ CadDone
    val = sx1278.readRegister(REG_IRQ_FLAGS);
    while((bitRead(val,2) == 0) && (millis()-time)<10000 )
    {
      val = sx1278.readRegister(REG_IRQ_FLAGS);
    }

	// After waiting or detecting CadDone
	// check 'CadDetected' bit in 'RegIrqFlags' register
    if(bitRead(val,0) == 1)
    {
		#if (SX1278_debug_mode > 1)
			Serial.println("CAD true");
		#endif
		return true;
	}

	#if (SX1278_debug_mode > 1)
		Serial.println("CAD false");
	#endif
	return false;

}


SX1278	sx1278 = SX1278();
