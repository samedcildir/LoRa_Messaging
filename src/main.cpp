#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

#include "version.hpp"
#include "init.hpp"
#include "system_functions.hpp"
#include "uart.hpp"
#include "timer.hpp"
#include "spi.hpp"
#include "definitions.hpp"
#include "led.hpp"
#if LORA_TYPE == 1
	#include "lora_arduino.hpp"
#elif LORA_TYPE == 2
	#include "lora.hpp"
#endif

void sys_tick_handler(void)
{
	/* We call this handler every 1ms */
	millis_cnt++;
}

const uint16_t data_sz = 105;
uint8_t data_to_send[data_sz];
uint16_t data_idx = 4;
bool uart_msg_ready = false;
void usart1_isr(){
	if (USART_ISR(DEBUG_USART) & USART_ISR_RXNE){ // DEBUG USART RECEIVE INTERRUPT
		uint16_t ch = usart_recv(DEBUG_USART);
		if(uart_msg_ready) return;
		if(ch == '\r' || ch == '\n') uart_msg_ready = true;
		else {
			data_to_send[data_idx] = ch;
			data_idx++;
		}
		if(data_idx == data_sz - 1) uart_msg_ready = true;
		send_char(ch);
	}

	else { // If any other interrupt occurres, clear all of the interrupts in order to prevent infinite loop
		USART_ICR(DEBUG_USART) = 0xFFFFFFFF;
	}
}

void hard_fault_handler(void){
	fatal_error_handler_with_string("hard fault\r\n");
}

#if LORA_TYPE == 1
	int e;
	char my_packet[100];
#endif

int main(){
  init_all();
	setLED();

	send_data("LoRa Master v" TOSTRING(MAIN_VERSION) "." TOSTRING(SUB_VERSION) "." TOSTRING(SUB_SUB_VERSION) "\r\n");

#if LORA_TYPE == 1
  // Power ON the module
  if (sx1278.ON() == 0) {
    Serial.println("Setting power ON: SUCCESS ");
  } else {
    Serial.println("Setting power ON: ERROR ");
  }

  // Set transmission mode and print the result
  if (sx1278.setMode<LORA_MODE>() == 0) {
    Serial.println("Setting Mode: SUCCESS ");
  } else {
    Serial.println("Setting Mode: ERROR ");
  }

  // Set header
  if (sx1278.setHeaderON() == 0) {
    Serial.println("Setting Header ON: SUCCESS ");
  } else {
    Serial.println("Setting Header ON: ERROR ");
  }

  // Select frequency channel
  if (sx1278.setChannel(LORA_CHANNEL) == 0) {
    Serial.println("Setting Channel: SUCCESS ");
  } else {
    Serial.println("Setting Channel: ERROR ");
  }

  // Set CRC
  if (sx1278.setCRC_ON() == 0) {
    Serial.println("Setting CRC ON: SUCCESS ");
  } else {
    Serial.println("Setting CRC ON: ERROR ");
  }

  // Select output power (Max, High, Intermediate or Low)
  if (sx1278.setPower(LORA_POWER) == 0) {
    Serial.println("Setting Power: SUCCESS ");
  } else {
    Serial.println("Setting Power: ERROR ");
  }

  // Set the node address and print the result
  if (sx1278.setNodeAddress(LORA_ADDRESS) == 0) {
    Serial.println("Setting node address: SUCCESS ");
  } else {
    Serial.println("Setting node address: ERROR ");
  }

  // Print a success message
  Serial.println("sx1278 configured finished");
  Serial.println();
	clearLED();

  // Print a start message
  Serial.println("sx1278 module and STM32F042: send data received from serial with ack! also receive messages");

	uint32_t msg_num = 0;
	sx1278.receive();
	while(true){ // TODO: there may be write while reading error!!
		if(uart_msg_ready){
			data_to_send[0] = msg_num & 0xFF;
			data_to_send[1] = (msg_num >> 8) & 0xFF;
			data_to_send[2] = (msg_num >> 16) & 0xFF;
			data_to_send[3] = (msg_num >> 24) & 0xFF;
			data_to_send[data_idx] = '\0';
			Serial.println("");
			Serial.println("starting to send!");

			// Send message1 and print the result
			setLED();
			e = -1;
			while (e != 0){
				if(e != -1){
				  Serial.print("Packet1 sent with error, state ");
				  Serial.println(e, DEC);
				}
		  	e = sx1278.sendPacketMAXTimeoutACK(LORA_SEND_TO_ADDRESS, data_to_send, data_idx + 1);
			}
			clearLED();
		  Serial.print("Packet1 sent, state ");
		  Serial.println(e, DEC);

		  if (e == 0)
		  	Serial.println("Successful!!");

			msg_num++;
			uart_msg_ready = false;
			sx1278.receive();
		}
		else if(sx1278.readRegister(REG_IRQ_FLAGS) != 0){
			Serial.println("starting to recv!");
		  // Receive message for 10 seconds
		  e = sx1278.receivePacketTimeoutACK(10000, false);
		  if (e == 0) {
		    Serial.println("Package received!");

				if(sx1278.packet_received.length < 4){
					Serial.println("Message size is too small!!");
					continue;
				}

				int msg_num = sx1278.packet_received.data[0];
				msg_num |= (sx1278.packet_received.data[1] << 8);
				msg_num |= (sx1278.packet_received.data[2] << 16);
				msg_num |= (sx1278.packet_received.data[3] << 24);
		    for (unsigned int i = 4; i < sx1278.packet_received.length; i++)
		      my_packet[i - 4] = (char)sx1278.packet_received.data[i];

		    Serial.print("Message No, ");
				Serial.print(msg_num);
				Serial.print(": ");
		    Serial.println(my_packet);

				setLED();
				wait_with_timer2(1000);
				clearLED();
		  } else {
		    Serial.print("Package received ERROR: ");
				Serial.println(e, DEC);

				setLED();
				wait_with_timer2(400);
				clearLED();
				wait_with_timer2(200);
				setLED();
				wait_with_timer2(400);
				clearLED();
		  }
			sx1278.receive();
		}
	}
#elif LORA_TYPE == 2
	init_lora();
	#ifdef LORA_SEND
		uint8_t data[2];
		data[0] = 'H';
		data[1] = 'i';
		while(true){
			wait_with_timer2(2); // 2msec wait
			lora_send(2, data);
		}
	#else
		lora_cont_recv();
	#endif
#endif

  while(true) { asm volatile(""); }
}
