#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#define DEBUG_MODE        1
#define LORA_TYPE         1 // 1 -> arduino library port, 2 -> my try
// #define LORA_SEND           // not defined -> recv, defined -> send
#define SX1278_debug_mode 0 // 0,1,2,3 // best one is 2

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#if LORA_TYPE == 1
	//#define LORA_MODE  4
	#define LORA_CHANNEL  CH_6_BW_125
	#define LORA_MODE  16 // [13 went to dorm], [15 medium toa, bad battery life, passed metal test], [16 short toa, very good battery life, ]
	//#define LORA_CHANNEL  CH_6

	#define LORA_POWER  'I' // 'M'=20dbm - 'H'=14dbm - 'I'=8dbm - 'L'=2dbm

	#ifdef LORA_SEND
		#define LORA_ADDRESS  				2
		#define LORA_SEND_TO_ADDRESS  4
	#else
		#define LORA_ADDRESS  				4
		#define LORA_SEND_TO_ADDRESS  2
	#endif
#endif

#define MCO_OUT_PORT      GPIOA
#define MCO_OUT_PIN       GPIO8

#define SPI_SCK_PORT      GPIOB
#define SPI_SCK_PIN       GPIO7

#define SPI_MISO_PORT     GPIOB
#define SPI_MISO_PIN      GPIO6

#define SPI_MOSI_PORT     GPIOB
#define SPI_MOSI_PIN      GPIO5

#define SPI_NSS_PORT      GPIOB
#define SPI_NSS_PIN       GPIO4

#define LED_PORT          GPIOB
#define LED_PIN           GPIO3

// USART RELATED DEFINITIONS //
#define DEBUG_USART USART1
#define DEBUG_USART_RCC RCC_USART1
#define DEBUG_USART_PORT GPIOA
#define DEBUG_USART_PIN_TX GPIO9
#define DEBUG_USART_PIN_RX GPIO10
#define DEBUG_USART_AF GPIO_AF1
#define DEBUG_USART_NVIC NVIC_USART1_IRQ
#define DEBUG_USART_SPEED 115200

#endif
