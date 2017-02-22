#include "init.hpp"

#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>

#include "system_functions.hpp"
#include "uart.hpp"
#include "timer.hpp"
#include "definitions.hpp"
#include "spi.hpp"

void init_clock(){
	rcc_clock_setup_in_hsi_out_48mhz(); // TODO: there is a 48MHz RC Oscillator. Should I use that instead??
	                                    // If you change HSI, change I2C timing too.
}

void init_gpio(){
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	gpio_mode_setup(SPI_SCK_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_SCK_PIN);
	gpio_mode_setup(SPI_MISO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SPI_MISO_PIN);
	gpio_mode_setup(SPI_MOSI_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_MOSI_PIN);
	gpio_mode_setup(SPI_NSS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SPI_NSS_PIN);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
	unselect_chip();
}

void init_usart(){
	rcc_periph_clock_enable(DEBUG_USART_RCC);

	gpio_mode_setup(DEBUG_USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, DEBUG_USART_PIN_TX | DEBUG_USART_PIN_RX);
	gpio_set_af(DEBUG_USART_PORT, DEBUG_USART_AF, DEBUG_USART_PIN_TX | DEBUG_USART_PIN_RX);

	/* Setup DEBUG_USART parameters. */
	usart_set_databits(DEBUG_USART, 8);
	usart_set_baudrate(DEBUG_USART, DEBUG_USART_SPEED);
	usart_set_stopbits(DEBUG_USART, USART_CR2_STOP_1_0BIT);
	usart_set_mode(DEBUG_USART, USART_MODE_TX_RX);
	usart_set_parity(DEBUG_USART, USART_PARITY_NONE);
	usart_set_flow_control(DEBUG_USART, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(DEBUG_USART);

#if DEBUG_MODE
	send_debug("Init UART Done!");
#endif
}

void init_systick(){
  /* We are using AHB = 48MHz */
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_set_reload(479999); /* 100 tick per second */
  systick_interrupt_enable();
  systick_counter_enable();

	#if DEBUG_MODE
		send_debug("Init SYSTICK Done!");
	#endif
}

void init_mco(){
	gpio_mode_setup(MCO_OUT_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, MCO_OUT_PIN);
	gpio_set_output_options(MCO_OUT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, MCO_OUT_PIN);
	gpio_set_af(MCO_OUT_PORT, GPIO_AF0, MCO_OUT_PIN);
	rcc_set_mco(RCC_CFGR_MCO_SYSCLK);

	#if DEBUG_MODE
		send_debug("Init MCO Done!");
	#endif
}

void init_timer(){
	// first timer for general use
	rcc_periph_clock_enable(RCC_TIM2);

	timer_reset(TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	// TIM_CR1(TIM2) &= ~TIM_CR1_URS;
	timer_set_prescaler(TIM2, (rcc_apb1_frequency / 10000)); // 0.1ms per tick
	timer_disable_preload(TIM2);

	timer_one_shot_mode(TIM2);  // NOTE: STOPS after first event

	#if DEBUG_MODE
		send_debug("Init TIM2 Done!");
	#endif

	// second timer only for ext_adc timeout
	rcc_periph_clock_enable((rcc_periph_clken) RCC_TIM3);

	timer_reset(TIM3);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	// TIM_CR1(TIM3) &= ~TIM_CR1_URS;
	timer_set_prescaler(TIM3, (rcc_apb1_frequency / 1000)); // 1ms per tick
	timer_disable_preload(TIM3);

	timer_one_shot_mode(TIM3);  // NOTE: STOPS after first event
	wait_with_timer(100); // first wait is not processed for some reason!!
	wait_with_timer2(10); // first wait is not processed for some reason!!

	#if DEBUG_MODE
		send_debug("Init TIM3 Done!");
	#endif
}
