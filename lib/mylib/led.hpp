#ifndef LED_HPP
#define LED_HPP

#include <libopencm3/stm32/gpio.h>
#include "definitions.hpp"

inline void setLED(){
  gpio_set(LED_PORT, LED_PIN);
}
inline void clearLED(){
  gpio_clear(LED_PORT, LED_PIN);
}

#endif
