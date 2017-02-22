#ifndef SPI_HPP
#define SPI_HPP

#include "definitions.hpp"
#include <libopencm3/stm32/gpio.h>

inline uint16_t get_miso(){
  return gpio_get(SPI_MISO_PORT, SPI_MISO_PIN);
}
inline void set_mosi(){
  gpio_set(SPI_MOSI_PORT, SPI_MOSI_PIN);
}
inline void clear_mosi(){
  gpio_clear(SPI_MOSI_PORT, SPI_MOSI_PIN);
}
inline void set_sck(){
  gpio_set(SPI_SCK_PORT, SPI_SCK_PIN);
}
inline void clear_sck(){
  gpio_clear(SPI_SCK_PORT, SPI_SCK_PIN);
}
inline void select_chip(){
  gpio_clear(SPI_NSS_PORT, SPI_NSS_PIN);
}
inline void unselect_chip(){
  gpio_set(SPI_NSS_PORT, SPI_NSS_PIN);
}

void spi_write(uint8_t reg, uint8_t sz, uint8_t *data);
void spi_read(uint8_t reg, uint8_t sz, uint8_t *data);

void spi_write8(uint8_t reg, uint8_t data);
uint8_t spi_read8(uint8_t reg);

#endif
