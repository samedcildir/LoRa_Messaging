#include "spi.hpp"

// Private Function, assumes NSS and SCK low
void write_byte(uint8_t val){
  for(int i = 8; i; --i){
    if (val & 0x80)
      set_mosi();
    else clear_mosi();
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    set_sck();
    val <<= 1;
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    clear_sck();
  }
}
// Private Function, assumes NSS and SCK low
uint8_t read_byte(){
  uint8_t res = 0;
  for(int i = 8; i; --i){
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    set_sck();
    res <<= 1;
    if (get_miso())
      res |= 1;
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    asm volatile("nop");
    clear_sck();
  }

  return res;
}

void spi_write(uint8_t reg, uint8_t sz, uint8_t *data){
  clear_sck();
  select_chip();

  write_byte(reg | 0x80);
  for(int i = sz; i; --i) write_byte(data[sz - i]);

  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  unselect_chip();
}

void spi_read(uint8_t reg, uint8_t sz, uint8_t *data){
  clear_sck();
  select_chip();

  write_byte(reg & 0x7F);
  clear_mosi();
  for(int i = sz; i; --i) data[sz - i] = read_byte();

  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  unselect_chip();
}

void spi_write8(uint8_t reg, uint8_t data){
  clear_sck();
  select_chip();

  write_byte(reg | 0x80);
  write_byte(data);

  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  unselect_chip();
}
uint8_t spi_read8(uint8_t reg){
  clear_sck();
  select_chip();

  write_byte(reg & 0x7F);
  clear_mosi();
  uint8_t res = read_byte();

  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");
  unselect_chip();
  return res;
}
