#ifndef UART_HPP
#define UART_HPP

#include <libopencm3/stm32/usart.h>
#include "definitions.hpp"

void send_char(uint16_t ch);
void send_data(char const* data);
void send_error(char const* data);
void send_debug(char const* data);
void send_data(uint16_t count, uint16_t* data);
void send_int(uint32_t data);
void send_signed_int(int32_t data);
void send_double(double d, int precision);

uint16_t read_char();

enum Type{
  HEX, DEC
};

class _Serial{
public:
  static void print(int32_t val);
  static void print(uint32_t val, Type type);
  static void print(char const* data);
  static void println(char const* data);
  static void println(int32_t val);
  static void println(uint32_t val, Type type);
  static void println();
};

extern _Serial Serial;

#endif
