#include "uart.hpp"

void send_char(uint16_t ch){
  usart_send_blocking(DEBUG_USART, ch);
}
void send_data(char const* data){
  int i = 0;
  while(data[i] != 0){
    usart_send_blocking(DEBUG_USART, data[i]);
    i++;
  }
}
void send_error(char const* data){
  send_data("ERROR  -  ");
  send_data(data);
  send_data("\r\n");
}
void send_debug(char const* data){
  send_data("DEBUG  -  ");
  send_data(data);
  send_data("\r\n");
}
void send_data(uint16_t count, uint16_t* data){
  for(int i = 0; i < count; i++)
    usart_send_blocking(DEBUG_USART, data[i]);
}

void send_int(uint32_t data){
  if(data == 0){
    usart_send_blocking(DEBUG_USART, '0');
    return;
  }

  uint16_t digits[12];
  int i = 0;
  while(data != 0){
    digits[i] = data % 10;
    data /= 10;
    i++;
  }
  for(int j = i - 1; j >= 0; j--)
    usart_send_blocking(DEBUG_USART, (digits[j] + '0'));
}

void send_signed_int(int32_t data){
  if(data == 0){
    usart_send_blocking(DEBUG_USART, '0');
    return;
  }
  if(data < 0){
    data *= -1;
    usart_send_blocking(DEBUG_USART, '-');
  }

  uint16_t digits[12];
  int i = 0;
  while(data != 0){
    digits[i] = data % 10;
    data /= 10;
    i++;
  }
  for(int j = i - 1; j >= 0; j--)
    usart_send_blocking(DEBUG_USART, (digits[j] + '0'));
}

uint16_t read_char(){
  return usart_recv_blocking(DEBUG_USART);
}

void send_double(double d, int precision){
  if(d == 0){
    usart_send_blocking(DEBUG_USART, '0');
    usart_send_blocking(DEBUG_USART, '.');
    while(precision > 0){
      usart_send_blocking(DEBUG_USART, '0');
      precision--;
    }
    return;
  }
  if(d < 0){
    d *= -1;
    usart_send_blocking(DEBUG_USART, '-');
  }

  for(int i = 0; i < precision; i++) d *= 10;
  uint32_t data = d;
  uint16_t digits[12];
  int bas_say = 0;
  while(data != 0){
    digits[bas_say] = data % 10;
    data /= 10;
    bas_say++;
  }

  if(bas_say <= precision){
    usart_send_blocking(DEBUG_USART, '0');
    usart_send_blocking(DEBUG_USART, '.');
    for(int i = 0; i < precision - bas_say; i++)
      usart_send_blocking(DEBUG_USART, '0');

    for(int i = bas_say - 1; i >= 0; i--)
      usart_send_blocking(DEBUG_USART, digits[i] + '0');
  }
  else{
    for(int i = bas_say - 1; i >= precision; i--)
      usart_send_blocking(DEBUG_USART, digits[i] + '0');
    usart_send_blocking(DEBUG_USART, '.');
    for(int i = precision - 1; i >= 0; i--)
      usart_send_blocking(DEBUG_USART, digits[i] + '0');
  }
}

void _Serial::print(const char *data){
  send_data(data);
}
void _Serial::println(const char *data){
  send_data(data);
  send_data("\r\n");
}
void _Serial::println(){
  send_data("\r\n");
}
void _Serial::print(uint32_t val, Type type){
  if(type == HEX){
    usart_send_blocking(DEBUG_USART, '0');
    usart_send_blocking(DEBUG_USART, 'x');
    if(val == 0)
      usart_send_blocking(DEBUG_USART, '0');
    else{
      uint8_t digits[8] = {0};
      int i = 0;
      while(val != 0){
        digits[i] = val & 0xF;
        val >>= 4;
        ++i;
      }
      --i;
      for(; i >= 0; --i){
        uint8_t v = digits[i];
        if(v < 10)
          usart_send_blocking(DEBUG_USART, v + '0');
        else
          usart_send_blocking(DEBUG_USART, v - 10 + 'A');
      }
    }
  }
  else if(type == DEC){
    send_int(val);
  }
}
void _Serial::print(int32_t val){
  send_signed_int(val);
}
void _Serial::println(int32_t val){
  _Serial::print(val);
  send_data("\r\n");
}
void _Serial::println(uint32_t val, Type type){
  _Serial::print(val, type);
  send_data("\r\n");
}

_Serial Serial;
