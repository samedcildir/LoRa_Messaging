#ifndef SYSTEM_FUNCTIONS_HPP
#define SYSTEM_FUNCTIONS_HPP

#include <libopencm3/stm32/dac.h>

void init_all();

void mDELAY(uint32_t ms);
void uDELAY(uint32_t us);

void fatal_error_handler(uint32_t type);
void fatal_error_handler_with_string(const char* name);
void error(uint32_t type);

extern volatile uint32_t millis_cnt;
inline uint32_t millis(){
  return millis_cnt;
}

// b[n], b[n-1], .. , b[0] // NOTE: Little Endian /// low to high
template<typename T>
T get_data(uint8_t *data_arr){
  T d = 0;
  for(uint16_t i = 0; i < sizeof(T); i++)
    d |= (data_arr[i] << (8 * i));
  return d;
}

// b[0], b[1], .. , b[n] // NOTE: Big Endian /// high to low
template<typename T>
T get_data_rev(uint8_t *data_arr){
  T d = 0;
  for(uint16_t i = 0; i < sizeof(T); i++)
    d |= (data_arr[i] << (8 * (sizeof(T) - 1 - i)));
  return d;
}

// NOTE: Little Endian
template<typename T>
void get_data_arr(T data, uint8_t *data_arr){
  for(uint16_t i = 0; i < sizeof(T); i++){
    data_arr[i] = data & 0xFF;
    data >>= 8;
  }
}

template<typename T>
bool compare(uint8_t *data_arr, T org_data){
  return get_data<T>(data_arr) == org_data;
}

#endif
