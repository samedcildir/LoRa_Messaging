#ifndef TIMER_HPP
#define TIMER_HPP

#include <libopencm3/stm32/timer.h>

void stop_timer();
void set_timer(uint16_t limit);
void wait_with_timer(uint16_t limit);

inline bool is_timer_ended(){
  return (TIM_CR1(TIM2) & TIM_CR1_CEN) == 0;
}

void stop_timer2();
void set_timer2(uint16_t limit);
void wait_with_timer2(uint16_t limit);

inline bool is_timer2_ended(){
  return (TIM_CR1(TIM3) & TIM_CR1_CEN) == 0;
}

#endif
