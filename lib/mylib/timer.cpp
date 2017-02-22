#include "timer.hpp"

void stop_timer(){
  timer_disable_counter(TIM2);
}

/// limit is in 0.1ms (10000 for 1 sec)
void set_timer(uint16_t limit){
  stop_timer();
  while(!is_timer_ended()) asm(""); // wait for stop of the timer

  timer_set_period(TIM2, limit);

  timer_enable_counter(TIM2);
  while(is_timer_ended()) asm(""); // wait for start of the timer
}

// NOTE: may not work with limit < 5
void wait_with_timer(uint16_t limit){
  set_timer(limit - 1); // for some reason, it skips one cycle
  while(!is_timer_ended()) asm("");
}


void stop_timer2(){
  timer_disable_counter(TIM3);
}
/// limit is in 1ms (1000 for 1 sec)
void set_timer2(uint16_t limit){
  stop_timer2();
  while(!is_timer2_ended()) asm(""); // wait for stop of the timer

  timer_set_period(TIM3, limit);
	//TIM_EGR(TIM3) &= ~TIM_EGR_UG;

  timer_enable_counter(TIM3);
  while(is_timer2_ended()) asm(""); // wait for start of the timer
}
// NOTE: may not work with limit < 5
void wait_with_timer2(uint16_t limit){
  set_timer2(limit - 1); // for some reason, it skips one cycle
  while(!is_timer2_ended()) asm("");
}
