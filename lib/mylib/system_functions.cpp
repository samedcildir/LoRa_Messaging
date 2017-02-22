#include "system_functions.hpp"
#include "init.hpp"
#include "uart.hpp"

volatile uint32_t millis_cnt = 0;

void init_all(){
  init_clock();
  init_gpio();
  init_usart();
  init_systick();
  // init_mco();        // NOTE: test done, works as expected (48MHz)
  init_timer();         // NOTE: first test done, counts with 1sec interval correctly

	#if DEBUG_MODE
		send_debug("Init ALL Done!");
	#endif
}

// TODO: calibrate this
// NOTE: do not use with numbers bigger than 397682
void mDELAY(uint32_t ms){
	/*uint32_t lim = ms * 32000; // approximately 1ms = 32000 cycle
	while(lim--) asm ("");*/
  asm(
    ".syntax unified\n\t"
    "MOV R0, %[mlim]\n\t"
    "mloop_start:\n\t"
    "SUBS R0, R0, #1\n\t"
    "BNE mloop_start\n\t"
    ".syntax divided\n\t" : : [mlim] "r" (ms * 10800)
  );
}

// TODO: calibrate this
// NOTE: do not use with numbers bigger than 390451572
// NOTE: it is not as correct as mDELAY
void uDELAY(uint32_t us){
	/*uint32_t lim = ms * 32000; // approximately 1ms = 32000 cycle
	while(lim--) asm ("");*/
  asm(
    ".syntax unified\n\t"
    "MOV R0, %[ulim]\n\t"
    "uloop_start:\n\t"
    "SUBS R0, R0, #1\n\t"
    "BNE uloop_start\n\t"
    ".syntax divided\n\t" : : [ulim] "r" (us * 11)
  );
}

void fatal_error_handler(uint32_t type){
  send_data("FATAL ERROR: ");
  send_int(type);
  send_data("\r\n");

  while(1) asm volatile("");
}

void fatal_error_handler_with_string(const char* name){
  send_data("FATAL ERROR: ");
  send_data(name);
  send_data("\r\n");

  while(1) asm volatile("");
}

void error(uint32_t type){
  send_data("ERROR: ");
  send_int(type);
  send_data("\r\n");
}
