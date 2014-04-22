/*=============================================================================
  *
  * @file     : delay.c
  * @author        : JackABK
  * @data       : 2014/4/22
  * @brief   : delay time for the stm32f4
  *
  *============================================================================*/
#include "stm32f4xx.h"

void delay(uint32_t ms) {
		ms *= 3360;
		while(ms--) {
				__NOP();
		}
}

void delay_us(uint32_t us) {
		us *= 3;
		while(us--) {
				__NOP();
		}
}
