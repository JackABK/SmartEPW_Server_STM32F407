/*=============================================================================
  *
  * @file     : delay.c
  * @author        : JackABK
  * @data       : 2014/4/22
  * @brief   : delay time for the stm32f4
  *
  *============================================================================*/
#include "stm32f4xx.h"

void delay(u32 ms) {
    ms *= 12000;
    while(ms--) {
        __NOP();
    }
}


void delay_us(u32 us) {
    us *= 12;
    while(us--) {
        __NOP();
    }
}
