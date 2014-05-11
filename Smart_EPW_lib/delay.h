/*=============================================================================
  *
  * @file     : delay.h
  * @author        : JackABK
  * @data       : 2014/4/22
  * @brief   : delay.c header file
  *
  *============================================================================*/
#ifndef __DELAY_H__
#define __DELAY_H__
#include "stm32f4xx.h"

#if 0
/* Private function prototypes -----------------------------------------------*/
extern void Delay(__IO uint32_t nTime);
extern void TimingDelay_Decrement(void);
#endif


/******************************************************************************
 * @Brief STM32F4 168MHz Time delay Function Reference
 *
 **/
void delay(u32 ms);
void delay_us(u32 us);
#endif /* __DELAY_H__ */
