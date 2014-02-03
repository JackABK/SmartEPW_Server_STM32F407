/*=============================================================================
  *
  * @file     : car_behavior.h
  * @author        : JackABK
  * @data       : 2014/2/3
  * @brief   : car_behavior.c header file
  *
  *============================================================================*/
#ifndef __CAR_BEHAVIOR_H__
#define __CAR_BEHAVIOR_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

extern void attachInterrupt(uint32_t EXTI_LineX);
extern void Car_State_Polling();
extern void detachInterrupt(uint32_t EXTI_LineX);
extern void EXTI0_IRQHandler();
extern void EXTI1_IRQHandler();
extern void init_car();
extern void init_encoder(void);
extern void init_External_Interrupt(void);
extern void init_motor(void);
extern void PerformCommand(unsigned char acpt_cmd , unsigned char acpt_pwm_value);
extern void PI_Algorithm_Polling(void);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CAR_BEHAVIOR_H__ */
