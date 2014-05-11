/*=============================================================================
  *
  * @file     : linear_actuator.h
  * @author        : skyline
  * @data       : 2014/3/10
  * @brief   : linear_actuator.c header file
  *
  *============================================================================*/
#ifndef __LINEAR_ACTUATOR_H__
#define __LINEAR_ACTUATOR_H__

#include "stm32f4xx_syscfg.h"


#define ACTU_PWM_PORT							GPIOB
#define ACTU_A_PWM_PIN							GPIO_Pin_4
#define ACTU_B_PWM_PIN							GPIO_Pin_5

#define ACTU_CWCCW_PORT							GPIOD

#define ACTU_A_IN1_PIN							GPIO_Pin_0
#define ACTU_A_IN2_PIN							GPIO_Pin_1
#define ACTU_B_IN3_PIN							GPIO_Pin_2
#define ACTU_B_IN4_PIN							GPIO_Pin_3


/*
 *Limit Switch
 */
#define LS_READ_PORT							GPIOC
#define LS_A_UPPER_PIN							GPIO_Pin_0
#define LS_A_LOWER_PIN							GPIO_Pin_1
#define LS_B_UPPER_PIN							GPIO_Pin_2
#define LS_B_LOWER_PIN							GPIO_Pin_3
enum{
	STOP,
	CW,
	CCW
};



struct limit_switch_info{
	uint8_t actuatorA_LS_state;
	uint8_t actuatorB_LS_state;
};



void get_LimitSwitch_A_lower();
int get_LimitSwitch_A_upper_Vol();
void get_LimitSwitch_B_lower();
void get_LimitSwitch_B_upper();
static void init_CWCCW();
void init_linear_actuator();
static void init_LS_ADC();
static void init_PWM();
void set_linearActuator_A_cmd(int flag , int pwm_value);
void set_linearActuator_B_cmd(int flag , int pwm_value);
int get_LimitSwitch_Vt(uint8_t actuator_id, uint8_t dir_flag);



#endif /* __LINEAR_ACTUATOR_H__ */
