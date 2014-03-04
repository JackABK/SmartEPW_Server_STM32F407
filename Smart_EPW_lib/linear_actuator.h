/*=============================================================================*
 *
 *   * @file     : linear_actuator.h
 *    * @author        : skyline
 *     * @data       : 2014/3/4
 *      * @brief   : linear_actuator.c header file
 *       *
 *        *============================================================================*/




#ifndef __LINEAR_ACTUATOR_H__
#define __LINEAR_ACTUATOR_H__


#define ACTU_PWM_PORT								GPIOB
#define ACTU_FRONT_PWM_PIN							GPIO_Pin_4
#define ACTU_BEHIND_PWM_PIN							GPIO_Pin_5

#define ACTU_CWCCW_PORT								GPIOD

#define ACTU_FRONT_IN1_PIN							GPIO_Pin_0
#define ACTU_FRONT_IN2_PIN							GPIO_Pin_1
#define ACTU_BEHIND_IN3_PIN							GPIO_Pin_2
#define ACTU_BEHIND_IN4_PIN							GPIO_Pin_3

enum{
	STOP,
	CW,
	CCW
};

#endif
