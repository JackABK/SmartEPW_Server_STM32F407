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


/*=================Re-define the all by pins=========================*/
/****Motor****/            
#define MOTOR_PWM_PORT                                            GPIOD
#define MOTOR_LEFT_PWM_PIN                                        GPIO_Pin_13
#define MOTOR_RIGHT_PWM_PIN                                       GPIO_Pin_15

#define MOTOR_CWCCW_PORT                                          GPIOD
#define MOTOR_LEFT_CWCCW_PIN                                      GPIO_Pin_12
#define MOTOR_RIGHT_CWCCW_PIN                                     GPIO_Pin_14


/****Encoder****/
#define ENCODER_PORT                                              GPIOA
#define ENCODER_LEFT_PHASE_A_PIN                                  GPIO_Pin_0      /*the inturrupt is maping to EXTI0*/
#define ENCODER_RIGHT_PHASE_A_PIN                                 GPIO_Pin_1      /*the inturrupt is maping to EXTI1*/
#define ENCODER_LEFT_PHASE_B_PIN                                  GPIO_Pin_2
#define ENCODER_RIGHT_PHASE_B_PIN                                 GPIO_Pin_3
/*===============end of define  the all by pins========================*/

extern void attachInterrupt(uint32_t EXTI_LineX);
extern void Car_State_Polling();
extern void detachInterrupt(uint32_t EXTI_LineX);
extern void EXTI0_IRQHandler();
extern void EXTI1_IRQHandler();
extern void init_car();
extern void init_encoder(void);
extern void init_External_Interrupt(void);
extern void init_motor(void);
extern void PerformCommand(unsigned char DIR_cmd, 
                           unsigned char pwm_value,
                           unsigned char Kp,
                           unsigned char Ki,
                           unsigned char Kd);

extern void PID_Algorithm_Polling(void);
extern void init_motor_CWCCW(void);



#endif /* __CAR_BEHAVIOR_H__ */
