/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
 File Name     : EPW_command.h
Version       : Initial Draft
Author        : JackABK
Created       : 2014/1/31
Last Modified :
Description   : car_command.c header file
Function List :
History       :
1.Date        : 2014/1/31
Author      : JackABK
Modification: Created file

 ******************************************************************************/

#ifndef __EPW_COMMAND_H__
#define __EPW_COMMAND_H__



extern inline void backward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void forward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void left_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void right_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void stop_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern void proc_cmd(char *cmd , uint32_t SpeedValue_left , uint32_t SpeedValue_right );


/* EPW control issue */
enum _EPW_Control_id {
    EPW_MOTOR_DIR = 100, 
    EPW_MOTOR_PWM,
    EPW_ACTUATOR_A,
    EPW_ACTUATOR_B,
    EPW_PID_ALG_KP,
    EPW_PID_ALG_KI,
    EPW_PID_ALG_KD,
    EPW_PID_ALG,
    EPW_GPIO,
    EPW_Servo_0,
    EPW_Servo_1,
};
typedef enum _EPW_Control_id EPW_Control_id;


/* EPW info issue */
enum _EPW_Info_id {
    EPW_ULTRASONIC_0 =200,        
    EPW_ULTRASONIC_1,
    EPW_ULTRASONIC_2,
    EPW_ULTRASONIC_3,
    EPW_ACTUATOR_LIMIT_SWITCH_A,
    EPW_ACTUATOR_LIMIT_SWITCH_B,
    EPW_LEFT_RPM,
    EPW_RIGHT_RPM,
};
typedef enum _EPW_Info_id EPW_Info_id;


/* commands which can be send to the output plugin */
typedef enum _cmd_group cmd_group;
enum  _cmd_group {
    OUT_EPW_CMD = 0, // "EPW" means electric power wheelchair, include the EPW's motor, linear actuator etc...
    SEND_EPW_INFO = 1, // include the ultrasonic, limit switch, etc...
};




#endif /* __CAR_COMMAND_H__ */
