/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
 File Name     : car_command.h
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

#ifndef __CAR_COMMAND_H__
#define __CAR_COMMAND_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

extern inline void backward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void forward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void left_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void right_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern inline void stop_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
extern void proc_cmd(char *cmd , uint32_t SpeedValue_left , uint32_t SpeedValue_right );

/* this a chip set ing eintldkoe, ithkoe , dksods . */
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __CAR_COMMAND_H__ */
