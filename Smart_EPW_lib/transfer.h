/*=============================================================================
 *
 * @file     : transfer.h
 * @author        : JackABK
 * @data       : 2014/2/2
 * @brief   : shell.c header file
 *
 *============================================================================*/


#ifndef __TRANSFER_H__
#define __TRANSFER_H__
#include "EPW_command.h"
#include "stm32f4xx.h"

extern void receive_task(void *p);
extern void init_send_out_info(void);
extern void send_out_task(void);

/*determine yes or no reatch the MAX_STRLEN  */
extern uint8_t Receive_String_Ready;

/*arrange the receive of command to structure */
#pragma pack(1)               
static struct   receive_cmd_list{
        unsigned char Identifier[3];
        unsigned char group;
        unsigned char control_id;
        unsigned char value;
};
#pragma pack()     

/*list of the EPW information structure*/
typedef struct{
    char * name;
    EPW_Info_id id;
    unsigned char *value;
}EPW_info;

/* USART receive command and pwm value*/
/*should be to define to main.h or uart.h*/
enum { 
		RECEIVE_CMD,
		RECEIVE_PWM_VALUE
};


#endif /* __SHELL_H__ */
