/*=============================================================================
 *
 * @file     : shell.h
 * @author        : JackABK
 * @data       : 2014/2/2
 * @brief   : shell.c header file
 *
 *============================================================================*/


#ifndef __SHELL_H__
#define __SHELL_H__

extern void shell_task(void *p);


/*determine yes or no reatch the MAX_STRLEN  */
extern uint8_t Receive_String_Ready;

/*arrange the receive of command to structure */
#pragma pack(1)               
static struct   receive_cmd_list{
        unsigned char Identifier[3];
        unsigned char group;
        unsigned char control_id;
        unsigned char value;
        /*
		unsigned char DIR_cmd;
		unsigned char pwm_value;
        unsigned char Kp;
        unsigned char Ki;
        unsigned char Kd;
        */
};
#pragma pack()     

/* USART receive command and pwm value*/
/*should be to define to main.h or uart.h*/
enum { 
		RECEIVE_CMD,
		RECEIVE_PWM_VALUE
};


#endif /* __SHELL_H__ */
