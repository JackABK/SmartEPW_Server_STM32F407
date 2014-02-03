/*=============================================================================
 *
 * @file     : shell.c
 * @author        : JackABK
 * @data       : 2014/2/2
 * @brief   :  trasfer the message from other device and could be achieve interactive effect.
 *
 *============================================================================*/

#include "FreeRTOS.h"
#include "stm32f4xx_usart.h"
#include "uart.h"

/*determine yes or no reatch the MAX_STRLEN  */
uint8_t Receive_String_Ready  =  0;

/*arrange the receive of command to structure */
#pragma pack(1)               
static struct   receive_cmd_list{
		unsigned char motion_command ;
		unsigned char  pwm_value ;
};
#pragma pack()     


/* USART receive command and pwm value*/
/*should be to define to main.h or uart.h*/
enum { 
		RECEIVE_CMD,
		RECEIVE_PWM_VALUE
};

/*============================================================================*
 ** Prototype    : shell_task
 ** Description  : receive the command from the other device, and send to relative info.
 ** Input          : void *p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
void shell_task(void *p)
{
		int i ;
		struct  receive_cmd_list   receive_cmd_type;

		while (1) {
				if(Receive_String_Ready ){
						GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

#if 0
						receive_cmd_type.motion_command = received_string[0];
						receive_cmd_type.pwm_value =       (received_string[1] &0xff )<<24   | \
														   (received_string[2] &0xff) <<16  |  \
														   (received_string[3] &0xff) <<8       |  \
														   (received_string[4]) ;
#endif

						/* put the receive command to command structure */
						receive_cmd_type.motion_command = received_string[RECEIVE_CMD];
						receive_cmd_type.pwm_value = received_string[RECEIVE_PWM_VALUE];

						/*analyze the command and do it. */
						PerformCommand(receive_cmd_type.motion_command, receive_cmd_type.pwm_value);


						/*print out to other device to check acceped */
						//printf("cmd :%c  pwm : %c\n" ,received_string[RECEIVE_CMD] , received_string[RECEIVE_PWM_VALUE]);

						/*clear the received string and the flag*/
						Receive_String_Ready = 0;
						for( i = 0 ; i< MAX_STRLEN ; i++){
								received_string[i]= 0;
						}
				} 

		}
}

