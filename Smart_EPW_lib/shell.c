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
#include "shell.h"



/*============================================================================*
 ** Prototype    : shell_task
 ** Description  : receive the command from the other device, and send to relative info.
 ** Input          : void *p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
void shell_task(void *p)
{
		int i , j;
		struct  receive_cmd_list * receive_cmd_type;
        printf("accept 0\n");
		while (1) {
				if(Receive_String_Ready ){
						//GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

                        /*load the accept command string to the command list structure*/
                        receive_cmd_type = received_string;


#if 0
						/*identifier the command's format, if yes, analyze the command list and perform it. */
                        if(receive_cmd_type->Identifier[0] =='c' && receive_cmd_type->Identifier[1] =='m' && receive_cmd_type->Identifier[2] =='d'){
        					PerformCommand(receive_cmd_type->DIR_cmd, receive_cmd_type->pwm_value,
        					               receive_cmd_type->Kp,      receive_cmd_type->Ki,     
        					               receive_cmd_type->Kd);
                        }
#endif

                        /*identifier the command's format, if yes, analyze the command list and perform it. */
                        if(receive_cmd_type->Identifier[0] =='c' && receive_cmd_type->Identifier[1] =='m' && receive_cmd_type->Identifier[2] =='d'){
                            PerformCommand(receive_cmd_type->group,receive_cmd_type->control_id, receive_cmd_type->value);
                            printf("accept 1\n");
                        }

                        printf("accept 2\n");
						/*clear the received string and the flag*/
						Receive_String_Ready = 0;
						for( i = 0 ; i< MAX_STRLEN ; i++){
								received_string[i]= 0;
						}
				} 

		}
}

