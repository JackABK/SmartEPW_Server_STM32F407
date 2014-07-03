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
#include "transfer.h"
#include "EPW_command.h"


extern float rpm_left_motor;
extern float rpm_right_motor;

extern int encoder_left_counter;
extern int encoder_right_counter;

/*============================================================================*
 ** Prototype    : receive_task
 ** Description  : receive the command from the other device, and send to relative info.
 ** Input          : void *p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
void receive_task(void *p)
{
	    int i , j;
		struct  receive_cmd_list * receive_cmd_type;
        
		while (1) {
				if(Receive_String_Ready ){
						//GPIO_ToggleBits(GPIOD,GPIO_Pin_14);

                        /*load the accept command string to the command list structure*/
                        receive_cmd_type = received_string;

                        /*identifier the command's format, if yes, analyze the command list and perform it. */
                        if(receive_cmd_type->Identifier[0] =='c' && receive_cmd_type->Identifier[1] =='m' && receive_cmd_type->Identifier[2] =='d'){
                            PerformCommand(receive_cmd_type->group,receive_cmd_type->control_id, receive_cmd_type->value);
                            
                        }

                        
						/*clear the received string and the flag*/
						Receive_String_Ready = 0;
						for( i = 0 ; i< MAX_STRLEN ; i++){
								received_string[i]= 0;
						}
				} 

		}
}

#define SEND_OUT_PERIOD 30 /*ms*/
#define DECLARE_EPW_INFO(info_name, info_id, info_value) {.name=info_name, .id=info_id, .value=info_value}

void send_out_task(void *p){
    cmd_group EPW_cmd_group = OUT_EPW_CMD;
    int i;
    unsigned char distance_cm[4]={0};
    unsigned char motor_L_rpm=0,motor_R_rpm=0;
    EPW_info  EPW_info_list[] = {                                         
        DECLARE_EPW_INFO("ultrasonic 0" , EPW_ULTRASONIC_0 , distance_cm),
        DECLARE_EPW_INFO("ultrasonic 1" , EPW_ULTRASONIC_1 , distance_cm+1),
        DECLARE_EPW_INFO("ultrasonic 2" , EPW_ULTRASONIC_2 , distance_cm+2),
        DECLARE_EPW_INFO("ultrasonic 3" , EPW_ULTRASONIC_3 , distance_cm+3),
        DECLARE_EPW_INFO("EPW_LEFT_RPM" , EPW_LEFT_RPM , &motor_L_rpm),
        DECLARE_EPW_INFO("EPW_RIGHT_RPM", EPW_RIGHT_RPM ,&motor_R_rpm)
    };
   
    while(1){
        /*update the new value*/
        distance_cm[0]=(unsigned char)Get_CH1Distance();
        distance_cm[1]=(unsigned char)Get_CH2Distance();
        distance_cm[2]=(unsigned char)Get_CH3Distance();
        distance_cm[3]=(unsigned char)Get_CH4Distance();
        motor_L_rpm=(unsigned char)rpm_left_motor;
        motor_R_rpm=(unsigned char)rpm_right_motor;

        
#ifdef DEBUG_MODE
        printf("----------------------Debug mode-------------------------------\r\n");
        for(i=0;i<sizeof(EPW_info_list)/sizeof(EPW_info);i++){
            printf("EPW_issue name:%s   EPW_issue_id:%d EPW_issue_value:%d\r\n",EPW_info_list[i].name,EPW_info_list[i].id, *(EPW_info_list[i].value));
        }
        printf("---------------------End of debug mode-------------------------\r\n\r\n\r\n");
        
        printf("%d %d %d %d  encoder counter: %d %d\r\n",distance_cm[0],distance_cm[1],distance_cm[2],distance_cm[3],encoder_left_counter,encoder_right_counter);
        
#else 
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ULTRASONIC_0,distance_cm[0] );
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ULTRASONIC_1,distance_cm[1] );        
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ULTRASONIC_2,distance_cm[2] );        
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ULTRASONIC_3,distance_cm[3] );        
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ACTUATOR_LIMIT_SWITCH_A,get_Linear_Actuator_A_LS_State() ); /*the actuatorA of limit switch state*/
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_ACTUATOR_LIMIT_SWITCH_B,get_Linear_Actuator_B_LS_State() ); /*the actuatorB of limit switch state*/
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_LEFT_RPM, (unsigned char)rpm_left_motor);        
        printf("cmd%c%c%cend",EPW_cmd_group,EPW_RIGHT_RPM,(unsigned char) rpm_right_motor);
        fflush(0);/*force clear buffer and send out the other device. */

#endif
        vTaskDelay(SEND_OUT_PERIOD);
    }
}

