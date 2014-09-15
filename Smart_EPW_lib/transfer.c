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
#include "timers.h"


extern float rpm_left_motor;
extern float rpm_right_motor;

extern int encoder_left_counter;
extern int encoder_right_counter;

extern int encoder_left_counter_sum;
extern int encoder_right_counter_sum;

extern float Kp_left;
extern float Ki_left;
extern float Kd_left;

extern float Kp_right;
extern float Ki_right;
extern float Kd_right;

extern int pwm_value_left_mcu;
extern int pwm_value_right_mcu;


xTimerHandle send_out_info_Timers;
#define SEND_OUT_POLLING_PERIOD  700//unit : ms


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
void init_send_out_info(){
    send_out_info_Timers=xTimerCreate("send_out_info_Timers",( SEND_OUT_POLLING_PERIOD), pdTRUE, ( void * ) 1,  send_out_task);
    xTimerStart( send_out_info_Timers, 0 );
}

#define DECLARE_EPW_INFO(info_name, info_id, info_value) {.name=info_name, .id=info_id, .value=info_value}
void send_out_task(void){
    static cmd_group EPW_cmd_group = OUT_EPW_CMD;
    static int i;
    static unsigned char distance_cm[4]={0};
    static unsigned char motor_L_rpm=0, motor_R_rpm=0;

    static unsigned char encoder_left_count_sum=0, encoder_right_count_sum=0;
    static unsigned char KP_left=0, KI_left=0, KD_left=0;
    static unsigned char KP_right=0, KI_right=0, KD_right=0;
    static unsigned char pwm_left_value_display=0, pwm_right_value_display=0;
    
    static EPW_info  EPW_info_list[] = {                                         
        DECLARE_EPW_INFO("ultrasonic 0" , EPW_ULTRASONIC_0 , distance_cm),
        DECLARE_EPW_INFO("ultrasonic 1" , EPW_ULTRASONIC_1 , distance_cm+1),
        DECLARE_EPW_INFO("ultrasonic 2" , EPW_ULTRASONIC_2 , distance_cm+2),
        DECLARE_EPW_INFO("ultrasonic 3" , EPW_ULTRASONIC_3 , distance_cm+3),
        DECLARE_EPW_INFO("EPW_LEFT_RPM" , EPW_LEFT_RPM , &motor_L_rpm),
        DECLARE_EPW_INFO("EPW_RIGHT_RPM", EPW_RIGHT_RPM ,&motor_R_rpm),
        DECLARE_EPW_INFO("EPW_ENCODER_LEFT_COUNT", EPW_ENCODER_LEFT_COUNT ,&encoder_left_count_sum),
        DECLARE_EPW_INFO("EPW_ENCODER_RIGHT_COUNT", EPW_ENCODER_RIGHT_COUNT ,&encoder_right_count_sum),
        DECLARE_EPW_INFO("EPW_PID_KP_LEFT", EPW_PID_KP_LEFT ,&KP_left),
        DECLARE_EPW_INFO("EPW_PID_KI_LEFT", EPW_PID_KI_LEFT ,&KI_left),
        DECLARE_EPW_INFO("EPW_PID_KD_LEFT", EPW_PID_KD_LEFT ,&KD_left),
        DECLARE_EPW_INFO("EPW_PID_KP_RIGHT", EPW_PID_KP_RIGHT ,&KP_right),
        DECLARE_EPW_INFO("EPW_PID_KI_RIGHT", EPW_PID_KI_RIGHT ,&KI_right),
        DECLARE_EPW_INFO("EPW_PID_KD_RIGHT", EPW_PID_KD_RIGHT ,&KD_right),
        DECLARE_EPW_INFO("EPW_PID_KI_RIGHT", EPW_PWM_LEFT_VALUE ,&pwm_left_value_display),
        DECLARE_EPW_INFO("EPW_PID_KD_RIGHT", EPW_PWM_RIGHT_VALUE ,&pwm_right_value_display)
    };
        /*update the new value*/
        distance_cm[0]=(unsigned char)Get_CH1Distance();
        distance_cm[1]=(unsigned char)Get_CH2Distance();
        distance_cm[2]=(unsigned char)Get_CH3Distance();
        distance_cm[3]=(unsigned char)Get_CH4Distance();
        motor_L_rpm=(unsigned char)rpm_left_motor;
        motor_R_rpm=(unsigned char)rpm_right_motor;
        encoder_left_count_sum=(unsigned char)encoder_left_counter_sum;
        encoder_right_count_sum=(unsigned char)encoder_right_counter_sum;
        KP_left= (int)(Kp_left*10.0f);
        KI_left= (int)(Ki_left*10.0f);
        KD_left= (int)(Kd_left*10.0f);
        KP_right=(int)(Kp_right*10.0f);
        KI_right=(int)(Ki_right*10.0f);
        KD_right=(int)(Kd_right*10.0f);
        pwm_left_value_display=pwm_value_left_mcu;
        pwm_right_value_display=pwm_value_right_mcu;

        for(i=0;i<sizeof(EPW_info_list)/sizeof(EPW_info);i++){
            printf("cmd%c%c%cend",EPW_cmd_group,EPW_info_list[i].id, *(EPW_info_list[i].value));
        }
        fflush(0);/*force clear buffer and send out the other device. */
}

