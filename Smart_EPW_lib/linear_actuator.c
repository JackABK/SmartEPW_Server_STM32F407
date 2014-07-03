#include "linear_actuator.h"
#include "stm32f4xx.h"                                                                                  
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

typedef enum{
    ACTUATOR_GROUP_A,
    ACTUATOR_GROUP_B
}actuator_group;

typedef enum{
		ACTUATOR_STATE_IDLE,
		ACTUATOR_STATE_MOVE_CW,                               
		ACTUATOR_STATE_MOVE_CCW
}actuator_state_t;
static actuator_state_t actuator_state_A, actuator_state_B; 

/*Timer handle declare for detect the LS state*/
xTimerHandle detect_LS_Timers;

#define DETECT_LS_POLLING_PERIOD  50//unit : ms



static void init_PWM(){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO B clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		GPIO_InitStruct.GPIO_Pin =  ACTU_A_PWM_PIN|ACTU_B_PWM_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( ACTU_PWM_PORT, &GPIO_InitStruct );   

		/*====================TIM Setting=============================*/

		GPIO_PinAFConfig(ACTU_PWM_PORT, GPIO_PinSource4, GPIO_AF_TIM3); 
		GPIO_PinAFConfig(ACTU_PWM_PORT, GPIO_PinSource5, GPIO_AF_TIM3); 

		/* TIM3 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		/**
		 * Compute the prescaler value
		 * old version , 84MHz / 2000 / 42  = 1KHz
		 * --> u32 PrescalerValue = 42 - 1; 
		 * --> u32 TimPeriod = 2000 - 1;
		 *
		 * new version is setting for pwm 8 bit resolution , 84MHz / 256 / 250  ~= 1312.5 Hz
		 */   
		u32 PrescalerValue = 250 - 1; /*YinChen added*/
		u32 TimPeriod = 256 - 1;

		/* Time base configuration */
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = TimPeriod;     
		TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue ; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0 ;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/*====================PWM Setting=============================*/
		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0; /*max pwm value is TIM's period, in our case, it's  255*/
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel1   (ACTU_A_PWM_PIN)*/
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel2   (ACTU_B_PWM_PIN)*/
		TIM_OC2Init(TIM3, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_Cmd(TIM3, ENABLE);

}
 
static void init_LS(void)
{
        GPIO_InitTypeDef GPIO_InitStruct;
        /* Enable GPIO D clock. */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        GPIO_InitStruct.GPIO_Pin =  LS_A_UPPER_PIN | LS_A_LOWER_PIN | LS_B_UPPER_PIN | LS_B_LOWER_PIN; 

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_Init( LS_READ_PORT, &GPIO_InitStruct ); 
}


static void init_CWCCW(){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO E clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		GPIO_InitStruct.GPIO_Pin =  ACTU_A_IN1_PIN| ACTU_A_IN2_PIN | ACTU_B_IN3_PIN | ACTU_B_IN4_PIN ;
	
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( ACTU_CWCCW_PORT, &GPIO_InitStruct ); 
		
		GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_RESET);
		GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_RESET);
		GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_RESET);
		GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_RESET);	
}

void init_linear_actuator(){
		init_PWM();
		init_CWCCW();
		init_LS();

        detect_LS_Timers=xTimerCreate("Detect limit Switch state Polling",( DETECT_LS_POLLING_PERIOD), pdTRUE, ( void * ) 1,  detect_LS_Polling);
		xTimerStart( detect_LS_Timers, 0 );
}

void set_linearActuator_A_cmd(int flag , int pwm_value){
     if(flag==LINEAR_ACTU_STOP){
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_RESET);/* 0 */
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_RESET);/* 0 */
         TIM_SetCompare1(TIM3 , 0);
         actuator_state_A = ACTUATOR_STATE_IDLE;
     }else if(flag==LINEAR_ACTU_CW && get_Linear_Actuator_A_LS_State()!=0x01 && get_Linear_Actuator_A_LS_State()!=0x03){ /*if not upper limited.*/
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_RESET);/* 0 */
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_SET);/* 1 */
         TIM_SetCompare1(TIM3 , 255);
         actuator_state_A = ACTUATOR_STATE_MOVE_CW;
     }else if(flag==LINEAR_ACTU_CCW && get_Linear_Actuator_A_LS_State()!=0x02 && get_Linear_Actuator_A_LS_State()!=0x03){ /*if not lower limited*/
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_SET);/* 1 */
    	 GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_RESET);/* 0 */ 
         TIM_SetCompare1(TIM3 , 255);         
         actuator_state_A = ACTUATOR_STATE_MOVE_CCW;
     }
}
void set_linearActuator_B_cmd(int flag , int pwm_value){
     if(flag==LINEAR_ACTU_STOP){
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_RESET);/* 0 */
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_RESET);/* 0 */
         TIM_SetCompare2(TIM3 , 0);         
         actuator_state_B = ACTUATOR_STATE_IDLE;
     }else if(flag==LINEAR_ACTU_CW && get_Linear_Actuator_B_LS_State()!=0x01 && get_Linear_Actuator_B_LS_State()!=0x03){ /*if not upper limited.*/
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_RESET);/* 0 */
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_SET);/* 1 */                        
         TIM_SetCompare2(TIM3 , 255);
         actuator_state_B = ACTUATOR_STATE_MOVE_CW;
     }else if(flag==LINEAR_ACTU_CCW && get_Linear_Actuator_B_LS_State()!=0x02 && get_Linear_Actuator_B_LS_State()!=0x03){ /*if not lower limited*/
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_SET);/* 1 */
         GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_RESET);/* 0 */ 
         TIM_SetCompare2(TIM3 , 150);         
         actuator_state_B = ACTUATOR_STATE_MOVE_CCW;
     }
}

static void detect_LS_Polling(){    
    /*detect actuator A LS*/
    switch(actuator_state_A){
        case ACTUATOR_STATE_IDLE:
           break;//wait the next state into.
        case ACTUATOR_STATE_MOVE_CW:
           if(get_Linear_Actuator_A_LS_State()==0x01 || get_Linear_Actuator_A_LS_State()==0x03){
               set_linearActuator_A_cmd(LINEAR_ACTU_STOP,0);
               actuator_state_B=ACTUATOR_STATE_IDLE;
           }
           break;
        case ACTUATOR_STATE_MOVE_CCW:
            if(get_Linear_Actuator_A_LS_State()==0x02 || get_Linear_Actuator_A_LS_State()==0x03){
               set_linearActuator_A_cmd(LINEAR_ACTU_STOP,0);
               actuator_state_A=ACTUATOR_STATE_IDLE;
           }
        default:
           break;
    }
    
    /*detect actuator B LS*/
    switch(actuator_state_B){
       case ACTUATOR_STATE_IDLE:
           break;//wait the next command into.
       case ACTUATOR_STATE_MOVE_CW:
           if(get_Linear_Actuator_B_LS_State()==0x01 || get_Linear_Actuator_B_LS_State()==0x03){
               set_linearActuator_B_cmd(LINEAR_ACTU_STOP,0);
               actuator_state_B=ACTUATOR_STATE_IDLE;
           }
           break;
       case ACTUATOR_STATE_MOVE_CCW:
           if(get_Linear_Actuator_B_LS_State()==0x02 || get_Linear_Actuator_B_LS_State()==0x03){
               set_linearActuator_B_cmd(LINEAR_ACTU_STOP,0);
               actuator_state_B=ACTUATOR_STATE_IDLE;
           }
           break;
       default: 
           break;
   }
}

int get_Linear_Actuator_A_LS_State(void){
    static int actuator_A_LS_state;
    actuator_A_LS_state  = GPIO_ReadInputDataBit(LS_READ_PORT, LS_A_UPPER_PIN)? 0x01 : 0x00; /*upper limited*/
    actuator_A_LS_state |= GPIO_ReadInputDataBit(LS_READ_PORT, LS_A_LOWER_PIN)? 0x02 : 0x00; /*lower limited*/
    /*
     * 00 :  normal range.
     * 01 :  upper limited
     * 02 :  lower limited
     * 03 :  both limited, but isn't impossible in actually.
     */ 
    return actuator_A_LS_state;
}
int get_Linear_Actuator_B_LS_State(void){
    static int actuator_B_LS_state;
    actuator_B_LS_state  = GPIO_ReadInputDataBit(LS_READ_PORT, LS_B_UPPER_PIN)? 0x01 : 0x00; /*upper limited*/
    actuator_B_LS_state |= GPIO_ReadInputDataBit(LS_READ_PORT, LS_B_LOWER_PIN)? 0x02 : 0x00; /*lower limited*/
    /*
     * 00 :  normal range.
     * 01 :  upper limited
     * 02 :  lower limited
     * 03 :  both limited, but isn't impossible in actually.
     */ 
    return actuator_B_LS_state;
}


