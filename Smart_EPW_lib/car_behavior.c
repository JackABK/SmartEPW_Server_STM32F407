#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "car_behavior.h"
#include "ultrasound.h"
#include "timers.h"
#include "uart.h"
#include "clib.h"
#include "car_command.h"


#define CAR_POLLING_PERIOD  20//unit : ms
                                    

/*=================Re-define the all by pins=========================*/
             /****Motor****/            
#define MOTOR_PORT                                                GPIOD
#define MOTOR_LEFT_PWM_PIN                                        GPIO_Pin_13
#define MOTOR_RIGHT_PWM_PIN                                       GPIO_Pin_15

                /****Encoder****/
#define ENCODER_PORT                                              GPIOA
#define ENCODER_LEFT_PHASE_A_PIN                                  GPIO_Pin_0      /*the inturrupt is maping to EXTI0*/
#define ENCODER_RIGHT_PHASE_A_PIN                                 GPIO_Pin_1      /*the inturrupt is maping to EXTI1*/
#define ENCODER_LEFT_PHASE_B_PIN                                  GPIO_Pin_2
#define ENCODER_RIGHT_PHASE_B_PIN                                 GPIO_Pin_3
/*===============end of define  the all by pins========================*/


typedef enum{
    CAR_STATE_IDLE,
    CAR_STATE_REST,                               
    CAR_STATE_MOVE_FORWARD,
    CAR_STATE_MOVE_BACK    
}car_state_t;

static car_state_t car_state;




/*encoder_left  variable.  setting*/
static float rpm_left_motor = 0;
static int encoder_left_counter;
/*encoder_right  variable  setting*/
static float rpm_right_motor = 0;
static int encoder_right_counter;

static float rpm_error ;


/*pwm regulate of two motor */
static int SpeedValue_left ; /*default speedvalue*/
static int SpeedValue_right ;
static char flag;

/*Timer handle declare*/
xTimerHandle carTimers;
xTimerHandle PD_Timers;


/*============================================================================*/
/*============================================================================*
 ** function : init_motor
 ** brief :  initialize the motor control pin set, the AF is  setting by pwm mode based on timer.
 ** param :  None
 ** retval : None
 **============================================================================*/
 /*============================================================================*/
void init_motor(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/* Enable GPIO A clock. */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_PWM_PIN|MOTOR_RIGHT_PWM_PIN;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init( MOTOR_PORT, &GPIO_InitStruct );   

        /*====================TIM Setting=============================*/
        GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource13, GPIO_AF_TIM4);
        GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource15, GPIO_AF_TIM4);
        
        /* TIM4 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

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
            TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

        /*====================PWM Setting=============================*/
        TIM_OCInitTypeDef TIM_OCInitStructure;
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        /* PWM1 Mode configuration: Channel2   (MOTOR_LEFT_PWM_PIN)*/
        TIM_OC2Init(TIM4, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
        /* PWM1 Mode configuration: Channel4   (MOTOR_RIGHT_PWM_PIN)*/
        TIM_OC4Init(TIM4, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
        
        TIM_Cmd(TIM4, ENABLE);
}


/*============================================================================*/
/*============================================================================*
 ** function : init_encoder
 ** brief : initialization the encoder , GPIO is setting to input.
 ** param : None
 ** retval : None
 **============================================================================*/
 /*============================================================================*/
void init_encoder(void)
{
         GPIO_InitTypeDef GPIO_InitStruct;
        /* Enable GPIO A clock. */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        GPIO_InitStruct.GPIO_Pin =       ENCODER_LEFT_PHASE_A_PIN \
                                                           |   ENCODER_RIGHT_PHASE_A_PIN  \ 
                                                           |   ENCODER_LEFT_PHASE_B_PIN \
                                                           |  ENCODER_RIGHT_PHASE_B_PIN ; 
        
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init( ENCODER_PORT, &GPIO_InitStruct ); 
}

/*============================================================================*/
/*============================================================================*
 ** function : init_External_Interrupt
 ** brief : connect the two encoder phase A to interrupt
 ** param : None
 ** retval : None
 **============================================================================*/
 /*============================================================================*/
void init_External_Interrupt(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	

    /* Connect EXTI Line0 to PA0 pin */
       SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
       EXTI_InitStruct.EXTI_Line = EXTI_Line0;
       EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
       EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
       EXTI_InitStruct.EXTI_LineCmd = ENABLE;
       EXTI_Init(&EXTI_InitStruct);
       EXTI_ClearITPendingBit(EXTI_Line0);
       NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);


   
   /* Connect EXTI Line1 to PA1 pin */
       SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
       EXTI_InitStruct.EXTI_Line = EXTI_Line1;
       EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
       EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
       EXTI_InitStruct.EXTI_LineCmd = ENABLE;
       EXTI_Init(&EXTI_InitStruct);
       EXTI_ClearITPendingBit(EXTI_Line1);
       NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
       NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
       NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
       NVIC_Init(&NVIC_InitStructure);
}


void init_car(){
	  init_motor();
           init_External_Interrupt();
	  ultra_sound_init();
   
	carTimers=xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
	xTimerStart( carTimers, 0 );

        PD_Timers=xTimerCreate("PI_Algorithm_Polling",( 1000), pdTRUE, ( void * ) 1,  PI_Algorithm_Polling	);
	xTimerStart( PD_Timers, 0 );
}



/*============================================================================*
  ** Prototype    : Car_State_Polling
  ** Description  : record the car's state of all behavior and keep to polling
  ** Input        : None
  ** Output       : None
  ** Return Value : 
  *============================================================================*/
  #define THRESHOLD_DISTANCE 100 /*unit : cm*/
/**
  *  the unit is  (CAR_POLLING_PERIOD ms) , so depend on the CAR_POLLING_PERIOD.
  *  for example the CAR_POLLING_PERIOD=20, CAR_MOVING_PERIOD=10,
  *  then the car moving will be keep 200(10*20) ms time.
  */
#define CAR_MOVING_PERIOD 10 
#define CAR_REST_PERIOD  5
void Car_State_Polling(){
	static int count;
	if(car_state==CAR_STATE_IDLE){
		count=0;
	}
	else if(car_state==CAR_STATE_REST){
		proc_cmd("stop" , 0 , 0);
		count++;
		if(count>=CAR_REST_PERIOD){
			count=0;
			car_state=CAR_STATE_IDLE;
		}
	}
	else if(car_state==CAR_STATE_MOVE_FORWARD){
		count++;
		if(count>=CAR_MOVING_PERIOD){
			count=0;
			car_state=CAR_STATE_REST;
		}
			
	}	
	else if(car_state==CAR_STATE_MOVE_BACK){
		count++;
		if(count>=CAR_MOVING_PERIOD){
			count=0;
			car_state=CAR_STATE_REST;
		}
			
	}    
    
}

/*============================================================================*/
/*============================================================================*
 ** function : PerformCommand
 ** brief : parse the command from the uart siganl
 ** param : acpt_cmd
 ** retval :  None
 **============================================================================*/
 /*============================================================================*/
void PerformCommand(unsigned char acpt_cmd , unsigned char acpt_pwm_value)
{
        if(acpt_cmd == 'f'){
                encoder_left_counter=0;
                encoder_right_counter=0;
                SpeedValue_left = (int)acpt_pwm_value; 
                SpeedValue_right=SpeedValue_left;
                
                proc_cmd("forward" , SpeedValue_left , SpeedValue_right);
                car_state = CAR_STATE_MOVE_FORWARD;
        }
        else if(acpt_cmd == 's'){
                 encoder_left_counter=0;
                 encoder_right_counter=0;

                /*even stop function is  always zero , but for security, I also set speedvalue to zero.*/
                SpeedValue_left = 0;
                SpeedValue_right = 0;
                proc_cmd("stop" , SpeedValue_left , SpeedValue_right);
                car_state = CAR_STATE_IDLE;
                
        }
        else if(acpt_cmd == 'b'){
                encoder_left_counter=0;
                encoder_right_counter=0;
                SpeedValue_left = (int)acpt_pwm_value ; 
                SpeedValue_right=SpeedValue_left;
                proc_cmd("backward" , SpeedValue_left , SpeedValue_right);
                car_state = CAR_STATE_MOVE_BACK;
        }
        else{
                /*do not anything*/
        }
}






/*============================================================================*/
/*============================================================================*
 ** function : PI_Algorithm_Polling
 ** brief : loop read the encoder signal  and caclulator the RPM , the cycle time is 20msec
 ** param : None
 ** retval : None
 **============================================================================*/
 /*============================================================================*/
void PI_Algorithm_Polling(void)
{
   
                    detachInterrupt(EXTI_Line0); /*close external interrupt 0*/ 
                    detachInterrupt(EXTI_Line1); /*close external interrupt 1*/ 

                    /*use the count total by  1 sec to caclulator RPM */
                    /*the encoder spec is  500 pulse/1 rev */
                    rpm_left_motor=(float)encoder_left_counter * 60 /500 / 1;
                    rpm_right_motor=(float)encoder_right_counter *  60  /500 / 1;
                    /*calculator the calibration of right motor based on left motor */

                    rpm_error =   (rpm_left_motor - rpm_right_motor) * 4 /10  ;
                    SpeedValue_right=   SpeedValue_right +   (int)rpm_error;

                    /*print the message to stdout*/
                    
                    //printf(" %d\n" ,(int)(SpeedValue_right));
                    //printf("encoder_left_counter : %d\r\n" , encoder_left_counter  );
                    //printf("encoder_right_counter : %d\r\n" , encoder_right_counter  );
                    
                   //printf("%d\r\n" ,  rpm_left_motor );
                   //printf("%d\r\n" ,  rpm_right_motor );

                    /*test pirntf*/

                    /*
                    float myfloat = 120.0f;
                    myfloat = myfloat * 60 /500;
                    int myint;
                    */
                    
                   // myint =  round(myfloat)  ;
                    
                   //printf("%d\n\r" , (int)rpm_error);
                    
                   //printf("rpm_left = %.2f \n ", rpm_left_motor);
                   //printf("rpm_right = %.2f \n ", rpm_right_motor);
                 
                   
                    /*restart motor calibration and re-count encoder count*/
                    if(car_state == CAR_STATE_MOVE_FORWARD){
                                proc_cmd("forward" , SpeedValue_left , SpeedValue_right);
                    }
                    else if(car_state == CAR_STATE_MOVE_BACK) {
                                proc_cmd("backward" , SpeedValue_left , SpeedValue_right);
                    }

                    encoder_left_counter = 0;   
                    encoder_right_counter = 0;   
                   
                    /*restart enable the external interrupt*/
                    attachInterrupt(EXTI_Line0); 
                    attachInterrupt(EXTI_Line1);
        
}

void EXTI0_IRQHandler(){

    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        encoder_left_counter++ ;
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void EXTI1_IRQHandler(){
      if(EXTI_GetITStatus(EXTI_Line1) != RESET)
     {
        encoder_right_counter++ ;
        EXTI_ClearITPendingBit(EXTI_Line1);
      }
}


void detachInterrupt(uint32_t EXTI_LineX){
      EXTI_InitTypeDef EXTI_InitStruct;
      EXTI_InitStruct.EXTI_Line = EXTI_LineX;
      EXTI_InitStruct.EXTI_LineCmd = DISABLE;
}

void attachInterrupt(uint32_t EXTI_LineX){
      EXTI_InitTypeDef EXTI_InitStruct;
      EXTI_InitStruct.EXTI_Line = EXTI_LineX;
      EXTI_InitStruct.EXTI_LineCmd = ENABLE;
}











