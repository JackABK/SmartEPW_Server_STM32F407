#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "ultrasound.h"
#include "timers.h"
#include "uart.h"
#include "clib.h"
#include "EPW_command.h"
#include "PID.h"



#define CAR_POLLING_PERIOD  20//unit : ms
#define PID_POLLING_PERIOD  20//unit : ms






typedef enum{
		CAR_STATE_IDLE,
		CAR_STATE_REST,                               
		CAR_STATE_MOVE_FORWARD,
		CAR_STATE_MOVE_BACK,
		CAR_STATE_MOVE_LEFT,
		CAR_STATE_MOVE_RIGHT
}car_state_t;

static car_state_t car_state;


/*create the pid struct for use*/
pid_struct PID_Motor_L;
pid_struct PID_Motor_R;



/*encoder_left  variable.  setting*/
 float rpm_left_motor = 0.0f;
static int encoder_left_counter;
/*encoder_right  variable  setting*/
 float rpm_right_motor = 0.0f;
static int encoder_right_counter;

static float set_rpm=300.0f;

/*pwm regulate of two motor */
static int SpeedValue_left; /*default speedvalue*/
static int SpeedValue_right;
static int motor_pwm_value=50; /*global pwm value.*/
static char flag;

/*pid alg premeter.*/
static float Kp,Ki,Kd;

/*Timer handle declare*/
xTimerHandle carTimers;
xTimerHandle PID_Timers;


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
		GPIO_Init( MOTOR_PWM_PORT, &GPIO_InitStruct );   

		/*====================TIM Setting=============================*/
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource15, GPIO_AF_TIM4);

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
		TIM_OCInitStructure.TIM_Pulse = 0; /*max pwm value is TIM's period, in our case, it's  255*/
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel2   (MOTOR_LEFT_PWM_PIN)*/
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel4   (MOTOR_RIGHT_PWM_PIN)*/
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_Cmd(TIM4, ENABLE);
}


void init_motor_CWCCW(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO D clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        #ifdef L298N_MODE
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_IN1_PIN| MOTOR_LEFT_IN2_PIN | MOTOR_RIGHT_IN3_PIN | MOTOR_RIGHT_IN4_PIN ;
        #else /*Smart EPW*/
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_CWCCW_PIN | MOTOR_RIGHT_CWCCW_PIN;
        #endif
	
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( MOTOR_CWCCW_PORT, &GPIO_InitStruct ); 
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_LEFT_CWCCW_PIN,Bit_RESET);
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_RIGHT_CWCCW_PIN,Bit_RESET);	
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
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
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
        init_motor_CWCCW();
        init_encoder();
		init_External_Interrupt();
		ultra_sound_init();

		carTimers=xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
		xTimerStart( carTimers, 0 );


		PID_Timers=xTimerCreate("PID_Algorithm_Polling",( PID_POLLING_PERIOD), pdTRUE, ( void * ) 1,  PID_Algorithm_Polling);
		xTimerStart( PID_Timers, 0 );

        /*Initialization the right motor of pid paremeter.*/
        Kp=3.0f; Ki=2.0f; Kd=0.0f;
        InitPID(&PID_Motor_R , Kp ,Ki,Kd);
}



/*============================================================================*
 ** Prototype    : Car_State_Polling
 ** Description  : record the car's state of all behavior and keep to polling,
                   this is used for auto-avoidance polling mechanism
 ** Input        : None
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
#define THRESHOLD_DISTANCE 50 /*unit : cm*/
/**
 *  the unit is  (CAR_POLLING_PERIOD ms) , so depend on the CAR_POLLING_PERIOD.
 *  for example the CAR_POLLING_PERIOD=20, CAR_MOVING_PERIOD=10,
 *  then the car moving will be keep 200(10*20) ms time.
 */
#define CAR_MOVING_PERIOD 250 
#define CAR_REST_PERIOD  5
void Car_State_Polling(){
		static int count=0;
        unsigned int distance[4];
        distance[0]=Get_CH1Distance();
        distance[1]=Get_CH2Distance();
        //distance[2]=Get_CH3Distance();
        //distance[3]=Get_CH4Distance();
        
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

#if 0       
                if(distance[0]==0 || distance[1]==0){
                    car_state=CAR_STATE_IDLE;
                    return;
                }
                state|=((distance[0]>THRESHOLD_DISTANCE)?0x01:0x00);
                state|=((distance[1]>THRESHOLD_DISTANCE)?0x02:0x00);
                switch(state){
                    case 0x00:
                        //deadend
                        
                        break;
                    case 0x01:
                        //need to turn left
                        //proc_cmd("left" , SpeedValue_left , SpeedValue_right);

                        /*send warning message*/
                        //printf("Left %d %d" , distance[0] , distance[1]);
                        break;
                    case 0x02:
                        //need to turn right
                        //proc_cmd("right" , SpeedValue_left , SpeedValue_right);

                        /*send warning message*/
                        //printf("Right %d %d" , distance[0] , distance[1]);
                        break;
                    case 0x03:
                        //nothing ahead
                        break;
                }
#endif       
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
        else if(car_state==CAR_STATE_MOVE_LEFT){
                count++;
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
				}
		}   
        else if(car_state==CAR_STATE_MOVE_RIGHT){
                count++;
				if(count>=CAR_MOVING_PERIOD){
						count=0;
						car_state=CAR_STATE_REST;
    		    }
		}    
}

/*============================================================================*/
/*============================================================================*
 ** function : parse_EPW_motor_dir
 ** brief : parse the EPW of motor direction from the uart siganl,
            note, the motor_pwm_value  max is 255, even if 256, it's duty cycle is equal to 255.
 ** param : DIR_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/
void parse_EPW_motor_dir(unsigned char DIR_cmd)
{
       
		if(DIR_cmd == 'f'){
				car_state = CAR_STATE_MOVE_FORWARD;
				encoder_left_counter=0;
				encoder_right_counter=0;
				SpeedValue_left = (int)motor_pwm_value; 
				SpeedValue_right=SpeedValue_left;

				proc_cmd("forward" , SpeedValue_left , SpeedValue_right);
		}
		else if(DIR_cmd == 's'){
				car_state = CAR_STATE_IDLE;
				encoder_left_counter=0;
				encoder_right_counter=0;

				/*even stop function is  always zero , but for security, I also set speedvalue to zero.*/
				SpeedValue_left = 0;
				SpeedValue_right = 0;
				proc_cmd("stop" , SpeedValue_left , SpeedValue_right);

		}
		else if(DIR_cmd == 'b'){
                car_state = CAR_STATE_MOVE_BACK;
				encoder_left_counter=0;
				encoder_right_counter=0;
				SpeedValue_left = (int)motor_pwm_value ; 
				SpeedValue_right=SpeedValue_left;
				proc_cmd("backward" , SpeedValue_left , SpeedValue_right);
		}
        else if(DIR_cmd == 'l'){
                car_state = CAR_STATE_MOVE_LEFT;
				encoder_left_counter=0;
				encoder_right_counter=0;
				SpeedValue_left = (int)motor_pwm_value ; 
				SpeedValue_right=SpeedValue_left;
				proc_cmd("left" , SpeedValue_left , SpeedValue_right);
		}
        else if(DIR_cmd == 'r'){
                car_state = CAR_STATE_MOVE_RIGHT;
				encoder_left_counter=0;
				encoder_right_counter=0;
				SpeedValue_left = (int)motor_pwm_value ; 
				SpeedValue_right=SpeedValue_left;
				proc_cmd("right" , SpeedValue_left , SpeedValue_right);
		}
		else{
				/*do not anything*/
		}
}

void PerformCommand(unsigned char group,unsigned char control_id, unsigned char value)
{
   if(group == OUT_EPW_CMD){ /*0*/
		switch ( control_id )
		{
		    case EPW_MOTOR_DIR:
		        parse_EPW_motor_dir(value);
		        break;
		    case EPW_MOTOR_PWM:
		        motor_pwm_value = value; /*0~255*/
		        break;
		    case EPW_ACTUATOR_A :
		        set_linearActuator_A_cmd(value , 255); /*the actuator of pwm_value is fixed, value is dir flag.*/
		        break;
		    case EPW_ACTUATOR_B :                
		        set_linearActuator_B_cmd(value , 255); /*the actuator of pwm_value is fixed. value is dir flag.*/
		        break;
		    case EPW_PID_ALG_KP :
                 Kp = (float)value;
		         InitPID(&PID_Motor_R , Kp/10.0f ,Ki,Kd);
		        break;
		    case EPW_PID_ALG_KI :
                 Ki = (float)value;
		         InitPID(&PID_Motor_R , Kp, Ki/10.0f, Kd);
		        break;
            case EPW_PID_ALG_KD :
                 Kd = (float)value;
		         InitPID(&PID_Motor_R , Kp, Ki, Kd/10.0f);
		        break;
		    default:
		        ;
		}
   }
}



/*============================================================================*/
/*============================================================================*
 ** function : PID_Algorithm_Polling
 ** brief : loop read the encoder signal  and caclulator the RPM , the cycle time is 20msec
 ** param : None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
#define OUTPUT_INFO_PERIOD 25 /*unit : PID_POLLING_PERIOD ms*/
void PID_Algorithm_Polling(void)
{

		detachInterrupt(EXTI_Line0); /*close external interrupt 0*/ 
		detachInterrupt(EXTI_Line1); /*close external interrupt 1*/ 
        float temp ;
        static int output_info_count = 0 ;
        /*get the two motor parameter such as RPM, Rotations..*/
        getMotorData();

        /*calculate Position PID of two motor*/    
        SpeedValue_right = (int)PID_Pos_Calc(&PID_Motor_R , rpm_left_motor , rpm_right_motor);



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

void getMotorData(void)
{
    
    /*for L298N test is used by 600 pulse/rev of the encoder.*/
#ifdef L298N_MODE 
    rpm_left_motor=(float)encoder_left_counter * 60.0f /600.0f / 0.02f;
    rpm_right_motor=(float)encoder_right_counter *  60.0f  /600.0f / 0.02f;    
#else
    /*for SmartEPW is used by 500 pulse/rev */
    rpm_left_motor=(float)encoder_left_counter * 60.0f /500.0f / 0.02f;
    rpm_right_motor=(float)encoder_right_counter *  60.0f  /500.0f / 0.02f;          
    //get the speed of the two motors in deg/sec
    /***do someting***/ 
#endif
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











