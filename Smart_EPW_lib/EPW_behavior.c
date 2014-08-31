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
#include "linear_actuator.h"

#include "fat_filelib.h"
#include "stm32f4_discovery_sdio_sd.h"
#include "example.h"



#define CAR_POLLING_PERIOD  20//unit : ms
#define PID_POLLING_PERIOD  20//unit : ms


#define MOTOR_CW 0
#define MOTOR_CCW 1




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
int encoder_left_counter;
int encoder_left_counter_sum=0;
/*encoder_right  variable  setting*/
float rpm_right_motor = 0.0f;
int encoder_right_counter;
int encoder_right_counter_sum=0;
static float set_rpm=68.4f;
static float set_encoder_count=8.0f;


/*pwm regulate of two motor */
int pwm_value_left=120; /*default pwm value*/
int pwm_value_right=120;
static int pwm_value_left_err=0, pwm_value_right_err=0;
static int global_specified_distance_value=0; /*global speed value, range is 0~10.

/*for the backward of motro driver.*/
int pwm_value_left_pid=0, pwm_value_right_pid=0;
int pwm_value_left_mcu=0, pwm_value_right_mcu=0;


/*for the pwm compensate value by pid alg.*/
static int set_fixed_distance_encoder_value=107;
static int pwm_left_compensate_value_forward=35;
static int pwm_right_compensate_value_forward=40;

int pwm_left_compensate_value_back=20;
int pwm_right_compensate_value_back=23;



static char flag;
static int flag_PID_L_R=0;  /*L -> 1,  R -> 2*/
/*pid alg premeter.*/
float Kp_left,Ki_left,Kd_left;
float Kp_right, Ki_right, Kd_right;
/*Timer handle declare*/
xTimerHandle carTimers;
xTimerHandle PID_Timers;


/*check whether complete the goals of specified distance.*/
static int complete_specified_dist_goals=0;
FL_FILE *file;

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
										 |
										 ENCODER_RIGHT_PHASE_B_PIN ; 

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


void init_EPW_control(){
        
		init_motor();
        init_motor_CWCCW();
        init_encoder();
		init_External_Interrupt();
        

		carTimers=xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
		xTimerStart( carTimers, 0 );


		PID_Timers=xTimerCreate("PID_Algorithm_Polling",( PID_POLLING_PERIOD), pdTRUE, ( void * ) 1,  PID_Algorithm_Polling);
		xTimerStart( PID_Timers, 0 );

        /*Initialization the right motor of pid paremeter.*/
        Kp_left=5.0f; Ki_left=0.5f; Kd_left=5.5f;
        Kp_right=5.0f; Ki_right=0.5f; Kd_right=5.5f;
        
        InitPID(&PID_Motor_L , Kp_left ,Ki_left,Kd_left);
        InitPID(&PID_Motor_R , Kp_right ,Ki_right,Kd_right);

        /*For motor driver, the default idle state pwm value need to set 2.5V, it should be stop.*/
        parse_EPW_motor_dir('s');    
      

#ifdef ACCESS_SDIO         
        /*sdio*/
        accsess_sdio_setup();
#endif
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
//#define CAR_MOVING_PERIOD 250 
#define CAR_MOVING_PERIOD 1500 
#define CAR_REST_PERIOD  5
void Car_State_Polling(){
		static int count=0;
        unsigned int distance[4];
        //distance[0]=Get_CH1Distance();
        //distance[1]=Get_CH2Distance();
        //distance[2]=Get_CH3Distance();
        //distance[3]=Get_CH4Distance();
        
		if(car_state==CAR_STATE_IDLE){
				count=0;
		}
		else if(car_state==CAR_STATE_REST){
                proc_cmd("stop" , MOTOR_IDLE_VOLTAGE , MOTOR_IDLE_VOLTAGE);
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
                        //proc_cmd("left" , pwm_value_left , pwm_value_right);

                        /*send warning message*/
                        //printf("Left %d %d" , distance[0] , distance[1]);
                        break;
                    case 0x02:
                        //need to turn right
                        //proc_cmd("right" , pwm_value_left , pwm_value_right);

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
                encoder_left_counter_sum=0;
                encoder_right_counter_sum=0;
                //specified_distance_convert_to_setRPM(MOTOR_CW, global_specified_distance_value); 
				pwm_value_left_mcu = 150;
				pwm_value_right_mcu=pwm_value_left_mcu;
                //PID_Motor_L.Kp = 2.1f; PID_Motor_L.Ki = 0.8f; PID_Motor_L.Kd=0.0f;
                //PID_Motor_R.Kp = 2.0f; PID_Motor_R.Ki = 0.8f; PID_Motor_R.Kd=0.0f;
				proc_cmd("forward" , pwm_value_left_mcu , pwm_value_right_mcu);
                
		}
		else if(DIR_cmd == 's'){
				car_state = CAR_STATE_REST;
				encoder_left_counter=0;
				encoder_right_counter=0;
                encoder_left_counter_sum=0;
                encoder_right_counter_sum=0;
				/*even stop function is  always zero , but for security, I also set speedvalue to zero.*/
				pwm_value_left = MOTOR_IDLE_VOLTAGE;
				pwm_value_right = MOTOR_IDLE_VOLTAGE;
				proc_cmd("stop" , pwm_value_left , pwm_value_right);

		}
		else if(DIR_cmd == 'b'){
                car_state = CAR_STATE_MOVE_BACK;
				encoder_left_counter=0;
				encoder_right_counter=0;
                encoder_left_counter_sum=0;
                encoder_right_counter_sum=0;
                //specified_distance_convert_to_setRPM(MOTOR_CCW, global_specified_distance_value); 
				pwm_value_left = 110;
				pwm_value_right=pwm_value_left;
                
                pwm_value_left_mcu=110;
                pwm_value_right_mcu=pwm_value_left_mcu;


                /*for the testing of fixed pid par.*/
                //PID_Motor_L.Kp = 2.1f; PID_Motor_L.Ki = 0.8f; PID_Motor_L.Kd=0.0f;
                //PID_Motor_R.Kp = 2.1f; PID_Motor_R.Ki = 0.8f; PID_Motor_R.Kd=0.0f;
				proc_cmd("backward" , pwm_value_left_mcu , pwm_value_right_mcu);
		}
        else if(DIR_cmd == 'l'){
             /*testing by PID calibration using.*/
            #if 0
                car_state = CAR_STATE_MOVE_LEFT;
				encoder_left_counter=0;
				encoder_right_counter=0;
                encoder_left_counter_sum=0;
                encoder_right_counter_sum=0;
				pwm_value_left = 100; 
				pwm_value_right= 150; 
				proc_cmd("left" , pwm_value_left , pwm_value_right);
            #endif
            flag_PID_L_R = 1;
		}
        else if(DIR_cmd == 'r'){
            /*testing by PID calibration using.*/
            #if 0
                car_state = CAR_STATE_MOVE_RIGHT;
				encoder_left_counter=0;
				encoder_right_counter=0;
                encoder_left_counter_sum=0;
                encoder_right_counter_sum=0;
				pwm_value_left = 150; 
				pwm_value_right=100;
				proc_cmd("right" , pwm_value_left , pwm_value_right);
            #endif
            flag_PID_L_R = 2;
		}
		else{
				/*do not anything*/
		}
}


/*============================================================================*/
/*============================================================================*
 ** function : specified_distance_convert_to_setRPM
 ** brief : 
 **         1 speed value convert to 8 pwm scale.
 ** param : motor_dir, specified_distance_value
 ** retval : Null
 **============================================================================*/
/*============================================================================*/
void specified_distance_convert_to_setRPM(int motor_dir, int specified_distance_value){
#if 0
    /* 100 * 60 * 0.006 *specified_distance_value /2 */
    set_rpm = 18 * specified_distance_value;
#endif 
    /*RPM for wheelchair --> (60/2)*0.6f*/
    //if(car_state == CAR_STATE_MOVE_FORWARD) set_rpm = 10.0f; //fixed the speed of per second.
    //else if(car_state == CAR_STATE_MOVE_BACK) set_rpm = 9.0f; //back should be slow more then forward.


    if(car_state == CAR_STATE_MOVE_FORWARD) set_encoder_count = 5.0f; //fixed the speed of per second.
    else if(car_state == CAR_STATE_MOVE_BACK) set_encoder_count = 5.0f; //back should be slow more then forward
}
void PerformCommand(unsigned char group,unsigned char control_id, unsigned char value)
{
   static int actuator_pwm_value=0;
   if(group == OUT_EPW_CMD){ /*0*/
		switch ( control_id )
		{
		    case EPW_MOTOR_DIR:
		        parse_EPW_motor_dir(value);
		        break;
		    case EPW_MOTOR_PWM:
		        global_specified_distance_value = value; /*0~10 scale*/
		        break;
		    case EPW_ACTUATOR_A :
                //if(value==LINEAR_ACTU_CW) actuator_pwm_value=255;
                //else if(value==LINEAR_ACTU_CCW) actuator_pwm_value=150;
		        set_linearActuator_A_cmd(value , actuator_pwm_value); /*the actuator of pwm_value is fixed, value is dir flag.*/
		        break;
		    case EPW_ACTUATOR_B :   
                //if(value==LINEAR_ACTU_CW) actuator_pwm_value=255;
                //else if(value==LINEAR_ACTU_CCW) actuator_pwm_value=150;
		        set_linearActuator_B_cmd(value , actuator_pwm_value); /*the actuator of pwm_value is fixed. value is dir flag.*/
		        break;
		    case EPW_PID_ALG_KP :
                 if(flag_PID_L_R==1){/*L*/
                    Kp_left = (float)value / 10.0f;
                    PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 }else if(flag_PID_L_R==2){/*R*/
                    Kp_right=(float)value / 10.0f;
                    PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 }
                 #if 0
                 Kp_left = (float)value / 10.0f;
                 //Kp_right= (  (float)value - 1.0f )/ 10.0f;
                 Kp_right=Kp_left;
                 PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 #endif
		        break;
		    case EPW_PID_ALG_KI :
                if(flag_PID_L_R==1){/*L*/
                    Ki_left = (float)value / 10.0f; 
                    PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 }else if(flag_PID_L_R==2){/*R*/
                    Ki_right=(float)value / 10.0f;
                    PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 }
                 #if 0
                 Ki_left = (float)value / 10.0f;  
                 //Ki_right= (  (float)value + 1.0f )/ 10.0f;
                 Ki_right=Ki_left;
                 PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 #endif
		        break;
            case EPW_PID_ALG_KD :
                if(flag_PID_L_R==1){/*L*/
                    Kd_left = (float)value / 10.0f; 
                    PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 }else if(flag_PID_L_R==2){/*R*/
                    Kd_right=(float)value / 10.0f;
                    PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 }
                 #if 0
                 Kd_left = (float)value / 10.0f;  
                 Kd_right= Kd_left;
		         PID_Motor_L.Kp = Kp_left; PID_Motor_L.Ki = Ki_left; PID_Motor_L.Kd=Kd_left;
                 PID_Motor_R.Kp = Kp_right; PID_Motor_R.Ki = Ki_right; PID_Motor_R.Kd=Kd_right;
                 #endif
		        break;
            case EPW_SET_ENCODER_COUNT:
		        set_encoder_count= (float)value; /*0~15 scale*/
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
        /*get the two motor parameter such as RPM, Rotations..*/
        getMotorData();

		/*restart motor calibration and re-count encoder count*/
		if(car_state == CAR_STATE_MOVE_FORWARD){
                if(encoder_left_counter_sum>= math_round((float)global_specified_distance_value*0.86f*500.0f) &&  encoder_right_counter_sum>=math_round((float)global_specified_distance_value*0.86f*500.0f)){
                    proc_cmd("stop" , MOTOR_IDLE_VOLTAGE , MOTOR_IDLE_VOLTAGE);
                    car_state = CAR_STATE_IDLE;
                    complete_specified_dist_goals=1;
                    encoder_left_counter_sum=0;
                    encoder_right_counter_sum=0; 
#ifdef ACCESS_SDIO         
                    access_data_to_sdio();
#endif
                }
                //else if(encoder_left_counter==0&&encoder_right_counter==0){
                //        ;//proc_cmd("stop" , MOTOR_IDLE_VOLTAGE , MOTOR_IDLE_VOLTAGE);//detected no feedback
                //}
                else{/*detected encoder data, start to PID alg.*/
#ifdef ACCESS_SDIO         
                        access_data_to_sdio();
#endif
                        /*calculate Position PID of two motor*/
                        pwm_value_left_pid = math_round(PID_Pos_Calc(&PID_Motor_L , set_encoder_count , encoder_left_counter));
                        pwm_value_right_pid = math_round(PID_Pos_Calc(&PID_Motor_R , set_encoder_count , encoder_right_counter));
                        /** 
                         * the motor driver of pwm value accept CW is 127~200 for security
                         * the motor driver of pwm value accept CCW is 50~127 for security
                         **/

                        pwm_value_left_mcu = pwm_value_left_pid;
                        pwm_value_right_mcu = pwm_value_right_pid;


                        if(pwm_value_left_mcu <=127){
                            pwm_value_left_mcu = 127;
                        }else if (pwm_value_left_mcu >=190){
                        #if 0
                            if(encoder_left_counter_sum<=set_fixed_distance_encoder_value)
                                pwm_value_left_mcu=190-pwm_left_compensate_value_forward;
                            else
                                pwm_value_left_mcu = 190;
                        #endif
                            pwm_value_left_mcu = 190;
                        }
                        
                        if(pwm_value_right_mcu <=127){ 
                            pwm_value_right_mcu = 127;
                        }else if(pwm_value_right_mcu >=190){
                        #if 0
                            if(encoder_right_counter_sum<=set_fixed_distance_encoder_value)
                                pwm_value_right_mcu=190-pwm_right_compensate_value_forward;
                            else
                                pwm_value_right_mcu = 190;
                        #endif
                        pwm_value_right_mcu = 190;
                            
                        }
                        
                        proc_cmd("forward" , pwm_value_left_mcu , pwm_value_right_mcu);
                }
		}
		else if(car_state == CAR_STATE_MOVE_BACK) {
                if(encoder_left_counter_sum>= math_round((float)global_specified_distance_value*0.86f*500.0f) && encoder_right_counter_sum>=math_round((float)global_specified_distance_value*0.86f*500.0f)){
                    proc_cmd("stop" , MOTOR_IDLE_VOLTAGE , MOTOR_IDLE_VOLTAGE);
                    car_state = CAR_STATE_IDLE;
                    complete_specified_dist_goals=1;
                    encoder_left_counter_sum=0;
                    encoder_right_counter_sum=0; 
#ifdef ACCESS_SDIO         
                    access_data_to_sdio();
#endif
                }
                //else if(encoder_left_counter==0&&encoder_right_counter==0){
                //    ;//proc_cmd("stop" , MOTOR_IDLE_VOLTAGE , MOTOR_IDLE_VOLTAGE);//detected no feedback
                //}
                else{/*detected encoder data, start to PID alg.*/
#ifdef ACCESS_SDIO         
                    access_data_to_sdio();
#endif
                    /*calculate Position PID of two motor*/    
                    pwm_value_left_pid = math_round(PID_Pos_Calc(&PID_Motor_L , set_encoder_count , encoder_left_counter));
                    pwm_value_right_pid = math_round(PID_Pos_Calc(&PID_Motor_R , set_encoder_count , encoder_right_counter));
                    
                    pwm_value_left_err =pwm_value_left_pid - pwm_value_left;
                    pwm_value_right_err =pwm_value_right_pid - pwm_value_right;
                 
                    /*record the last pwm value.*/
                    pwm_value_left = pwm_value_left_pid; 
                    pwm_value_right = pwm_value_right_pid;
  
                    pwm_value_left_mcu = pwm_value_left_mcu-pwm_value_left_err;
                    pwm_value_right_mcu = pwm_value_right_mcu-pwm_value_right_err;
                    /** 
                     * the motor driver of pwm value accept CW is 127~200 for security
                     * the motor driver of pwm value accept CCW is 50~127 for security
                     **/
                    if(pwm_value_left_mcu <=90){
                        #if 0
                        if(encoder_left_counter_sum<=set_fixed_distance_encoder_value)
                            pwm_value_left_mcu=90+pwm_left_compensate_value_back;
                        else
                            pwm_value_left_mcu = 90;
                        #endif
                        pwm_value_left_mcu = 90;
                    }else if (pwm_value_left_mcu >=127){
                        pwm_value_left_mcu = 127;
                    }

                    if(pwm_value_right_mcu <=90){
                        #if 0
                        if(encoder_right_counter_sum<=set_fixed_distance_encoder_value)
                            pwm_value_right_mcu=90+pwm_right_compensate_value_back;
                        else
                            pwm_value_right_mcu = 90;
                        #endif
                        pwm_value_right_mcu = 90;
                    }else if(pwm_value_right_mcu >=127){
                        pwm_value_right_mcu = 127;
                    }
                    
                    proc_cmd("backward" , pwm_value_left_mcu , pwm_value_right_mcu);
                }
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
    rpm_right_motor=(float)encoder_right_counter * 60.0f  /600.0f / 0.02f;    
#else
    /*for SmartEPW is used by 500 pulse/rev,  =>  encoder_left_counter * (2.0f/3.0f)* 60.0f /500.0f / 0.02f*/
    rpm_left_motor=(float)encoder_left_counter * 4.0f;
    rpm_right_motor=(float)encoder_right_counter * 4.0f;    


    /*100msec version.    (float)encoder_left_counter *(2.0f/3.0f) * 60.0f /500.0f / 0.1f*/
    //rpm_left_motor=(float)encoder_left_counter * 0.8f;
    //rpm_right_motor=(float)encoder_right_counter * 0.8f; 
    
    //get the speed of the two motors in deg/sec
    /***do someting***/ 
#endif
}

void accsess_sdio_setup()
{
    // Initialise media
    media_init();

    // Initialise File IO Library
    fl_init();

    // Attach media access functions to library
    if (fl_attach_media(media_read, media_write) != FAT_INIT_OK)
    {
        printf("ERROR: Media attach failed\n");
        return; 
    }
    // List root directory
    fl_listdirectory("/");

    // Create File
    file = fl_fopen("/PID_2.txt", "a+");
    if (file)
    {
       printf("success to open the file\n");
    }
    else
       printf("ERROR: Create file failed\n");
    
           
    fl_fputs("Hello",file);

   
    // Close file
    fl_fclose(file);
    
    // Delete File
    //if (fl_remove("/file.bin") < 0)
    //printf("ERROR: Delete file failed\n");

    // List root directory
    //fl_listdirectory("/");

    //fl_shutdown();
}
void access_data_to_sdio(void){
    static char dir_str[4] = {0,0,0,0};
    static char Kp_str[4];
    static char Ki_str[4];
    static char Kd_str[4];
    static char Kp_right_str[4];
    static char Ki_right_str[4];
    static char Kd_right_str[4];
    static char pwm_left_value_str[4];
    static char pwm_right_value_str[4];
    static char encoder_left_cnt_str[4];
    static char encoder_right_cnt_str[4];
    static char rpm_left_str[4];
    static char rpm_right_str[4];
    static char set_rpm_str[4];
    static char set_encoder_count_str[4];
    
    if(car_state==CAR_STATE_MOVE_FORWARD) dir_str[0] = 'f';
    else if(car_state==CAR_STATE_MOVE_BACK) dir_str[0] = 'b';
    
    int2str((int)(Kp_left*10.0f), Kp_str);
    int2str((int)(Ki_left*10.0f), Ki_str);
    int2str((int)(Kd_left*10.0f), Kd_str);
    int2str((int)(Kp_right*10.0f), Kp_right_str);
    int2str((int)(Ki_right*10.0f), Ki_right_str);
    int2str((int)(Kd_right*10.0f), Kd_right_str);
    
    
    int2str((int)(pwm_value_left_mcu), pwm_left_value_str);
    int2str((int)(pwm_value_right_mcu), pwm_right_value_str);
    
   
    int2str((int)(encoder_left_counter), encoder_left_cnt_str);
    int2str((int)(encoder_right_counter), encoder_right_cnt_str);
    int2str((int)(rpm_left_motor), rpm_left_str);
    int2str((int)(rpm_right_motor), rpm_right_str);
    //int2str((int)(set_rpm), set_rpm_str);
    int2str((int)(set_encoder_count), set_encoder_count_str);
    
     // Create File
    file = fl_fopen("/PID_2.txt", "a+");
    if (file)
    {
        printf("success to open the file\n");
    }
    else
        printf("ERROR: Create file failed\n");

    fl_fseek(file,0,SEEK_END);
    //fl_fread(index,1,sizeof(unsigned char),file);

    fl_fputs(dir_str,file);
    fl_fputs("\t",file);
    fl_fputs(Kp_str,file);
    fl_fputs("\t",file);
    fl_fputs(Ki_str,file);
    fl_fputs("\t",file);
    fl_fputs(Kd_str,file);
    fl_fputs("\t",file);
    fl_fputs(Kp_right_str,file);
    fl_fputs("\t",file);
    fl_fputs(Ki_right_str,file);
    fl_fputs("\t",file);
    fl_fputs(Kd_right_str,file);
    fl_fputs("\t",file);
    fl_fputs(pwm_left_value_str,file);
    fl_fputs("\t",file);
    fl_fputs(pwm_right_value_str,file);
    fl_fputs("\t",file);
    fl_fputs(encoder_left_cnt_str,file);
    fl_fputs("\t",file);
    fl_fputs(encoder_right_cnt_str,file);
    fl_fputs("\t",file);
    fl_fputs(rpm_left_str,file);
    fl_fputs("\t",file);
    fl_fputs(rpm_right_str,file);
    fl_fputs("\t",file);
    fl_fputs(set_encoder_count_str,file);
    //fl_fputs(set_rpm_str,file);
    fl_fputs("\t",file);
    fl_fputs("\n",file);

    /*split the period of recording.*/
    if(complete_specified_dist_goals){
        fl_fputs("\n",file);
        complete_specified_dist_goals=0;
    }
    
    // Close file
    fl_fclose(file);
}
void EXTI0_IRQHandler(){

		if(EXTI_GetITStatus(EXTI_Line0) != RESET)
		{
                encoder_left_counter_sum++; 
				encoder_left_counter++ ;
				EXTI_ClearITPendingBit(EXTI_Line0);
		}
}

void EXTI1_IRQHandler(){
		if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{
            
                encoder_right_counter_sum++; 
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
