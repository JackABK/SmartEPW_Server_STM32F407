#include "linear_actuator.h"
#include "stm32f4xx.h"                                                                                  
#include "FreeRTOS.h"
#include "task.h"


unsigned int ADC_Value_temp1;



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
		TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

		TIM_Cmd(TIM3, ENABLE);

}


static void init_CWCCW(){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO D clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
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

/* Limit Switch */
static void init_LS_ADC(){
		ADC_InitTypeDef ADC_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		GPIO_InitTypeDef GPIO_InitStruct;
        DMA_InitTypeDef DMA_InitStructure;
		/* Enable GPIO C clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

        DMA_InitStructure.DMA_Channel = DMA_Channel_2; 
        DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int)0x4001224C;
        DMA_InitStructure.DMA_Memory0BaseAddr =(unsigned int) &ADC_Value_temp1;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = 1;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA2_Stream0, &DMA_InitStructure);
        DMA_Cmd(DMA2_Stream0, ENABLE);

    
		GPIO_InitStruct.GPIO_Pin =  LS_A_UPPER_PIN | LS_A_LOWER_PIN | LS_B_UPPER_PIN | LS_B_LOWER_PIN ;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(LS_READ_PORT, &GPIO_InitStruct);

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);
		
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_NbrOfConversion = 1;
		ADC_Init(ADC3, &ADC_InitStructure);

		ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

		/*ADC3 */
		ADC_Cmd(ADC3, ENABLE);

        ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);

        ADC_DMACmd(ADC3, ENABLE);

  
        ADC_SoftwareStartConv(ADC3);
}

void init_linear_actuator(){
		init_PWM();
		init_CWCCW();
		init_LS_ADC();
		return;

}

void set_linearActuator_A_cmd(int flag , int pwm_value){
		switch(flag){
				case STOP:
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_RESET);/* 0 */
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_RESET);/* 0 */
                        TIM_SetCompare1(TIM3 , 0);
						break;
				case CW:
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_RESET);/* 0 */
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_SET);/* 1 */
                        TIM_SetCompare1(TIM3 , pwm_value);
						break;
				case CCW:
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN1_PIN,Bit_SET);/* 1 */
						GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_A_IN2_PIN,Bit_RESET);/* 0 */ 
                        TIM_SetCompare1(TIM3 , pwm_value);
						break;
		}
}

void set_linearActuator_B_cmd(int flag , int pwm_value){
		switch(flag){
                case STOP:
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_RESET);/* 0 */
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_RESET);/* 0 */
                        TIM_SetCompare2(TIM3 , 0);
                        break;
                case CW:
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_RESET);/* 0 */
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_SET);/* 1 */                        
                        TIM_SetCompare2(TIM3 , pwm_value);
                        break;
                case CCW:
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN3_PIN,Bit_SET);/* 1 */
                        GPIO_WriteBit(ACTU_CWCCW_PORT,ACTU_B_IN4_PIN,Bit_RESET);/* 0 */ 
                        TIM_SetCompare2(TIM3 , pwm_value);
                        break;
        }
}

int get_LimitSwitch_A_upper_Vol(){
    /* ADC_Voltage = ADC_Value * VDD /(2^resolution - 1)
     * for our case, used of resoulution 12 bits
     */
    return ((float)ADC_Value_temp1 * 1.221f);
    
}

void get_LimitSwitch_A_lower(){
    GPIO_ReadInputDataBit(LS_READ_PORT, LS_A_LOWER_PIN);
}

void get_LimitSwitch_B_upper(){
    GPIO_ReadInputDataBit(LS_READ_PORT, LS_B_UPPER_PIN);
}

void get_LimitSwitch_B_lower(){
    GPIO_ReadInputDataBit(LS_READ_PORT, LS_B_LOWER_PIN);
}
