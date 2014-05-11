
#include "FreeRTOS.h"
#include "timers.h"
#include "ultrasound.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "delay.h"

#define GPIO_PORT_TRIGGER_0 GPIOA
#define GPIO_PIN_TRIGGER_0 GPIO_Pin_4
#define GPIO_PORT_TRIGGER_1 GPIOA
#define GPIO_PIN_TRIGGER_1 GPIO_Pin_5
#define GPIO_PORT_ECHO_0 GPIOA
#define GPIO_PIN_ECHO_0 GPIO_Pin_6
#define GPIO_PORT_ECHO_1 GPIOA
#define GPIO_PIN_ECHO_1 GPIO_Pin_7

#define GPIO_PORT_TRIGGER_2 GPIOC
#define GPIO_PIN_TRIGGER_2 GPIO_Pin_4
#define GPIO_PORT_TRIGGER_3 GPIOC
#define GPIO_PIN_TRIGGER_3 GPIO_Pin_5
#define GPIO_PORT_ECHO_2 GPIOB
#define GPIO_PIN_ECHO_2 GPIO_Pin_0
#define GPIO_PORT_ECHO_3 GPIOB
#define GPIO_PIN_ECHO_3 GPIO_Pin_1


static unsigned long CH1Distance[2];
static unsigned long CH2Distance[2];
static unsigned long CH3Distance[2];
static unsigned long CH4Distance[2];

static xTimerHandle ultrasoundTimers;

static unsigned long IC3CH1ReadValue1,IC3CH1ReadValue2, IC3CH2ReadValue1,IC3CH2ReadValue2;
static unsigned char CH1CaptureNumber=0,CH2CaptureNumber=0;

static unsigned long IC3CH3ReadValue1,IC3CH3ReadValue2, IC3CH4ReadValue1,IC3CH4ReadValue2;
static unsigned char CH3CaptureNumber=0,CH4CaptureNumber=0;

TIM_ICInitTypeDef TIM_ICInitStructure;


void init_trigger(void);
void init_echo(void);



static unsigned char state4chnl=0;

/*chnl:0~3*/
void issue_sound_pulse(){
    int i;
    if(state4chnl==0){
        GPIO_WriteBit(GPIO_PORT_TRIGGER_0,GPIO_PIN_TRIGGER_0,Bit_SET);
    }
    else if(state4chnl==1){
        GPIO_WriteBit(GPIO_PORT_TRIGGER_1,GPIO_PIN_TRIGGER_1,Bit_SET);
    }
    else if(state4chnl==2){
        GPIO_WriteBit(GPIO_PORT_TRIGGER_2,GPIO_PIN_TRIGGER_2,Bit_SET);
    }
    else if(state4chnl==3){
        GPIO_WriteBit(GPIO_PORT_TRIGGER_3,GPIO_PIN_TRIGGER_3,Bit_SET);
    }
    /*10us delay*/
    delay_us(10);

    if(state4chnl==0)
        GPIO_WriteBit(GPIO_PORT_TRIGGER_0,GPIO_PIN_TRIGGER_0,Bit_RESET);
    else if(state4chnl==1)
        GPIO_WriteBit(GPIO_PORT_TRIGGER_1,GPIO_PIN_TRIGGER_1,Bit_RESET);
    else if(state4chnl==2)
        GPIO_WriteBit(GPIO_PORT_TRIGGER_2,GPIO_PIN_TRIGGER_2,Bit_RESET);
    else if(state4chnl==3)
        GPIO_WriteBit(GPIO_PORT_TRIGGER_3,GPIO_PIN_TRIGGER_3,Bit_RESET);

}

//#define UltrasoundPolling issue_sound_pulse
#if (1)


void UltrasoundPolling(){

    issue_sound_pulse();
    state4chnl++;
    state4chnl&=0x03;
}
#endif

void init_periph_clk(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);


}
void ultra_sound_init(){
    init_periph_clk();
    init_echo();
    init_trigger();

    ultrasoundTimers=xTimerCreate("ultrasound",(30 ), pdTRUE, ( void * ) 1,  UltrasoundPolling	 );
    xTimerStart( ultrasoundTimers, 0 );


}
void init_echo(){

    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;


    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ECHO_0|GPIO_PIN_ECHO_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
    GPIO_Init(GPIO_PORT_ECHO_0, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_ECHO_2|GPIO_PIN_ECHO_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
    GPIO_Init(GPIO_PORT_ECHO_2, &GPIO_InitStructure);



    GPIO_PinAFConfig(GPIO_PORT_ECHO_0,GPIO_PinSource6,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIO_PORT_ECHO_0,GPIO_PinSource7,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIO_PORT_ECHO_2,GPIO_PinSource0,GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIO_PORT_ECHO_2,GPIO_PinSource1,GPIO_AF_TIM3);


    TIM_PrescalerConfig(TIM3,84-1,TIM_PSCReloadMode_Immediate);//24MHz->1Mz(1us)

    /* Enable the TIM3 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3,&TIM_ICInitStructure);

    TIM_Cmd(TIM3, ENABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);


}



void init_trigger(){
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin =	GPIO_PIN_TRIGGER_0|GPIO_PIN_TRIGGER_1; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			  // Alt Function - Push Pull
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIO_PORT_TRIGGER_0, &GPIO_InitStruct );

    GPIO_InitStruct.GPIO_Pin =	GPIO_PIN_TRIGGER_2|GPIO_PIN_TRIGGER_3; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;			  // Alt Function - Push Pull
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIO_PORT_TRIGGER_2, &GPIO_InitStruct );


    GPIO_WriteBit(GPIO_PORT_TRIGGER_0,GPIO_PIN_TRIGGER_0,Bit_RESET);
    GPIO_WriteBit(GPIO_PORT_TRIGGER_1,GPIO_PIN_TRIGGER_1,Bit_RESET);
    GPIO_WriteBit(GPIO_PORT_TRIGGER_2,GPIO_PIN_TRIGGER_2,Bit_RESET);
    GPIO_WriteBit(GPIO_PORT_TRIGGER_3,GPIO_PIN_TRIGGER_3,Bit_RESET);

}


void TIM3_IRQHandler(void){
    if(TIM_GetITStatus(TIM3,TIM_IT_CC1)==SET){
        if(CH1CaptureNumber == 0){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH1ReadValue1=TIM_GetCapture1(TIM3);
            CH1CaptureNumber=1;

        }
        else if(CH1CaptureNumber ==1){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH1ReadValue2=TIM_GetCapture1(TIM3);
            CH1Distance[0]=CH1Distance[1];
            if(IC3CH1ReadValue2>IC3CH1ReadValue1){
                CH1Distance[1] = (IC3CH1ReadValue2 - IC3CH1ReadValue1); 
            }
            else{
                CH1Distance[1] = (0xFFFF-IC3CH1ReadValue1) + IC3CH1ReadValue2; 
            }
            CH1Distance[1]/=58;
            CH1CaptureNumber=0;

        }
        //TIM_ITConfig(TIM3, TIM_IT_CC1, DISABLE);
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
    }
    else if(TIM_GetITStatus(TIM3,TIM_IT_CC2)==SET){
        if(CH2CaptureNumber == 0){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH2ReadValue1=TIM_GetCapture2(TIM3);
            CH2CaptureNumber=1;

        }
        else if(CH2CaptureNumber ==1){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH2ReadValue2=TIM_GetCapture2(TIM3);
            CH2Distance[0]=CH2Distance[1];
            if(IC3CH2ReadValue2>IC3CH2ReadValue1){
                CH2Distance[1]= (IC3CH2ReadValue2 - IC3CH2ReadValue1); 
            }
            else{
                CH2Distance[1] = (0xFFFF-IC3CH2ReadValue1) + IC3CH2ReadValue2; 
            }
            CH2Distance[1]/=58;
            CH2CaptureNumber=0;

        }

        //TIM_ITConfig(TIM3, TIM_IT_CC2, DISABLE);
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
    }
    else if(TIM_GetITStatus(TIM3,TIM_IT_CC3)==SET){
        if(CH3CaptureNumber == 0){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH3ReadValue1=TIM_GetCapture3(TIM3);
            CH3CaptureNumber=1;

        }
        else if(CH3CaptureNumber ==1){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH3ReadValue2=TIM_GetCapture3(TIM3);
            CH3Distance[0]=CH3Distance[1];
            if(IC3CH3ReadValue2>IC3CH3ReadValue1){
                CH3Distance[1]= (IC3CH3ReadValue2 - IC3CH3ReadValue1); 
            }
            else{
                CH3Distance[1] = (0xFFFF-IC3CH3ReadValue1) + IC3CH3ReadValue2; 
            }
            CH3Distance[1]/=58;
            CH3CaptureNumber=0;

        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
    }
    else if(TIM_GetITStatus(TIM3,TIM_IT_CC4)==SET){
        if(CH4CaptureNumber == 0){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH4ReadValue1=TIM_GetCapture4(TIM3);
            CH4CaptureNumber=1;

        }
        else if(CH4CaptureNumber ==1){
            TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
            TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
            TIM_ICInit(TIM3,&TIM_ICInitStructure);
            IC3CH4ReadValue2=TIM_GetCapture4(TIM3);
            CH4Distance[0]=CH4Distance[1];
            if(IC3CH4ReadValue2>IC3CH4ReadValue1){
                CH4Distance[1]= (IC3CH4ReadValue2 - IC3CH4ReadValue1); 
            }
            else{
                CH4Distance[1] = (0xFFFF-IC3CH4ReadValue1) + IC3CH4ReadValue2; 
            }
            CH4Distance[1]/=58;
            CH4CaptureNumber=0;

        }
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
    }



}

unsigned long Get_CH1Distance(){
    if((long)(CH1Distance[1]-CH1Distance[0])>100 || (long)(CH1Distance[0]-CH1Distance[1])>100)
        return 0;
    return CH1Distance[0];
}

unsigned long Get_CH2Distance(){
    if((long)(CH2Distance[1]-CH2Distance[0])>100 || (long)(CH2Distance[0]-CH2Distance[1])>100)
        return 0;
    return CH2Distance[0];

}
unsigned long Get_CH3Distance(){
    if((long)(CH3Distance[1]-CH3Distance[0])>100 || (long)(CH3Distance[0]-CH3Distance[1])>100)
        return 0;
    return CH3Distance[0];

}
unsigned long Get_CH4Distance(){
    if((long)(CH4Distance[1]-CH4Distance[0])>100 || (long)(CH4Distance[0]-CH4Distance[1])>100)
        return 0;
    return CH4Distance[0];

}



