/*                                                                              
 ****************************************************************   
 * @file    main.c
 * @date    2013-12-12
 * @author  JackABK
 *          github: https://github.com/JackABK 
 *          e-mail: abkabkabkyes@gmail.com
 *
 * @brief   This project will complete easy PI control through encoder,
 *          it apply in the motor controller and caclulator the RPM. 
 *          The encoder signal is A leader to B 90 angle, 
 *          so that like as following graphic, it's CW.
 *              __    __
 *         A __|  |__|  |__
 *                __    __ 
 *         B   __|  |__|  |__
 *
 *
 ***************************************************************
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "example.h"
#include "car_behavior.h"
#include "uart.h"
#include  "clib.h"
#include  "shell.h"

#define USE_FILELIB_STDIO_COMPAT_NAMES

/*============================================================================*
 ** Prototype    : tesing_task
 ** Description  : test the relative mode and print to stdout , it's a task thread.
 **                          it's tesing the distance info by now.
 ** Input          : void* p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
 typedef struct DISTANCE_INFO{
				int counter;
				int avg;
		}DISTANCE_INFO_STRU;
DISTANCE_INFO_STRU distance_info_CH1;
void tesing_task(void* p) { 
		printf("Start testing the distance measure .\n");
		for (;;) {
				/*check out the system is not crash */
				//GPIO_ToggleBits(GPIOD,GPIO_Pin_12);



                /*
                forward_cmd(100 , 100);
				vTaskDelay(1000);
                
                stop_cmd(100 , 100);
				vTaskDelay(1000);
               
                backward_cmd(100 , 100);
                    vTaskDelay(1000);

                left_cmd(100 , 100);
                vTaskDelay(1000);

                right_cmd(100 , 100);
                vTaskDelay(1000);
                */
                
				if (distance_info_CH1.counter< 3   )  {
						distance_info_CH1.avg += Get_CH1Distance();
						distance_info_CH1.counter++;
				}
				else{
						distance_info_CH1.avg  =  round( (float)distance_info_CH1.avg / 3.0f);
						//printf("%ld %ld %ld %ld\n\r",distance_avg_1,Get_CH2Distance() ,Get_CH3Distance(),Get_CH4Distance() );
						//printf("%ld\n\r",distance_avg_1);

						distance_info_CH1.counter=0;
						distance_info_CH1.avg = 0;
				}
		}
		vTaskDelete(NULL);
}



void init_LED(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO C clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
		// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( GPIOD, &GPIO_InitStruct ); 
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
}


void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
		taskDISABLE_INTERRUPTS();
		for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
		(void) pcTaskName;
		(void) pxTask;
		/* Run time stack overflow checking is performed if
		   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
		   function is called if a stack overflow is detected. */
		taskDISABLE_INTERRUPTS();
		for(;;);
}

int main(void) {

		uint8_t ret = pdFALSE;
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		init_USART3(9600);
		init_LED();
		init_car();               

		/*create the task. */         
		ret = xTaskCreate(tesing_task, "FPU", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		ret &= xTaskCreate(shell_task, "remote task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		if (ret == pdTRUE) {
				printf("System Started!\n");
				vTaskStartScheduler();  // should never return
		} else {
				printf("System Error!\n");
				// --TODO blink some LEDs to indicates fatal system error
		}

		for (;;);

}





