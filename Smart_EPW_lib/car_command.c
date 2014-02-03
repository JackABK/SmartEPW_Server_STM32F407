#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include  "clib.h"
#include  "car_command.h"


typedef void cmd_func(uint32_t SpeedValue_left , uint32_t SpeedValue_right);
typedef struct{
		char * cmd_name ;
		char * desc;
		cmd_func * cmd_func_handler;
}cmd_list;

/* using preprocessor to beautify the command , refer to: 
 *   1.http://gcc.gnu.org/onlinedocs/cpp/Stringification.html 
 *   2.http://stackoverflow.com/questions/216875/in-macros/216912
 */
#define DECLARE_COMMAND(n, d) {.cmd_name=#n, .cmd_func_handler=n ## _cmd, .desc=d}

static cmd_list  motor_cmds_table[] = {
		DECLARE_COMMAND(forward,"Motor Forward!!"),
		DECLARE_COMMAND(stop,"Motor Stop!!"),
		DECLARE_COMMAND(backward,"Motor Backward!!"),
		DECLARE_COMMAND(left,"Motor Left!!"),
		DECLARE_COMMAND(right,"Motor Right!!")
};


/* =========================Control Function============================ */
inline void forward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right){
		TIM_SetCompare2(TIM4, SpeedValue_left);    
		TIM_SetCompare4(TIM4, SpeedValue_right);    
}
inline void stop_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right){
		/*stop will be always zero.**/
		TIM_SetCompare2(TIM4, 0);    
		TIM_SetCompare4(TIM4, 0);  
}
inline void backward_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right){
		TIM_SetCompare2(TIM4, SpeedValue_left);    
		TIM_SetCompare4(TIM4, SpeedValue_right);

}
inline void left_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right){
		/*forward and turn left big angle*/

}
inline void right_cmd(uint32_t SpeedValue_left , uint32_t SpeedValue_right){
		/*forward and turn right big angle*/

}
/* ======================End of the Control Function===================== */


void proc_cmd(char *cmd , uint32_t SpeedValue_left , uint32_t SpeedValue_right )
{
		int i;
		for (i = 0; i < sizeof(motor_cmds_table)/sizeof(cmd_list); i++) {          
				if(strncmp(cmd, motor_cmds_table[i].cmd_name , strlen(motor_cmds_table[i].cmd_name)) == 0){
						motor_cmds_table[i].cmd_func_handler(SpeedValue_left , SpeedValue_right);
						return;
				}      
		}
} 


