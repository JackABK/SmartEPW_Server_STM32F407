/*=============================================================================
  *
  * @file     : PID.h
  * @data       : 2014/2/6
  * @brief   : PID.c header file
  *
  *============================================================================*/
#ifndef __PID_H__
#define __PID_H__


typedef struct _pid_struct {
	float Kp;
	float Ki;
	float Kd;
	float err;
	float last_err;
	float prev_err;
    float sum_err;
    float p_Term;
    float i_Term;
    float d_Term;
    float output;
    float out_max;
    float out_min;
} pid_struct;


extern inline void InitPID (pid_struct *pid,float p,float i,float d);
extern inline float PID_Pos_Calc(pid_struct *pid ,float x,float y );
extern inline float PID_Inc_Calc(pid_struct *pid ,float x,float y );



#endif /* __PID_H__ */
