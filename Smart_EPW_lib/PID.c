 /*=============================================================================
  *
  * @file     : PID.c
  * @data     : 2014/2/6
  * @brief    : pid control algorithm by Postion and Incremental method
  *
  *============================================================================*/
#include "stm32f4xx.h"
#include "PID.h"


inline void InitPID (pid_struct *pid,float p,float i,float d) {
	pid->Kp = p;	
    pid->Ki = i;	
    pid->Kd = d;
	pid->err = 0.0f;	
	pid->last_err = 0.0f;	
    pid->prev_err = 0.0f;
    pid->sum_err = 0.0f;
    pid->p_Term = 0.0f;
    pid->i_Term = 0.0f;
    pid->d_Term = 0.0f;
    pid->output = 0.0f;	    
    pid->out_max = 255.0f;
    pid->out_min = 0.0f;
    
}


/* PID Position control. */
inline float PID_Pos_Calc(pid_struct *pid ,float x,float y ) {
	/*----------------------------------------------------------------------
     * The Formula of Position PID controller:
	 * e(n) = x(n) - y(n)
     * u(n) = Kp * e(n) + Ki * SUM(e(i), i: from 0 to n) + Kd * (e(n) - e(n-1))
     *----------------------------------------------------------------------*/
	pid->err = x - y;
	pid->sum_err += pid->err;


    pid->p_Term = pid->Kp*pid->err;
    pid->i_Term = pid->Ki*pid->sum_err;
    pid->d_Term = pid->Kd*(pid->err-pid->last_err);


    if(pid->i_Term > 255.0f) pid->i_Term = 255.0f;
    else if(pid->i_Term < 0.0f) pid->i_Term = 0.0f;

    pid->output = pid->p_Term + pid->i_Term + pid->d_Term;
    
    if(pid->output > pid->out_max) pid->output = pid->out_max;
    else if(pid->output < pid->out_min) pid->output = pid->out_min;

	pid->last_err = pid->err;
	return pid->output;
}

/* PID Incremental control. */
inline float PID_Inc_Calc(pid_struct *pid ,float x,float y ) {
	/*------------------------------------------------------------------------
	 * The Formula of Incremental PID controller:
	 * e(n) = x(n) - y(n)
	 * u(n) = Kp * e(n) + Ki * SUM(e(i), i: from 0 to n) + Kd * (e(n) - e(n-1))
	 *-----------------------------------------------------------------------*/
	
	/* Move the previous error. */
 
	/* Have the current error. */
	pid->err = x- y;

    pid->p_Term = pid->Kp*(pid->err - pid->last_err);
    pid->i_Term = pid->Ki*pid->prev_err;
    pid->d_Term = pid->Kd*(pid->err - 2.0f*pid->last_err + pid->prev_err);

    if(pid->i_Term > 255.0f) pid->i_Term = 255.0f;
    else if(pid->i_Term < 0.0f) pid->i_Term = 0.0f;
    
    pid->output = pid->p_Term + pid->i_Term + pid->d_Term;

    if(pid->output > pid->out_max)  pid->output = pid->out_max;
    else if(pid->output < pid->out_min)  pid->output = pid->out_min;

    pid->prev_err = pid->last_err;
	pid->last_err = pid->err;
	return pid->output;
}


