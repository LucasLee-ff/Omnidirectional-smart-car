/*
 * PID.c
 *
 *  Created on: 2021年2月21日
 *      Author: 16917
 */
#include "headfile.h"
#include "PID.h"
#include "Motor.h"

//extern Pid_Param Pid_fl,Pid_fr,Pid_rl,Pid_rr;
//extern int16 master_encoder_left,master_encoder_right;
//extern int16 slave_encoder_left,slave_encoder_right;

void PID_Init(Pid_Param *tmp)//PID初始化
{
    tmp->kp=17;// P:115 I: 2 D: 15
    tmp->ki=0;// 5ms:125,2,80
    tmp->kd=0;//10ms: 13,5,20
    tmp->imax=6000;//

    tmp->out=0;
    tmp->out_p=0;
    tmp->out_i=0;
    tmp->out_d=0;

    tmp->integrator=0;
    tmp->last_last_error=0;
    tmp->last_error=0;
    tmp->last_t=0;
}

void PID_incCtrl(Pid_Param *tmp, float error)
{
    tmp->out_p = tmp->kp * (error-tmp->last_error);
    tmp->out_i=tmp->ki * error;
    tmp->out_d = tmp->kd * (error - 2*tmp->last_error + tmp->last_last_error);

    tmp->last_last_error = tmp->last_error;
    tmp->last_error = error;

    tmp->out = tmp->out+tmp->out_p + tmp->out_i + tmp->out_d;

    if(tmp->out > 6000)
        tmp->out = 6000;
    if(tmp->out < -6000)
        tmp->out = -6000;
}
