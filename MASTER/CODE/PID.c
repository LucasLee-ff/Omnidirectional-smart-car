/*
 * PID.c
 *
 *  Created on: 2021��2��21��
 *      Author: 16917
 */
#include "headfile.h"
#include "PID.h"
#include "Motor.h"

//extern Pid_Param Pid_fl,Pid_fr,Pid_rl,Pid_rr;
//extern int16 master_encoder_left,master_encoder_right;
//extern int16 slave_encoder_left,slave_encoder_right;

void PID_dir_Init(Pid_Param *tmp,int8 sign)
{
    if(sign==0)
    {
        tmp->kp=0.8;//0.92(�ٶ�110), 1.15(�ٶ�120), 1.26(�ٶ�130), 1.47(�ٶ�140)
        tmp->ki=0;
        tmp->kd=1;//0.8(�ٶ�110), 0.5(�ٶ�120), 0.5(�ٶ�130), 0.3(�ٶ�140)
    }
    else if(sign==1)
    {
        tmp->kp=0.7;//0.63(�ٶ�70), 0.95(�ٶ�80), 1.05(�ٶ�90), 1.15(�ٶ�95)
        tmp->ki=0;
        tmp->kd=0;//
    }
        tmp->out=0;
        tmp->out_p=0;
        tmp->out_i=0;
        tmp->out_d=0;

        tmp->last_last_error=0;
        tmp->last_error=0;
}

void PID_Init(Pid_Param *tmp,int8 sign)//PID��ʼ��
{
    switch(sign)
    {
    case 0: tmp->kp=102;//5.29--106,23,150
            tmp->ki=26;//250: 102,26,100
            tmp->kd=100;//150: 97,19,140
            break;
    case 1: tmp->kp=102;//5.29--106,23,150
            tmp->ki=26;//250: 102,26,100
            tmp->kd=100;//150: 97,21,130
            break;
    case 2: tmp->kp=89;
            tmp->ki=19;//250: 89,19,150
            tmp->kd=150;//150: 70,13,100
            break;
    case 3: tmp->kp=110;
            tmp->ki=28;//250: 110,28,120
            tmp->kd=120;//150: 81,15,110
            break;
    }
    tmp->out=0;
    tmp->out_p=0;
    tmp->out_i=0;
    tmp->out_d=0;

    tmp->last_out_d=0;
    tmp->last_last_error=0;
    tmp->last_error=0;
}

void PID_posCtrl(Pid_Param *tmp, float error)
{
    tmp->out_p = tmp->kp * error;
    tmp->out_i += tmp->ki * error;
    tmp->out_d = tmp->kd*(error - tmp->last_error);

    tmp->last_error = error;

    tmp->out = tmp->out_p + tmp->out_i + tmp->out_d;
}

void PID_incCtrl(Pid_Param *tmp, float error)
{
    tmp->out_p = tmp->kp * (error-tmp->last_error);

    tmp->out_i=tmp->ki * error;
    tmp->out_d = tmp->kd * (error - 2*tmp->last_error + tmp->last_last_error);

    tmp->last_last_error = tmp->last_error;
    tmp->last_error = error;
    tmp->last_out_d = tmp->out_d;

    tmp->out = tmp->out + tmp->out_p + tmp->out_i + tmp->out_d;

    if(tmp->out > 8000)
        tmp->out = 8000;
    if(tmp->out < -8000)
        tmp->out = -8000;
}
