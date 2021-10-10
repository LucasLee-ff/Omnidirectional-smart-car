/*
 * PID.h
 *
 *  Created on: 2021年2月21日
 *      Author: 16917
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D

    float                out_p;  //P输出
    float                out_i;  //I输出
    float                out_d;  //D输出
    float                out;    //pid输出
    float                last_out_d;//上次D输出

    float                last_error; //< 上次误差
    float                last_last_error;//< 上次误差与上上次误差之差
}Pid_Param;

//float straight_Speed=400,round_Speed=200,branch_Speed=300;

void PID_Init(Pid_Param *tmp,int8 sign);
void PID_dir_Init(Pid_Param *tmp,int8 sign);
void PID_incCtrl(Pid_Param *tmp, float error);
void PID_posCtrl(Pid_Param *tmp, float error);

#endif /* CODE_PID_H_ */
