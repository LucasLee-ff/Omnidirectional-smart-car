/*
 * PID.h
 *
 *  Created on: 2021��2��21��
 *      Author: 16917
 */

#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D

    float                out_p;  //P���
    float                out_i;  //I���
    float                out_d;  //D���
    float                out;    //pid���
    float                last_out_d;//�ϴ�D���

    float                last_error; //< �ϴ����
    float                last_last_error;//< �ϴ���������ϴ����֮��
}Pid_Param;

//float straight_Speed=400,round_Speed=200,branch_Speed=300;

void PID_Init(Pid_Param *tmp,int8 sign);
void PID_dir_Init(Pid_Param *tmp,int8 sign);
void PID_incCtrl(Pid_Param *tmp, float error);
void PID_posCtrl(Pid_Param *tmp, float error);

#endif /* CODE_PID_H_ */
