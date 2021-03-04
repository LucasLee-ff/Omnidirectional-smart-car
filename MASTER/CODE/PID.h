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
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_last_error;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
}Pid_Param;

//float straight_Speed=400,round_Speed=200,branch_Speed=300;

void PID_Init(Pid_Param *tmp);
void PID_incCtrl(Pid_Param *tmp, float error);

#endif /* CODE_PID_H_ */
