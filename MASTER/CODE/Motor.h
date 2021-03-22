/*
 * Motor.h
 *
 *  Created on: 2021��2��21��
 *      Author: 16917
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

typedef enum{
    FL=1,
    FR,
    RL,
    RR,
}Wheel_Type;//�ĸ����ӵ�ö�ٶ���

void Duty_Init();
void Duty_Single(Wheel_Type wh,int32 duty);
void Duty_All(int32 duty_fl,int32 duty_fr,int32 duty_rl,int32 duty_rr);
void Duty_Close();
#endif /* CODE_MOTOR_H_ */
