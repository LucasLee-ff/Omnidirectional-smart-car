/*
 * Motor.c
 *
 *  Created on: 2021年2月21日
 *      Author: 16917
 */
#include "headfile.h"
#include "Motor.h"
#define PWM_FREQ 17000
#define MAX_DUTY 10000


void Duty_Init()
{
    pwm_init(PWM4_CH1_B6,PWM_FREQ,0);//电机PWM引脚初始化
    pwm_init(PWM4_CH2_B7,PWM_FREQ,0);
    pwm_init(PWM4_CH3_B8,PWM_FREQ,0);
    pwm_init(PWM4_CH4_B9,PWM_FREQ,0);

    gpio_init(C10,GPO,0,GPIO_PIN_CONFIG);//电机方向引脚初始化
    gpio_init(C11,GPO,0,GPIO_PIN_CONFIG);
    gpio_init(B12,GPO,1,GPIO_PIN_CONFIG);
    gpio_init(A8,GPO,1,GPIO_PIN_CONFIG);
    return;
}

void Duty_Single(Wheel_Type wh,int32 duty)//设定某个轮子的占空比
{
    if(duty>10000)
        duty=10000;
    if(duty<-10000)
        duty=-10000;
    uint32 tmp;
    if(duty<0)
        tmp=(uint32)-duty;
    else
        tmp=(uint32)duty;
    switch(wh)
    {
    case FL:
        if(duty>0)
            gpio_set(C10,0);
        else
            gpio_set(C10,1);
        pwm_duty(PWM4_CH1_B6,tmp);
        break;
    case FR:
        if(duty>0)
            gpio_set(C11,0);
        else
            gpio_set(C11,1);
        pwm_duty(PWM4_CH2_B7,tmp);
        break;
    case RL:
        if(duty>0)
            gpio_set(B12,1);
        else
            gpio_set(B12,0);
        pwm_duty(PWM4_CH3_B8,tmp);
        break;
    case RR:
        if(duty>0)
            gpio_set(A8,1);
        else
            gpio_set(A8,0);
        pwm_duty(PWM4_CH4_B9,tmp);
        break;
    }
    return;
}

void Duty_All(int32 duty_fl,int32 duty_fr,int32 duty_rl,int32 duty_rr)//设定所有轮子的占空比
{
    Duty_Single(FL,duty_fl);
    Duty_Single(FR,duty_fr);
    Duty_Single(RL,duty_rl);
    Duty_Single(RR,duty_rr);
    return;
}
