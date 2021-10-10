/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            main
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//整套推荐IO查看Projecct文件夹下的TXT文本

//打开新的工程或者工程移动了位置务必执行以下操作
//右键单击工程，选择刷新

#include "headfile.h"
#include "Pid.h"
#include "Motor.h"
#include "shangweiji.h"

#define E_START                 0       //准备状态
#define E_OK                    1       //成功
#define E_FRAME_HEADER_ERROR    2       //帧头错误
#define E_FRAME_RTAIL_ERROR     3       //帧尾错误

#define LINE_LEN                14      //数据长度
uint8   temp_buff[LINE_LEN];            //主机用于接收数据的BUFF
vuint8  uart_flag;                      //接收数据标志位

int16 slave_encoder_left;               //从机左编码器值
int16 slave_encoder_right;              //从机右编码器值
int16 slave_position;                   //从机转角值
int16 master_encoder_left;              //主机左编码器值
int16 master_encoder_right;             //主机右编码器值

int16 transinfo,transinfo_last=5;                       //传输信息

int8 motion_Sign;//
uint8 show_flag;//数据显示标志位
int16 slave_position_filter=0,slave_position_filter_Cnt=0;
int16 target_main_speed=100,target_sup_speed=70,target_rotate_speed=80,target_out_speed=70,target_rotate_sup_speed=60;
int16 target_fl_speed,target_fr_speed,target_rl_speed,target_rr_speed;
Pid_Param Pid_fl,Pid_fr,Pid_rl,Pid_rr,Pid_dir_redbull,Pid_dir_crab; //Pid参数结构
uint16 servo_Init_Duty=750;
uint16 servo_StoL_duty = 1250,servo_StoR_duty=250;
uint16 servo_toS_duty=750;//舵机pwm信号

//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取从机数据
//  @param      data            串口数据
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_slave_data(uint8 data)
{
    static uint8 uart_num = 0;
    temp_buff[uart_num++] = data;

    if(1 == uart_num)
    {
        //接收到的第一个字符不为0xD8，帧头错误
        if(0xD8 != temp_buff[0])
        {
            uart_num = 0;
            uart_flag = E_FRAME_HEADER_ERROR;
        }
    }

    if(LINE_LEN == uart_num)
    {
        uart_flag = E_OK;
        //接收到最后一个字节为0xEE
        if(0xEE == temp_buff[LINE_LEN - 1])
        {
            uart_flag = E_OK;
        }
        else    //接收到最后一个字节不是0xEE，帧尾错误
        {
            uart_flag = E_FRAME_RTAIL_ERROR;
        }
        uart_num = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议对从机发送过来的数据，进行数据解析
//  @param      *line                           串口接收到的数据BUFF
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void data_analysis(uint8 *line)
{
    if(line[1] == 0xB0)    slave_encoder_left  = ((int16)line[2] << 8) | line[3];
    if(line[4] == 0xB1)    slave_encoder_right = ((int16)line[5] << 8) | line[6];
    if(line[7] == 0xB2)    slave_position      = ((int16)line[8] << 8) | line[9];
    if(line[10] == 0xB3)   transinfo           = ((int16)line[11] <<8) | line[12];
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      串口3中断服务函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void)
{
    uint8 dat;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除串口接收中断标志位
        dat = USART_ReceiveData(USART3);                //获取串口数据
        get_slave_data(dat);                            //将每一个字节的串口数据存入temp_buff中。
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      外部中断0中断服务函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void)
{
    if(SET == EXTI_GetITStatus(EXTI_Line0))
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        while(uart_flag != E_OK);                       //等待接收数据
        uart_flag = E_START;                            //清空标志位
        data_analysis(temp_buff);                       //数据解析

        slave_encoder_right=-slave_encoder_right;
        master_encoder_left=timer_quad_get(TIMER_2);//获得主机编码器的值
        master_encoder_right=-timer_quad_get(TIMER_3);//获得主机编码器的值
        timer_quad_clear(TIMER_2);
        timer_quad_clear(TIMER_3);

        if(slave_position_filter_Cnt==0)//低通滤波
        {
            slave_position_filter=slave_position;
            slave_position_filter_Cnt=1;
        }
        else
        {
            slave_position=slave_position*0.8+slave_position_filter*0.2;
            slave_position_filter=slave_position;
        }

        switch(motion_Sign)
        {
        case 0://出库状态
            if(transinfo==16)
                motion_Sign=1;
            break;
        case 1://车直走状态
            if(transinfo==46)//需要进右岔路
                motion_Sign=4;
            else if(transinfo==44)//需要进左岔路
                motion_Sign=5;
            else if(transinfo==11)//左旋转进车库
                motion_Sign=8;
            else if(transinfo==13)//右旋转进车库
                motion_Sign=9;
            break;
        case 2://车在左岔路中横着走
            if(transinfo==44)//变回直走状态时需要左旋转
                motion_Sign=6;
            break;
        case 3://车在右岔路中横着走
            if(transinfo==46)//变回直走状态时需要右旋转
                motion_Sign=7;
            break;
        case 4://车直走状态要进入右岔路，需要左旋转
            if(transinfo==33)
                motion_Sign=3;
            break;
        case 5://车直走状态要进入左岔路，需要右旋转
            if(transinfo==33)
                motion_Sign=2;
            break;
        case 6://车在左岔路中恢复直走状态，需要左旋转
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 7://车在右岔路中恢复直走状态，需要右旋转
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 8://左旋转进车库
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 9://右旋转进车库
            if(transinfo==33)
                motion_Sign=1;
        }

        switch(motion_Sign)
        {
        case 0:
            if(transinfo==55)//左出库
            {
                target_fl_speed=target_out_speed;
                target_fr_speed=-target_out_speed;
                target_rl_speed=-target_out_speed;
                target_rr_speed=target_out_speed;
            }
            else if(transinfo==57)//右出库
            {
                target_fl_speed=-target_out_speed;
                target_fr_speed=target_out_speed;
                target_rl_speed=target_out_speed;
                target_rr_speed=-target_out_speed;
            }
            break;
        case 1://直走
            PID_posCtrl(&Pid_dir_redbull, slave_position);
            target_fl_speed=target_main_speed+Pid_dir_redbull.out;
            target_fr_speed=target_main_speed-Pid_dir_redbull.out;
            target_rl_speed=target_main_speed+Pid_dir_redbull.out;
            target_rr_speed=target_main_speed-Pid_dir_redbull.out;
            break;
        case 2://左岔路
            PID_posCtrl(&Pid_dir_crab, slave_position);
            target_fl_speed=-target_sup_speed+Pid_dir_crab.out;
            target_fr_speed=target_sup_speed-Pid_dir_crab.out;
            target_rl_speed=target_sup_speed+Pid_dir_crab.out;
            target_rr_speed=-target_sup_speed-Pid_dir_crab.out;
            break;
        case 3://右岔路
            PID_posCtrl(&Pid_dir_crab, slave_position);
            target_fl_speed=target_sup_speed+Pid_dir_crab.out;
            target_fr_speed=-target_sup_speed-Pid_dir_crab.out;
            target_rl_speed=-target_sup_speed+Pid_dir_crab.out;
            target_rr_speed=target_sup_speed-Pid_dir_crab.out;
            break;
        case 4://车直走状态，进入右岔路需要左旋转
            pwm_duty(PWM1_CH1_A8, servo_StoR_duty);
            target_fl_speed=-target_rotate_speed;
            target_fr_speed=target_rotate_speed;
            target_rl_speed=-target_rotate_speed;
            target_rr_speed=target_rotate_speed;
            break;
        case 6://车在左岔路中，变为直走需要左旋转
            pwm_duty(PWM1_CH1_A8, servo_toS_duty);
            target_fl_speed=-target_rotate_sup_speed;
            target_fr_speed=target_rotate_sup_speed;
            target_rl_speed=-target_rotate_sup_speed;
            target_rr_speed=target_rotate_sup_speed;
            break;
        case 5://车直走状态，进入左岔路需要右旋转
            pwm_duty(PWM1_CH1_A8, servo_StoL_duty);
            target_fl_speed=target_rotate_speed;
            target_fr_speed=-target_rotate_speed;
            target_rl_speed=target_rotate_speed;
            target_rr_speed=-target_rotate_speed;
            break;
        case 7://车在右岔路中，变为直走需要右旋转
            pwm_duty(PWM1_CH1_A8, servo_toS_duty);
            target_fl_speed=target_rotate_sup_speed;
            target_fr_speed=-target_rotate_sup_speed;
            target_rl_speed=target_rotate_sup_speed;
            target_rr_speed=-target_rotate_sup_speed;
            break;
        case 8:
            target_fl_speed=-target_rotate_sup_speed;
            target_fr_speed=target_rotate_sup_speed;
            target_rl_speed=-target_rotate_sup_speed;
            target_rr_speed=target_rotate_sup_speed;
            break;
        case 9:
            target_fl_speed=target_rotate_sup_speed;
            target_fr_speed=-target_rotate_sup_speed;
            target_rl_speed=target_rotate_sup_speed;
            target_rr_speed=-target_rotate_sup_speed;
            break;
        }

        if(transinfo==90)//车库刹车
            target_fl_speed=target_fr_speed=target_rl_speed=target_rr_speed=0;

        PID_incCtrl(&Pid_fl,(float)(target_fl_speed-slave_encoder_left));
        PID_incCtrl(&Pid_fr,(float)(target_fr_speed-slave_encoder_right));
        PID_incCtrl(&Pid_rl,(float)(target_rl_speed-master_encoder_left));
        PID_incCtrl(&Pid_rr,(float)(target_rr_speed-master_encoder_right));

        Duty_All((int32)Pid_fl.out,(int32)Pid_fr.out,(int32)Pid_rl.out,(int32)Pid_rr.out);

        show_flag = 1;                                  //屏幕显示标志位
    }
}

void Encoder_Init_Master()//主机编码器初始化函数
{
    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//编码器3初始化，使用定时器2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//编码器4初始化，使用定时器3
    return;
}

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口

    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);  //串口3初始化，波特率460800
    uart_rx_irq(UART_3, ENABLE);                            //默认抢占优先级0 次优先级0。
    gpio_interrupt_init(A0, RISING, GPIO_INT_CONFIG);       //A0初始化为GPIO 上升沿触发
    nvic_init(EXTI0_IRQn, 1, 1, ENABLE);     //EXTI0优先级配置，抢占优先级1，次优先级1

    pwm_init(PWM1_CH1_A8, 50, 750);  //舵机初始化到中间位置
    motion_Sign=0;//运动方式控制
    PID_Init(&Pid_fl,0);
    PID_Init(&Pid_fr,1);
    PID_Init(&Pid_rl,2);
    PID_Init(&Pid_rr,3);
    PID_dir_Init(&Pid_dir_redbull,0);
    PID_dir_Init(&Pid_dir_crab,1);
    Duty_Init();
    Encoder_Init_Master();
    //总中断最后开启
    EnableGlobalIRQ(0);

    while(1)
    {
        if(show_flag==1)
        {
            show_flag=0;
        }
    }
}
