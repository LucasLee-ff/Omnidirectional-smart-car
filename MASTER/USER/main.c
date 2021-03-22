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
int16 track;                       //赛道类型

uint8 show_flag;                        //数据显示标志位

int16 target;
int16 target_fl,target_fr,target_rl,target_rr;

Pid_Param Pid_fl,Pid_fr,Pid_rl,Pid_rr; //Pid参数结构

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
    if(line[10] == 0xB3)   track               = ((int16)line[11] <<8) | line[12];
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

        float k;//大圆环 速度95 转角0.6  S弯 55 0.5

        target=100;//80 120 90_0.75 115 1.2
        k=0.5;//很稳 100，0.95

        //最初版本 target=35 k=0.28
        if(slave_position>2||slave_position<-2)
        {
            target_fl=target+k*slave_position;
            target_fr=target-k*slave_position;
            target_rl=target+k*slave_position;
            target_rr=target-k*slave_position;
        }
        else
            target_fl=target_fr=target_rl=target_rr=target;

        OutPut_Data(slave_encoder_left, slave_encoder_right, master_encoder_left, master_encoder_right);

        PID_incCtrl(&Pid_fl,(float)(target_fl-slave_encoder_left));
        PID_incCtrl(&Pid_fr,(float)(target_fr-slave_encoder_right));
        PID_incCtrl(&Pid_rl,(float)(target_rl-master_encoder_left));
        PID_incCtrl(&Pid_rr,(float)(target_rr-master_encoder_right));
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

    ips114_init();

    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据
    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据
    //串口的抢占优先级一定要比外部中断的抢占优先级高，这样才能实时接收从机数据

    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);  //串口3初始化，波特率460800
    uart_rx_irq(UART_3, ENABLE);                            //默认抢占优先级0 次优先级0。
    gpio_interrupt_init(A0, RISING, GPIO_INT_CONFIG);       //A0初始化为GPIO 上升沿触发
    nvic_init(EXTI0_IRQn, 1, 1, ENABLE);                    //EXTI0优先级配置，抢占优先级1，次优先级1

    PID_Init(&Pid_fl);
    PID_Init(&Pid_fr);
    PID_Init(&Pid_rl);
    PID_Init(&Pid_rr);
    Duty_Init();
    Encoder_Init_Master();

    //ips114_showstr(0, 3, "test");
    //systick_delay_ms(300);
    //此处编写用户代码(例如：外设初始化代码等)
    //总中断最后开启
    EnableGlobalIRQ(0);
    while(1)
    {
        if(show_flag)
        {
            //将接收到的从机数据显示到屏幕上。

            ips114_showint16(0, 0, slave_encoder_left);
            ips114_showint16(80, 0, slave_encoder_right);
            ips114_showint16(0, 1, master_encoder_left);
            ips114_showint16(80, 1, master_encoder_right);

            ips114_showfloat(0, 2, Pid_fl.out,4, 2);
            ips114_showfloat(80,2,Pid_fr.out, 4, 2);
            ips114_showfloat(0, 3, Pid_rl.out, 4, 2);
            ips114_showfloat(80, 3, Pid_fr.out, 4, 2);

            show_flag = 0;
        }
    }
}



