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
#include "TrackGet.h"

#define LINE_LEN                11              //数据长度
uint8 temp_buff[LINE_LEN];                      //从机向主机发送数据BUFF

int16 slave_encoder_left;                       //从机左编码器值
int16 slave_encoder_right;                      //从机右编码器值
int16 slave_position;                           //从机转角值

extern int16 validLine;
extern int16 Border[3][High];
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取传感器数据
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_sensor_data(void)
{
    //这里仅仅是提供一个模拟数据
    slave_encoder_left=timer_quad_get(TIMER_2);
    slave_encoder_right=timer_quad_get(TIMER_3);
    timer_quad_clear(TIMER_2);
    timer_quad_clear(TIMER_3);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      根据协议处理数据
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void process_data(void)
{
    temp_buff[0] = 0xD8;                         //帧头
    temp_buff[1] = 0xB0;                         //功能字
    temp_buff[2] = slave_encoder_left>>8;        //数据高8位
    temp_buff[3] = slave_encoder_left&0xFF;      //数据低8位

    temp_buff[4] = 0xB1;                         //功能字
    temp_buff[5] = slave_encoder_right>>8;       //数据高8位
    temp_buff[6] = slave_encoder_right&0xFF;     //数据低8位

    temp_buff[7] = 0xB2;                         //功能字
    temp_buff[8] = slave_position>>8;            //数据高8位
    temp_buff[9] = slave_position&0xFF;          //数据低8位

    temp_buff[10] = 0xEE;                        //帧尾
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      定时器4中断服务函数
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        GPIO_PIN_SET(A0);                           //A0引脚拉高

        get_sensor_data();                          //获取传感器数据。
        process_data();                             //根据协议处理数据，并存入temp_buff中。
        uart_putbuff(UART_3, temp_buff, LINE_LEN);  //通过串口3将数据发送出去。

        GPIO_PIN_RESET(A0);                         //A0引脚拉低
    }
}

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //务必保留，本函数用于初始化MPU 时钟 调试串口

    gpio_init(A0, GPO, 0, GPIO_PIN_CONFIG);                 //同步引脚初始化
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);  //串口3初始化，波特率460800
    timer_pit_interrupt_ms(TIMER_4, 5);                     //定时器4初始化

    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//编码器1初始化，使用定时器2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//编码器2初始化，使用定时器3

    mt9v03x_init();
    ips114_init();
    EnableGlobalIRQ(0);
    uint8 post_image[MT9V03X_H][MT9V03X_W];
    while(1)
    {
        if(mt9v03x_finish_flag==1)
        {
            trackBorder_Get();
            slave_position=centre_line_get();
            for(int i=0;i<MT9V03X_H;i++)
                for(int j=0;j<MT9V03X_W;j++)
                {
                    if(mt9v03x_image[i][j]>100)
                        post_image[i][j]=255;
                    else
                        post_image[i][j]=0;
                }
            ips114_displayimage032_zoom1(mt9v03x_image[0],MT9V03X_W,MT9V03X_H,0,0,MT9V03X_W,MT9V03X_H);
            for(int i=High-1;i>=0;i--)
                post_image[i][Border[CENTRE][i]]=0;
            ips114_displayimage032_zoom1(post_image[0],MT9V03X_W,MT9V03X_H,0,MT9V03X_H+1,MT9V03X_W,MT9V03X_H);
            ips114_showint16(0,7,slave_position);
            ips114_showint16(0,8,validLine);
            mt9v03x_finish_flag=0;
        }
    }
}



