/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file            main
 * @company         �ɶ���ɿƼ����޹�˾
 * @author          ��ɿƼ�(QQ790875685)
 * @version         �鿴doc��version�ļ� �汾˵��
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/
//�����Ƽ�IO�鿴Projecct�ļ����µ�TXT�ı�

//���µĹ��̻��߹����ƶ���λ�����ִ�����²���
//�Ҽ��������̣�ѡ��ˢ��

#include "headfile.h"
#include "TrackGet.h"

#define LINE_LEN                11              //���ݳ���
uint8 temp_buff[LINE_LEN];                      //�ӻ���������������BUFF

int16 slave_encoder_left;                       //�ӻ��������ֵ
int16 slave_encoder_right;                      //�ӻ��ұ�����ֵ
int16 slave_position;                           //�ӻ�ת��ֵ

extern uint8 Border[3][High];
extern uint8 validLine;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����������
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_sensor_data(void)
{
    //����������ṩһ��ģ������
    slave_encoder_left=timer_quad_get(TIMER_2);
    slave_encoder_right=timer_quad_get(TIMER_3);
    timer_quad_clear(TIMER_2);
    timer_quad_clear(TIMER_3);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Э�鴦������
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void process_data(void)
{
    temp_buff[0] = 0xD8;                         //֡ͷ
    temp_buff[1] = 0xB0;                         //������
    temp_buff[2] = slave_encoder_left>>8;        //���ݸ�8λ
    temp_buff[3] = slave_encoder_left&0xFF;      //���ݵ�8λ

    temp_buff[4] = 0xB1;                         //������
    temp_buff[5] = slave_encoder_right>>8;       //���ݸ�8λ
    temp_buff[6] = slave_encoder_right&0xFF;     //���ݵ�8λ

    temp_buff[7] = 0xB2;                         //������
    temp_buff[8] = slave_position>>8;            //���ݸ�8λ
    temp_buff[9] = slave_position&0xFF;          //���ݵ�8λ

    temp_buff[10] = 0xEE;                        //֡β
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ʱ��4�жϷ�����
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
        GPIO_PIN_SET(A0);                           //A0��������

        get_sensor_data();                          //��ȡ���������ݡ�
        process_data();                             //����Э�鴦�����ݣ�������temp_buff�С�
        uart_putbuff(UART_3, temp_buff, LINE_LEN);  //ͨ������3�����ݷ��ͳ�ȥ��

        GPIO_PIN_RESET(A0);                         //A0��������
    }
}

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���

    ips114_init();
    mt9v03x_init();

    gpio_init(A0, GPO, 0, GPIO_PIN_CONFIG);                 //ͬ�����ų�ʼ��
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);  //����3��ʼ����������460800
    timer_pit_interrupt_ms(TIMER_4, 5);                     //��ʱ��4��ʼ��

    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//������1��ʼ����ʹ�ö�ʱ��2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//������2��ʼ����ʹ�ö�ʱ��3

    //systick_delay_ms(200);
    //ips114_showstr(0,0,"test");
    EnableGlobalIRQ(0);
    uint8 post_image[MT9V03X_H][MT9V03X_W];
    uint8 threshold,threshold_Last;
    uint8 count=0;
    int16 slave_last;
    uint8 cnt=0;
    float k;
    while(1)
    {
        if(mt9v03x_finish_flag==1)
       {
           //threshold=OTSU(mt9v03x_image[0]);
           trackBorder_Get(100);
           slave_position=centre_line_get();
           k=Regression(Border[0]);
           if(cnt==0)
           {
               slave_last=slave_position;
               cnt=1;
           }
           else
           {
               if(slave_position-slave_last>50||slave_position-slave_last<-50)
                   slave_position=slave_last;
               else
                   slave_last=slave_position;
           }

           for(int i=0;i<MT9V03X_H;i++)
               for(int j=0;j<MT9V03X_W;j++)
               {
                   if(mt9v03x_image[i][j]>100)
                       post_image[i][j]=255;
                   else
                       post_image[i][j]=0;
               }
           for(int i=High-1;i>=validLine;i--)//validLine
               post_image[i][Border[CENTRE][i]]=0;
           ips114_displayimage032(post_image[0],MT9V03X_W,MT9V03X_H);
           ips114_showint16(0,5,slave_position);
           ips114_showuint8(80,5,threshold);

           mt9v03x_finish_flag=0;
       }
    }
}



