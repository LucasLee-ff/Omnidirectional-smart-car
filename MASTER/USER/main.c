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
#include "Pid.h"
#include "Motor.h"
#include "shangweiji.h"

#define E_START                 0       //׼��״̬
#define E_OK                    1       //�ɹ�
#define E_FRAME_HEADER_ERROR    2       //֡ͷ����
#define E_FRAME_RTAIL_ERROR     3       //֡β����

#define LINE_LEN                14      //���ݳ���
uint8   temp_buff[LINE_LEN];            //�������ڽ������ݵ�BUFF
vuint8  uart_flag;                      //�������ݱ�־λ

int16 slave_encoder_left;               //�ӻ��������ֵ
int16 slave_encoder_right;              //�ӻ��ұ�����ֵ
int16 slave_position;                   //�ӻ�ת��ֵ
int16 master_encoder_left;              //�����������ֵ
int16 master_encoder_right;             //�����ұ�����ֵ

int16 transinfo,transinfo_last=5;                       //������Ϣ

int8 motion_Sign;//
uint8 show_flag;//������ʾ��־λ
int16 slave_position_filter=0,slave_position_filter_Cnt=0;
int16 target_main_speed=100,target_sup_speed=70,target_rotate_speed=80,target_out_speed=70,target_rotate_sup_speed=60;
int16 target_fl_speed,target_fr_speed,target_rl_speed,target_rr_speed;
Pid_Param Pid_fl,Pid_fr,Pid_rl,Pid_rr,Pid_dir_redbull,Pid_dir_crab; //Pid�����ṹ
uint16 servo_Init_Duty=750;
uint16 servo_StoL_duty = 1250,servo_StoR_duty=250;
uint16 servo_toS_duty=750;//���pwm�ź�

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ�ӻ�����
//  @param      data            ��������
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
        //���յ��ĵ�һ���ַ���Ϊ0xD8��֡ͷ����
        if(0xD8 != temp_buff[0])
        {
            uart_num = 0;
            uart_flag = E_FRAME_HEADER_ERROR;
        }
    }

    if(LINE_LEN == uart_num)
    {
        uart_flag = E_OK;
        //���յ����һ���ֽ�Ϊ0xEE
        if(0xEE == temp_buff[LINE_LEN - 1])
        {
            uart_flag = E_OK;
        }
        else    //���յ����һ���ֽڲ���0xEE��֡β����
        {
            uart_flag = E_FRAME_RTAIL_ERROR;
        }
        uart_num = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ����Э��Դӻ����͹��������ݣ��������ݽ���
//  @param      *line                           ���ڽ��յ�������BUFF
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
//  @brief      ����3�жϷ�����
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
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //������ڽ����жϱ�־λ
        dat = USART_ReceiveData(USART3);                //��ȡ��������
        get_slave_data(dat);                            //��ÿһ���ֽڵĴ������ݴ���temp_buff�С�
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ⲿ�ж�0�жϷ�����
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
        while(uart_flag != E_OK);                       //�ȴ���������
        uart_flag = E_START;                            //��ձ�־λ
        data_analysis(temp_buff);                       //���ݽ���

        slave_encoder_right=-slave_encoder_right;
        master_encoder_left=timer_quad_get(TIMER_2);//���������������ֵ
        master_encoder_right=-timer_quad_get(TIMER_3);//���������������ֵ
        timer_quad_clear(TIMER_2);
        timer_quad_clear(TIMER_3);

        if(slave_position_filter_Cnt==0)//��ͨ�˲�
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
        case 0://����״̬
            if(transinfo==16)
                motion_Sign=1;
            break;
        case 1://��ֱ��״̬
            if(transinfo==46)//��Ҫ���Ҳ�·
                motion_Sign=4;
            else if(transinfo==44)//��Ҫ�����·
                motion_Sign=5;
            else if(transinfo==11)//����ת������
                motion_Sign=8;
            else if(transinfo==13)//����ת������
                motion_Sign=9;
            break;
        case 2://�������·�к�����
            if(transinfo==44)//���ֱ��״̬ʱ��Ҫ����ת
                motion_Sign=6;
            break;
        case 3://�����Ҳ�·�к�����
            if(transinfo==46)//���ֱ��״̬ʱ��Ҫ����ת
                motion_Sign=7;
            break;
        case 4://��ֱ��״̬Ҫ�����Ҳ�·����Ҫ����ת
            if(transinfo==33)
                motion_Sign=3;
            break;
        case 5://��ֱ��״̬Ҫ�������·����Ҫ����ת
            if(transinfo==33)
                motion_Sign=2;
            break;
        case 6://�������·�лָ�ֱ��״̬����Ҫ����ת
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 7://�����Ҳ�·�лָ�ֱ��״̬����Ҫ����ת
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 8://����ת������
            if(transinfo==33)
                motion_Sign=1;
            break;
        case 9://����ת������
            if(transinfo==33)
                motion_Sign=1;
        }

        switch(motion_Sign)
        {
        case 0:
            if(transinfo==55)//�����
            {
                target_fl_speed=target_out_speed;
                target_fr_speed=-target_out_speed;
                target_rl_speed=-target_out_speed;
                target_rr_speed=target_out_speed;
            }
            else if(transinfo==57)//�ҳ���
            {
                target_fl_speed=-target_out_speed;
                target_fr_speed=target_out_speed;
                target_rl_speed=target_out_speed;
                target_rr_speed=-target_out_speed;
            }
            break;
        case 1://ֱ��
            PID_posCtrl(&Pid_dir_redbull, slave_position);
            target_fl_speed=target_main_speed+Pid_dir_redbull.out;
            target_fr_speed=target_main_speed-Pid_dir_redbull.out;
            target_rl_speed=target_main_speed+Pid_dir_redbull.out;
            target_rr_speed=target_main_speed-Pid_dir_redbull.out;
            break;
        case 2://���·
            PID_posCtrl(&Pid_dir_crab, slave_position);
            target_fl_speed=-target_sup_speed+Pid_dir_crab.out;
            target_fr_speed=target_sup_speed-Pid_dir_crab.out;
            target_rl_speed=target_sup_speed+Pid_dir_crab.out;
            target_rr_speed=-target_sup_speed-Pid_dir_crab.out;
            break;
        case 3://�Ҳ�·
            PID_posCtrl(&Pid_dir_crab, slave_position);
            target_fl_speed=target_sup_speed+Pid_dir_crab.out;
            target_fr_speed=-target_sup_speed-Pid_dir_crab.out;
            target_rl_speed=-target_sup_speed+Pid_dir_crab.out;
            target_rr_speed=target_sup_speed-Pid_dir_crab.out;
            break;
        case 4://��ֱ��״̬�������Ҳ�·��Ҫ����ת
            pwm_duty(PWM1_CH1_A8, servo_StoR_duty);
            target_fl_speed=-target_rotate_speed;
            target_fr_speed=target_rotate_speed;
            target_rl_speed=-target_rotate_speed;
            target_rr_speed=target_rotate_speed;
            break;
        case 6://�������·�У���Ϊֱ����Ҫ����ת
            pwm_duty(PWM1_CH1_A8, servo_toS_duty);
            target_fl_speed=-target_rotate_sup_speed;
            target_fr_speed=target_rotate_sup_speed;
            target_rl_speed=-target_rotate_sup_speed;
            target_rr_speed=target_rotate_sup_speed;
            break;
        case 5://��ֱ��״̬���������·��Ҫ����ת
            pwm_duty(PWM1_CH1_A8, servo_StoL_duty);
            target_fl_speed=target_rotate_speed;
            target_fr_speed=-target_rotate_speed;
            target_rl_speed=target_rotate_speed;
            target_rr_speed=-target_rotate_speed;
            break;
        case 7://�����Ҳ�·�У���Ϊֱ����Ҫ����ת
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

        if(transinfo==90)//����ɲ��
            target_fl_speed=target_fr_speed=target_rl_speed=target_rr_speed=0;

        PID_incCtrl(&Pid_fl,(float)(target_fl_speed-slave_encoder_left));
        PID_incCtrl(&Pid_fr,(float)(target_fr_speed-slave_encoder_right));
        PID_incCtrl(&Pid_rl,(float)(target_rl_speed-master_encoder_left));
        PID_incCtrl(&Pid_rr,(float)(target_rr_speed-master_encoder_right));

        Duty_All((int32)Pid_fl.out,(int32)Pid_fr.out,(int32)Pid_rl.out,(int32)Pid_rr.out);

        show_flag = 1;                                  //��Ļ��ʾ��־λ
    }
}

void Encoder_Init_Master()//������������ʼ������
{
    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//������3��ʼ����ʹ�ö�ʱ��2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//������4��ʼ����ʹ�ö�ʱ��3
    return;
}

int main(void)
{
    DisableGlobalIRQ();
    board_init();           //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���

    //���ڵ���ռ���ȼ�һ��Ҫ���ⲿ�жϵ���ռ���ȼ��ߣ���������ʵʱ���մӻ�����
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);  //����3��ʼ����������460800
    uart_rx_irq(UART_3, ENABLE);                            //Ĭ����ռ���ȼ�0 �����ȼ�0��
    gpio_interrupt_init(A0, RISING, GPIO_INT_CONFIG);       //A0��ʼ��ΪGPIO �����ش���
    nvic_init(EXTI0_IRQn, 1, 1, ENABLE);     //EXTI0���ȼ����ã���ռ���ȼ�1�������ȼ�1

    pwm_init(PWM1_CH1_A8, 50, 750);  //�����ʼ�����м�λ��
    motion_Sign=0;//�˶���ʽ����
    PID_Init(&Pid_fl,0);
    PID_Init(&Pid_fr,1);
    PID_Init(&Pid_rl,2);
    PID_Init(&Pid_rr,3);
    PID_dir_Init(&Pid_dir_redbull,0);
    PID_dir_Init(&Pid_dir_crab,1);
    Duty_Init();
    Encoder_Init_Master();
    //���ж������
    EnableGlobalIRQ(0);

    while(1)
    {
        if(show_flag==1)
        {
            show_flag=0;
        }
    }
}
