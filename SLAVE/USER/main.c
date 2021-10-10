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
#include "key.h"

#define LINE_LEN                14              //���ݳ���
uint8 temp_buff[LINE_LEN];                      //�ӻ���������������BUFF

int16 slave_encoder_left;                       //�ӻ��������ֵ
int16 slave_encoder_right;                      //�ӻ��ұ�����ֵ
int16 slave_position;                           //�ӻ�ת��ֵ

extern uint8 islost_record[3][50];
extern int8 stateRing_Sign,directRing_Sign;//Բ����ر�־λ
extern int8 stateBranch_Sign,directBranch_Sign;//��·��ر�־λ
extern int8 stateGarage_Sign,directGarage_Sign,is_Out_Garage;//������ر�־λ
extern int16 triangle_index,shortest_col_index,searsh_line_record[Width];

extern int16 Border[3][High],lostLeft_Sign,lostRight_Sign,validLine,searsh_mid_line;
extern int16 left_Delta_Exceed_Cnt,right_Delta_Exceed_Cnt,inflection_C,inflection_A,searsh_mid_line,inflection_B,inflection_frontZebra;
extern int8 is_Out_Garage;

int16 target_StoL_angle=180,target_LtoS_angle=1700;//�Ҳ�·�����ת�Ƕ�,�Ƕ�Ϊ����ֵ
int16 target_StoR_angle=180,target_RtoS_angle=1700;//���·�����ת�Ƕȣ��Ƕ�Ϊ����ֵ
int16 target_Right_InGarage=800,target_Left_InGarage=800;
float rotate_Angle=0,tmp_Angle=0;
int16 left_OutGarage_Start,right_OutGarage_Start;
int16 out_Aux,outGarage_Repair_Start,outGarage_Repair_End;

int16 transinfo=5;//5����գ�90����PWM���0��77������ɲ����44+directBranch_Sign����������ת

int16 Zebra_Last,Zebra_meet_Cnt,Is_meet_Zebra;
int16 should_In_Garage=0;
int16 out_border=80;

uint8 button;
int16 mode=0;
extern int16 in_braking_point, out_braking_point;
extern int16 finish_point;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ��ȡ����������
//  @param      void
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void get_sensor_data(void)
{
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

    temp_buff[10] = 0xB3;
    temp_buff[11] = transinfo>>8;
    temp_buff[12] = transinfo&0xFF;

    temp_buff[13] = 0xEE;                        //֡β
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

        if(stateBranch_Sign==2)
        {
            get_icm20602_gyro_spi();
            tmp_Angle=icm_gyro_z*0.005;
            rotate_Angle=rotate_Angle+tmp_Angle;
            if((directBranch_Sign==LEFT)&&(rotate_Angle>=target_StoL_angle))
            {
                transinfo=33;
                stateBranch_Sign=3;
            }
            else if((directBranch_Sign==RIGHT)&&(rotate_Angle<=-target_StoR_angle))
            {
                transinfo=33;
                stateBranch_Sign=3;
            }
        }
        else if(stateBranch_Sign==5)
        {
            get_icm20602_gyro_spi();
            tmp_Angle=icm_gyro_z*0.005;
            rotate_Angle=rotate_Angle+tmp_Angle;
            if((directBranch_Sign==LEFT)&&(rotate_Angle<=-target_LtoS_angle))
            {
                transinfo=33;
                stateBranch_Sign=0;
            }
            else if((directBranch_Sign==RIGHT)&&(rotate_Angle>=target_RtoS_angle))
            {
                transinfo=33;
                stateBranch_Sign=0;
            }
        }
        else if(transinfo==33)
            transinfo=5;

        if(stateGarage_Sign==1)
        {
            if(directGarage_Sign==LEFT)
            {
                get_icm20602_gyro_spi();
                tmp_Angle=icm_gyro_z*0.005;
                rotate_Angle=rotate_Angle+tmp_Angle;
                if(rotate_Angle<=-target_Left_InGarage)
                {
                    transinfo=33;
                    stateGarage_Sign=0;
                }
            }
            else if(directGarage_Sign==RIGHT)
            {
                get_icm20602_gyro_spi();
                tmp_Angle=icm_gyro_z*0.005;
                rotate_Angle=rotate_Angle+tmp_Angle;
                if(rotate_Angle>=target_Right_InGarage)
                {
                    transinfo=33;
                    stateGarage_Sign=0;
                }
            }
            else if(transinfo==33)
                transinfo=5;
        }

        if(stateGarage_Sign!=0&&stateBranch_Sign!=0 && transinfo==90)
            transinfo=5;

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

    icm20602_init_spi();

    gpio_init(A0, GPO, 0, GPIO_PIN_CONFIG);//ͬ�����ų�ʼ��
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);//����3��ʼ����������460800
    timer_pit_interrupt_ms(TIMER_4, 5);//��ʱ��4��ʼ��

    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//������1��ʼ����ʹ�ö�ʱ��2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//������2��ʼ����ʹ�ö�ʱ��3

    lcd_init();
    mt9v03x_init();
    set_exposure_time(MT9V03X_COF_UART,120);

    Key_Init();

    uint8 threshold=103;
    int16 slave_last=0;
    directGarage_Sign=RIGHT;//���ⷽ��
    directBranch_Sign=RIGHT;
    is_Out_Garage=0;
    Zebra_Last=Zebra_meet_Cnt=0;
    should_In_Garage=0;
    outGarage_Repair_Start=11;
    outGarage_Repair_End=4;
    transinfo=55+directGarage_Sign;//���ó��ⷽ��
    EnableGlobalIRQ(0);

    while(1)
    {
        button=Key_Read();
       switch(button)
       {
       case 1: mode++;
               mode=mode%5;
               break;
       case 2: if(mode==0)
                   out_border++;
               if(mode==1)
                   in_braking_point++;
               if(mode==2)
                   out_braking_point++;
               if(mode==3)
                   finish_point++;
               if(mode==4)
                   directGarage_Sign=LEFT;
               break;
       case 3: if(mode==0)
                   out_border--;
               if(mode==1)
                   in_braking_point--;
               if(mode==2)
                   out_braking_point--;
               if(mode==3)
                   finish_point--;
               if(mode==4)
                   directGarage_Sign=RIGHT;
               break;
       }
       if(mt9v03x_finish_flag==1)
       {
           if(stateBranch_Sign!=2&&stateBranch_Sign!=5)
           {
               trackBorder_Get(threshold);//������ֵ�õ�Ԥ���߽�

               if(is_Out_Garage==0)
               {
                   if(directGarage_Sign==LEFT)
                   {
                       if(validLine<=30&&Border[LEFT][High-1]<80)
                       {
                           is_Out_Garage=1;
                           transinfo=16;
                       }
                   }
                   else if(directGarage_Sign==RIGHT)
                   {
                       if(validLine<=30&&Border[RIGHT][High-1]>out_border)
                       {
                           is_Out_Garage=1;
                           transinfo=16;
                       }
                   }
               }
               else if(is_Out_Garage==1)
               {
                   if(transinfo==16)
                       transinfo=5;

                   /*Is_meet_Zebra=is_Zebra_In();
                   if(Is_meet_Zebra==1)
                       should_In_Garage=1;*/

                   Is_meet_Zebra=is_Zebra_In();
                   if(Is_meet_Zebra==1&&Zebra_Last==0)
                   {
                       Zebra_meet_Cnt++;
                       if(directBranch_Sign==LEFT)
                           directBranch_Sign=RIGHT;
                       else
                           directBranch_Sign=LEFT;
                   }
                   Zebra_Last=Is_meet_Zebra;

                   if(Zebra_meet_Cnt==2)
                       should_In_Garage=1;

                   if(should_In_Garage==0&&Is_meet_Zebra==0)
                   {
                       if(stateBranch_Sign==0)
                       {
                           judge_Ring();
                           repair_Ring(threshold);
                       }
                       if(stateRing_Sign==0)
                       {
                           judge_Branch(threshold);
                       }
                   }
                   else if(should_In_Garage==1)
                   {
                       judge_Garage_In();
                       //repair_Garage_In(threshold);
                   }
               }

               if(Is_meet_Zebra==1&&stateRing_Sign!=0)
                   stateRing_Sign=0;

               slave_position=centre_line_get();//�������ߵõ�ƫ��ֵ

               if((is_Out_Garage==0&&out_Aux>outGarage_Repair_Start)/*||(stateBranch_Sign==1)||(stateBranch_Sign==4)*/||(stateBranch_Sign==5)
                   ||(stateBranch_Sign==2)||(stateBranch_Sign==6))
                   slave_position=0;

                if((slave_position-slave_last>80||slave_position-slave_last<-80)
                    ||((stateRing_Sign==2||stateRing_Sign==3)&&directRing_Sign==RIGHT&&slave_position<0)
                    ||((stateRing_Sign==2||stateRing_Sign==3)&&directRing_Sign==LEFT&&slave_position>0)
                    ||(stateRing_Sign==4&&directRing_Sign==LEFT&&slave_position>0)
                    ||(stateRing_Sign==4&&directRing_Sign==RIGHT&&slave_position<0))
                    slave_position=slave_last;
                else
                    slave_last=slave_position;

                if(Is_meet_Zebra==1)
                    slave_position=0;

                if(validLine>=High-1&&stateBranch_Sign!=2&&stateBranch_Sign!=5&&is_Out_Garage==1)//ͣ��High-5
                    transinfo=90;

                /*if(Zebra_meet_Cnt==2&&validLine>High-5)
                    transinfo=90;*/

              for(int i=0;i<MT9V03X_H;i++)
                  for(int j=0;j<MT9V03X_W;j++)
                  {
                      if(mt9v03x_image[i][j]>threshold)
                          mt9v03x_image[i][j]=255;
                      else
                          mt9v03x_image[i][j]=0;
                  }

              if(inflection_frontZebra!=255)
              {
                  for(int16 i=0;i<Width-2;i++)
                      mt9v03x_image[inflection_frontZebra][i]=0;
              }

               for(int i=High-1;i>=validLine;i--)//validLine,����Ļ����ʾ���߼��߽�
               {
                   mt9v03x_image[i][Border[LEFT][i]]=0;
                   //mt9v03x_image[i][Border[CENTRE][i]]=0;
                   mt9v03x_image[i][Border[RIGHT][i]]=0;
               }


               lcd_displayimage032(mt9v03x_image[0],MT9V03X_W,MT9V03X_H);
               lcd_showint16(0, 4, stateGarage_Sign);
               lcd_showint16(80, 4, stateBranch_Sign);
               lcd_showint16(0, 7, slave_position);
               lcd_showint16(80, 7, validLine);
               switch(mode)
               {
               case 0: lcd_showstr(0, 5, "OG");
                       lcd_showint16(80, 5, out_border);
                       break;
               case 1: lcd_showstr(0, 5, "IB");
                       lcd_showint16(80, 5, in_braking_point);
                       break;
               case 2: lcd_showstr(0, 5, "OB");
                       lcd_showint16(80, 5, out_braking_point);
                       break;
               case 3: lcd_showstr(0, 5, "IG");
                       lcd_showint16(80, 5, finish_point);
                       break;
               case 4: lcd_showstr(0, 5, "GD");
                       lcd_showint16(0, 80, directGarage_Sign);
               }
           }
           mt9v03x_finish_flag=0;
       }
    }
}



