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
#include "key.h"

#define LINE_LEN                14              //数据长度
uint8 temp_buff[LINE_LEN];                      //从机向主机发送数据BUFF

int16 slave_encoder_left;                       //从机左编码器值
int16 slave_encoder_right;                      //从机右编码器值
int16 slave_position;                           //从机转角值

extern uint8 islost_record[3][50];
extern int8 stateRing_Sign,directRing_Sign;//圆环相关标志位
extern int8 stateBranch_Sign,directBranch_Sign;//岔路相关标志位
extern int8 stateGarage_Sign,directGarage_Sign,is_Out_Garage;//车库相关标志位
extern int16 triangle_index,shortest_col_index,searsh_line_record[Width];

extern int16 Border[3][High],lostLeft_Sign,lostRight_Sign,validLine,searsh_mid_line;
extern int16 left_Delta_Exceed_Cnt,right_Delta_Exceed_Cnt,inflection_C,inflection_A,searsh_mid_line,inflection_B,inflection_frontZebra;
extern int8 is_Out_Garage;

int16 target_StoL_angle=180,target_LtoS_angle=1700;//右岔路相关旋转角度,角度为绝对值
int16 target_StoR_angle=180,target_RtoS_angle=1700;//左岔路相关旋转角度，角度为绝对值
int16 target_Right_InGarage=800,target_Left_InGarage=800;
float rotate_Angle=0,tmp_Angle=0;
int16 left_OutGarage_Start,right_OutGarage_Start;
int16 out_Aux,outGarage_Repair_Start,outGarage_Repair_End;

int16 transinfo=5;//5代表空，90代表PWM输出0，77代表车库刹车，44+directBranch_Sign代表三岔旋转

int16 Zebra_Last,Zebra_meet_Cnt,Is_meet_Zebra;
int16 should_In_Garage=0;
int16 out_border=80;

uint8 button;
int16 mode=0;
extern int16 in_braking_point, out_braking_point;
extern int16 finish_point;
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获取传感器数据
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

    temp_buff[10] = 0xB3;
    temp_buff[11] = transinfo>>8;
    temp_buff[12] = transinfo&0xFF;

    temp_buff[13] = 0xEE;                        //帧尾
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

    icm20602_init_spi();

    gpio_init(A0, GPO, 0, GPIO_PIN_CONFIG);//同步引脚初始化
    uart_init(UART_3, 460800, UART3_TX_B10, UART3_RX_B11);//串口3初始化，波特率460800
    timer_pit_interrupt_ms(TIMER_4, 5);//定时器4初始化

    timer_quad_init(TIMER_2,TIMER2_CHA_A15,TIMER2_CHB_B3);//编码器1初始化，使用定时器2
    timer_quad_init(TIMER_3,TIMER3_CHA_B4,TIMER3_CHB_B5);//编码器2初始化，使用定时器3

    lcd_init();
    mt9v03x_init();
    set_exposure_time(MT9V03X_COF_UART,120);

    Key_Init();

    uint8 threshold=103;
    int16 slave_last=0;
    directGarage_Sign=RIGHT;//车库方向
    directBranch_Sign=RIGHT;
    is_Out_Garage=0;
    Zebra_Last=Zebra_meet_Cnt=0;
    should_In_Garage=0;
    outGarage_Repair_Start=11;
    outGarage_Repair_End=4;
    transinfo=55+directGarage_Sign;//设置出库方向
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
               trackBorder_Get(threshold);//根据阈值得到预备边界

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

               slave_position=centre_line_get();//根据中线得到偏角值

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

                if(validLine>=High-1&&stateBranch_Sign!=2&&stateBranch_Sign!=5&&is_Out_Garage==1)//停车High-5
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

               for(int i=High-1;i>=validLine;i--)//validLine,在屏幕上显示中线及边界
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



