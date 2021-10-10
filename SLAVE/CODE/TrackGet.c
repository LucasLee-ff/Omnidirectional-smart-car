/*
 * TrackGet.c
 *
 *  Created on: 2021��2��7��
 *      Author: 16917
 */
#include "headfile.h"
#include "TrackGet.h"

int16 delta=1;//ÿ������ɨ�����
int16 EdgeL,EdgeR;//��¼���ұ߽�
int16 validLine,searsh_mid_line,searsh_line_record[Width];//��Чɨ���м���
int16 triangle_index;
int16 shortest_col_index;
int16 leftLostLine_Cnt,rightLostLine_Cnt;//���߱�־
int8 stateRing_Sign=0,directRing_Sign=-1;//Բ����ر�־λ
int8 stateBranch_Sign=0,directBranch_Sign=0;//��·��ر�־λ
int8 stateGarage_Sign=0,directGarage_Sign=-1,is_Out_Garage = 0;//������ر�־λ

int16 inflection_frontZebra,inflection_rearZebra;//����յ�
int16 inflection_A,inflection_B,inflection_C;//Բ���յ㣬�ɽ���Զ���������յ�
int16 repair_Cnt;//����ѭ��������
int16 track_Width[High];//��¼ɨ��߽�õ����������
int16 garage_Record[High], index_Garage;//�а����ߵ���
int16 Border[3][High];//����������
uint8 islost_record[3][High];//ĳ�������Ҳ��Ƿ�������
int16 default_branchjudge_line=25;//��·ʶ����λ
int16 left_Delta_Exceed_Cnt,right_Delta_Exceed_Cnt;//Բ��ʶ����λ
int16 inflection_A_last=30,inflection_C_last=-1;
extern int16 transinfo;
extern uint8  mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern float rotate_Angle;
extern int16 left_OutGarage_Start,right_OutGarage_Start;
extern int16 Zebra_meet_Cnt,out_Aux;

int16 in_braking_point=33, out_braking_point=46;
int16 finish_point=37;

int16 index_t=High-1;

int16 direct_Weight[High]=
{
        105, 109, 110, 109, 108,
        105, 105, 100, 100, 100,
        74, 73, 72, 72, 71,
        70, 69, 68, 66, 65,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        /*
        105, 106, 108, 109, 110,//+15
        118, 118, 118, 118, 117,//+18
        86, 85, 84, 84, 83,//-13
        82, 81, 80, 78, 77,
        75, 74, 72, 70, 68,
        66, 64, 62, 60, 57,
        55, 52, 49, 47, 44,
        41, 37, 34, 31, 27,
        24, 20, 16, 12, 8,
        4, 0, 0, 0, 0*/
};
/*  110/120/130
    105, 106, 108, 109, 110,//+15
    118, 118, 118, 118, 117,//+18
    86, 85, 84, 84, 83,//-13
    82, 81, 80, 78, 77,
    75, 74, 72, 70, 68,
    66, 64, 62, 60, 57,
    55, 52, 49, 47, 44,
    41, 37, 34, 31, 27,
    24, 20, 16, 12, 8,
    4, 0, 0, 0, 0

    140
    102, 103, 105, 106, 107,//+12
    118, 118, 118, 118, 117,//+18
    90, 89, 88, 88, 87,//-9
    86, 85, 84, 82, 81,
    75, 74, 72, 70, 68,//-13
    66, 64, 62, 60, 57,
    55, 52, 49, 47, 44,
    41, 37, 34, 31, 27,
    24, 20, 16, 12, 8,
    4, 0, 0, 0, 0
 */

int16 direct_Weight_crab[High]=
{
        12, 12, 11, 0, 0,//111, 105,//114, 114, 113, 113, 110,
        0, 0, 0, 0, 0,//68, 68, 67, 61, 61,
        0, 0, 0, 0, 0,//38, 37, 36, 35, 34,
        0, 0, 0, 0, 0,//23, 22, 20, 19, 18,
        0, 0, 0, 0, 0,//14, 12, 11, 9, 7,
        0, 0, 0, 0, 0,//4, 2, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0
};

uint8 OTSU(uint8 *pre_image)//��̬������ֵ
{
    uint16 piexlCount[256];
    uint8 threshold_Max=200,threshold_Min=70;

    for(int i=0;i<256;i++)//��ʼ������
        piexlCount[i]=0;

    for(int i=0;i<High;i++)//ͳ��ÿ���Ҷ��ж��ٸ�����
        for(int j=0;j<Width;j++)
        {
            int tmp=*(pre_image+i*Width+j);
            piexlCount[tmp]++;
        }

    uint8 threshold;//��ֵ
    int32 deltaMax=0,deltaTmp;//��䷽��
    for(int i=threshold_Min;i<threshold_Max;i++)//Ѱ��ʹ��䷽��������ֵ
    {
        int32 N0,N1,U0,U1,U0tmp,U1tmp;
        N0=N1=U0=U1=U0tmp=U1tmp=0;
        for(int j=0;j<256;j++)
        {
            if(j<=i)//�ϰ�����
            {
                N0=N0+piexlCount[j];
                U0tmp=U0tmp+j*piexlCount[j];
            }
            else//��������
            {
                N1=N1+piexlCount[j];
                U1tmp=U1tmp+j*piexlCount[j];
            }
        }
        U0=U0tmp/N0;
        U1=U1tmp/N1;
        deltaTmp=N0*N1*(U0-U1)*(U0-U1);
        if(deltaTmp>deltaMax)
        {
            deltaMax=deltaTmp;
            threshold=i;
        }
    }
    return threshold;
}

void trackBorder_Get(uint8 threshold)//Ԥ����߽�
{
    left_Delta_Exceed_Cnt=right_Delta_Exceed_Cnt=0;
    leftLostLine_Cnt=0;//����������ʼ��
    rightLostLine_Cnt=0;
    validLine=High-1;
    index_Garage=0;

    int d_neg=-40,d_pos=40;//-25 25
    if(stateRing_Sign>=4&&stateRing_Sign<=5&&directRing_Sign==LEFT)
        d_neg=-90;
    else if(stateRing_Sign>=4&&stateRing_Sign<=5&&directRing_Sign==RIGHT)
        d_pos=90;

    if(stateRing_Sign==4&&directRing_Sign==LEFT)
        d_pos=0;
    if(stateRing_Sign==4&&directRing_Sign==RIGHT)
        d_neg=0;

    shortest_col_index = 70;

    for(int i=Width/2+d_neg;i<=Width/2+d_pos;i++)//��ȡԤ����Զ��Ч�е��������䳤��
    {
        int j=High-1;
        while(j>0&&mt9v03x_image[j][i]>threshold)
            j--;
        searsh_line_record[i]=j;//��¼ĳ�е���Զ����
        if(i>70&&i<=118)
        {
            if(searsh_line_record[i] > searsh_line_record[shortest_col_index])//��������±�
                shortest_col_index = i;
        }

        if(j<validLine)
        {
            validLine=j;
            searsh_mid_line=i;
        }
    }

    for(int i=High-1;i>=validLine;i--)//ɨ�����ұ��߲���������
    {
        for(int j=searsh_mid_line-delta;j>=0;j=j-delta)//ɨ����߽�
        {
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeL=j;
                islost_record[LEFT][i]=0;
                break;
            }
            else if(j==0)//δ�ҵ������
            {
                EdgeL=0;
                islost_record[LEFT][i]=1;
                leftLostLine_Cnt++;
                break;
            }
        }
        for(int j=searsh_mid_line+delta;j<Width-1;j=j+delta)//ɨ���ұ߽�
        {
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeR=j;
                islost_record[RIGHT][i]=0;
                break;
            }
            else if(j==Width-2)//δ�ҵ������
            {
                EdgeR=Width-2;
                islost_record[RIGHT][i]=1;
                rightLostLine_Cnt++;
                break;
            }
        }

        Border[LEFT][i]=EdgeL;//��߽�
        Border[RIGHT][i]=EdgeR;//�ұ߽�
        Border[CENTRE][i]=(EdgeL+EdgeR)>>1;//��ʱ����

        track_Width[i]=Border[RIGHT][i]-Border[LEFT][i];

        if(track_Width[i]<10)
        {
            garage_Record[index_Garage++]=i;

            if(searsh_mid_line-delta-30>0)
                for(int j=searsh_mid_line-delta-30;j>=0;j=j-delta)//ɨ����߽�
                {
                    if(mt9v03x_image[i][j]<threshold)
                    {
                        EdgeL=j;
                        islost_record[LEFT][i]=0;
                        break;
                    }
                    else if(j==0)//δ�ҵ������
                    {
                        EdgeL=0;
                        islost_record[LEFT][i]=1;
                        leftLostLine_Cnt++;
                        break;
                    }
                }
            if(searsh_mid_line+delta+30<Width-1)
                for(int j=searsh_mid_line+delta+30;j<Width-1;j=j+delta)//ɨ���ұ߽�
                {
                    if(mt9v03x_image[i][j]<threshold)
                    {
                        EdgeR=j;
                        islost_record[RIGHT][i]=0;
                        break;
                    }
                    else if(j==Width-2)//δ�ҵ������
                    {
                        EdgeR=Width-2;
                        islost_record[RIGHT][i]=1;
                        rightLostLine_Cnt++;
                        break;
                    }
                }
            Border[LEFT][i]=EdgeL;//��߽�
            Border[RIGHT][i]=EdgeR;//�ұ߽�
            Border[CENTRE][i]=(EdgeL+EdgeR)>>1;//��ʱ����
        }

        if(i<High-1)
        {
            int16 tmp=Border[LEFT][i]-Border[LEFT][i+1];
            if(tmp>22||tmp<-22)//22
                left_Delta_Exceed_Cnt++;
            tmp=Border[RIGHT][i]-Border[RIGHT][i+1];
            if(tmp>22||tmp<-22)
                right_Delta_Exceed_Cnt++;
        }
    }
}

uint8 is_Straight(uint8 start_Index,uint8 end_Index,uint8 side)//�ж�ĳһ�ߴ���㵽�յ��Ƿ�Ϊֱ��
{
    int white_Cnt=0;
    if(start_Index==end_Index)
        return 1;
    else
    {
        uint8 start=start_Index<end_Index?start_Index:end_Index;
        uint8 end=start_Index>end_Index?start_Index:end_Index;
        if(start<validLine)
            return 0;
        int8 delta=Border[side][start]-Border[side][start+1];
        for(uint8 i=start+1;i<end;i++)
        {
            int8 tmp=Border[side][i]-Border[side][i+1];
            if(Border[side][i]==0||Border[side][i]==Width-2)
                white_Cnt++;
            if(white_Cnt>10)
                return 0;
            if(tmp-delta>2||tmp-delta<-2)
                return 0;
        }
        return 1;
    }
}

uint8 find_Inflection_C(int16 start,int16 end,int dir,int sign)//Զ���յ�
{
    if(dir==LEFT)
    {
        if(sign==0)
        {
            for(int16 i=start;i<end;i++)
            {
                if(Border[LEFT][i]-Border[LEFT][i+1]>25)
                {

                        inflection_C=i;
                        return 1;
                }
            }
        }
        else if(sign==1)
        {
            for(int16 i=end-3;i>start;i--)
                if(i-2>=start
                   &&Border[LEFT][i]>5
                   &&Border[LEFT][i]-Border[LEFT][i+1]>3
                   &&Border[LEFT][i]-Border[LEFT][i+2]>3
                   &&Border[LEFT][i]-Border[LEFT][i+3]>3
                   &&Border[LEFT][i]-Border[LEFT][i-1]<=2
                   &&Border[LEFT][i]-Border[LEFT][i-1]>=-2
                   &&Border[LEFT][i-1]-Border[LEFT][i-2]<=2
                   &&Border[LEFT][i-1]-Border[LEFT][i-2]>=-2)//25
                {
                    inflection_C=i;
                        return 1;
                }
        }
    }
    else
    {
        if(sign==0)
        {
            for(int16 i=start;i<end;i++)
            {
                if(Border[RIGHT][i]-Border[RIGHT][i+1]<-25)//-30
                {
                    inflection_C=i;
                        return 1;
                }
            }
        }
        else if(sign==1)
        {
            for(int16 i=end-3;i>start;i--)
                if(i-2>=start
                   &&Border[RIGHT][i]!=Width-5
                   &&Border[RIGHT][i]-Border[RIGHT][i+1]<-3
                   &&Border[RIGHT][i]-Border[RIGHT][i+2]<-3
                   &&Border[RIGHT][i]-Border[RIGHT][i+3]<-3
                   &&Border[RIGHT][i]-Border[RIGHT][i-1]<=2
                   &&Border[RIGHT][i]-Border[RIGHT][i-1]>=-2
                   &&Border[RIGHT][i-1]-Border[RIGHT][i-2]<=2
                   &&Border[RIGHT][i-1]-Border[RIGHT][i-2]>=-2)
                {
                    inflection_C=i;
                    return 1;
                }
        }
    }
    inflection_C=255;
    return 0;
}

uint8 find_Inflection_B(int16 start,int16 end,int dir)//�м�յ�
{
    if(dir==LEFT)
    {
        for(int16 i=end-2;i>start+2;i--)
            if(Border[LEFT][i]>30
               &&Border[LEFT][i]-Border[LEFT][i-2]>=0
               &&Border[LEFT][i]-Border[LEFT][i+2]>0)
            {
                inflection_B=i;
                return 1;
            }
    }
    else
    {
        for(int16 i=end-2;i>start+2;i--)
            if(Border[RIGHT][i]<Width-30
               &&Border[RIGHT][i]-Border[RIGHT][i-2]<=0
               &&Border[RIGHT][i]-Border[RIGHT][i+2]<0)
            {
                inflection_B=i;
                return 1;
            }
    }
    inflection_B=255;
    return 0;
}

uint8 find_Inflection_A(int16 start,int16 end,int dir,int sign)//�����յ�
{
    if(dir==LEFT)
    {
        if(sign==0)//������A��
        {
            for(int16 i=end;i>start+3;i--)
                if(Border[LEFT][i]-Border[LEFT][i-3]>15)
                {
                    inflection_A=i;
                    return 1;
                }
        }
        else if(sign==1)//������A��
        {
            for(int16 i=end-4;i>start+4;i--)
                if((Border[RIGHT][i]-Border[RIGHT][i-2]<0)
                  &&(Border[RIGHT][i]-Border[RIGHT][i-4]<0)
                  &&(Border[RIGHT][i]-Border[RIGHT][i+2]<0)
                  &&(Border[RIGHT][i]-Border[RIGHT][i+4]<0))
                {
                    inflection_A=i;
                    return 1;
                }
        }
    }
    else
    {
        if(sign==0)//������A��
        {
            for(int16 i=end;i>start+3;i--)
                if(Border[RIGHT][i]-Border[RIGHT][i-3]<-20)
                {
                    inflection_A=i;
                    return 1;
                }
        }
        else if(sign==1)//������A��
        {
            for(int16 i=end-4;i>start+4;i--)
                if((Border[LEFT][i]-Border[LEFT][i-2]>0)
                  &&(Border[LEFT][i]-Border[LEFT][i-4]>0)
                  &&(Border[LEFT][i]-Border[LEFT][i+2]>0)
                  &&(Border[LEFT][i]-Border[LEFT][i+4]>0))
                {
                    inflection_A=i;
                    return 1;
                }
        }

    }
    inflection_A=255;
    return 0;
}

int16 centre_line_get()//��ȡ����ƫ��ֵ
{
    int32 sum_A=0,sum_B=0;
    //int16 cnt=0;

    Border[CENTRE][High-1]=(Border[LEFT][High-1]+Border[RIGHT][High-1])>>1;

    int16 end=validLine;
    if(stateRing_Sign==3&&inflection_C!=255)
        end=inflection_C;

    if(stateGarage_Sign==2)
        end=inflection_frontZebra;

    if(stateBranch_Sign==1||stateBranch_Sign==4)
        end=index_t;

    for(int i=High-2;i>=end;i--)
    {
        Border[CENTRE][i]=(Border[LEFT][i]+Border[RIGHT][i])>>1;

        if(stateBranch_Sign==3)
        {
            sum_A=sum_A+direct_Weight_crab[High-1-i]*(Border[CENTRE][i]-Width/2);
            sum_B=sum_B+direct_Weight_crab[High-1-i];
        }
        else
        {
            sum_A=sum_A+direct_Weight[High-1-i]*(Border[CENTRE][i]-Width/2);
            sum_B=sum_B+direct_Weight[High-1-i];
        }
    }
    return sum_A/sum_B;
}

void judge_Ring()//ʶ��Բ�����ж�����״̬
{
    switch(stateRing_Sign)
    {
    case 0://��ʼ״̬������Ƿ�����Բ��
        if(left_Delta_Exceed_Cnt>=3&&left_Delta_Exceed_Cnt<=4&&right_Delta_Exceed_Cnt<=1
            &&is_Straight(High-1, 15, RIGHT)==1
            &&find_Inflection_A(validLine<20?20:validLine, High-1, LEFT, 0)==1
            &&inflection_A>25)//��������Բ��
        {
                stateRing_Sign=1;
                directRing_Sign=LEFT;
        }

        else if(left_Delta_Exceed_Cnt<=1&&right_Delta_Exceed_Cnt>=3&&right_Delta_Exceed_Cnt<=4
            &&is_Straight(High-1, 15, LEFT)==1
            &&find_Inflection_A(validLine<20?20:validLine, High-1, RIGHT, 0)==1
            &&inflection_A>25)//��������Բ��
        {
                stateRing_Sign=1;
                directRing_Sign=RIGHT;
        }
        break;
    case 1:
        if((find_Inflection_A(validLine<13?13:validLine, High-1, directRing_Sign, 0)==0))
            stateRing_Sign=2;
        break;
    case 2://
        if((find_Inflection_C(validLine<6?6:validLine, High-1, directRing_Sign,0)==1)&&inflection_C>=12)
        {
            stateRing_Sign=3;
            inflection_C_last=inflection_C;
        }
        break;
    case 3://����2����״̬����ⲻ��C�յ�ʱ������һ״̬
        if(find_Inflection_C(validLine<4?4:validLine, High-1, directRing_Sign,0)==0
           ||(inflection_C!=255&&inflection_C>40))//42
            stateRing_Sign=4;
        break;
    case 4://����״̬����⵽A�յ�ʱ����ó�������
        if(/*validLine<=18&&*/find_Inflection_A(validLine, High-1, directRing_Sign,1)==1/*&&inflection_A>20
            &&inflection_A<40*/)
            stateRing_Sign=5;
        break;
    case 5://����3����״̬���ܼ�⵽C�յ�����ѳ���
        if((validLine<13)&&(find_Inflection_C(validLine<6?6:validLine, High-1, directRing_Sign,1)==1))
            stateRing_Sign=6;
        break;
    case 6://����4����״̬����������������ж������ߴ����뿪Բ��||��ⲻ��C�յ�
        if(find_Inflection_C(validLine, High-1, directRing_Sign,1)==0)
        {
            if(directRing_Sign==LEFT&&is_Straight(High-1, 25, RIGHT)==1)//
            {
                stateRing_Sign=0;//
                inflection_C_last=-1;
            }
            else if(directRing_Sign==RIGHT&&is_Straight(High-1, 25, LEFT)==1)//
            {
                stateRing_Sign=0;//
                inflection_C_last=-1;
            }
        }
        break;
    }
    return;
}

void repair_Ring(uint8 threshold)//Բ������
{
    switch(stateRing_Sign)
    {

    case 1://����1����
        if(directRing_Sign==LEFT)//�󻷲���
        {
            int16 i=inflection_A-3;
            repair_Cnt=3;
            int16 tmp=Border[LEFT][inflection_A]+0.8*repair_Cnt;
            while(i>0&&tmp>Border[LEFT][i])
            {
                Border[LEFT][i]=tmp;
                repair_Cnt++;
                i--;
                tmp=Border[LEFT][inflection_A]+0.8*repair_Cnt;
            }
        }
        else if(directRing_Sign==RIGHT)//�һ�����
        {
            int16 i=inflection_A-3;
            repair_Cnt=3;
            int16 tmp=Border[RIGHT][inflection_A]-0.8*repair_Cnt;
            while(i>0&&tmp<Border[RIGHT][i])
            {
                Border[RIGHT][i]=tmp;
                repair_Cnt++;
                i--;
                tmp=Border[RIGHT][inflection_A]-0.8*repair_Cnt;
            }
        }
        break;
    case 2:
        if(find_Inflection_B(validLine, High-1, directRing_Sign)==1)
        {
            if(directRing_Sign==LEFT)
            {
                repair_Cnt=1;
                int16 i=inflection_B+1;
                int16 tmp=Border[LEFT][inflection_B]-0.9*repair_Cnt;
                while(i<High)
                {
                    if(tmp<0)
                        tmp=0;
                    Border[LEFT][i]=tmp;
                    i++;
                    repair_Cnt++;
                    tmp=Border[LEFT][inflection_B]-0.9*repair_Cnt;
                }
            }
            else if(directRing_Sign==RIGHT)
            {
                repair_Cnt=1;
                int16 i=inflection_B+1;
                int16 tmp=Border[RIGHT][inflection_B]+0.9*repair_Cnt;
                while(i<High)
                {
                    if(tmp>Width-2)
                        tmp=Width-2;
                    Border[RIGHT][i]=tmp;
                    i++;
                    repair_Cnt++;
                    tmp=Border[RIGHT][inflection_B]+0.9*repair_Cnt;
                }
            }
        }
        break;
    case 3://����2����
        if(directRing_Sign==LEFT)//�󻷲���
        {
            int16 i=inflection_C+1;
            repair_Cnt=1;
            int16 tmp=Border[LEFT][inflection_C]+2.3*repair_Cnt;//2.2

            while(i<High&&tmp<Border[RIGHT][i])//��C�յ����
            {
               Border[RIGHT][i]=tmp;
               repair_Cnt++;
               i++;
               tmp=Border[LEFT][inflection_C]+2.3*repair_Cnt;//ϵ������
            }

            repair_Cnt=1;
            tmp=Border[LEFT][inflection_C]-1.1*repair_Cnt;
            i=inflection_C+1;
            while(i<High&&tmp>Border[LEFT][i])//Ѱ��B��
            {
                i++;
                repair_Cnt++;
                tmp=Border[LEFT][inflection_C]-1.1*repair_Cnt;
            }

            if(i-inflection_C>4)
                while(i<High)//��B�����
                {
                    Border[LEFT][i]=tmp;
                    i++;
                    repair_Cnt++;
                    tmp=Border[LEFT][inflection_C]-0.9*repair_Cnt;
                }
        }
        else if(directRing_Sign==RIGHT)//�һ�����
        {
            int16 i=inflection_C+1;
            repair_Cnt=1;
            int16 tmp=Border[RIGHT][inflection_C]-2.3*repair_Cnt;

            while(i<High&&tmp>Border[LEFT][i])//��C�յ����
            {
               Border[LEFT][i]=tmp;
               repair_Cnt++;
               i++;
               tmp=Border[RIGHT][inflection_C]-2.3*repair_Cnt;//ϵ������
            }

            repair_Cnt=1;
            tmp=Border[RIGHT][inflection_C]+1.1*repair_Cnt;
            i=inflection_C+1;
            while(i<High&&tmp<Border[RIGHT][i])//Ѱ��B��
            {
                i++;
                repair_Cnt++;
                tmp=Border[RIGHT][inflection_C]+1.1*repair_Cnt;
            }


            if(i-inflection_C>-4)
                while(i<High)//��B�����
                {
                    Border[RIGHT][i]=tmp;
                    i++;
                    repair_Cnt++;
                    tmp=Border[RIGHT][inflection_C]+0.9*repair_Cnt;
                }
        }
        break;
    case 5:
        if(find_Inflection_A(validLine, High-1, directRing_Sign, 1)==1)
        {
            if(directRing_Sign==LEFT)
            {
                repair_Cnt=1;
                for(int i=inflection_A-1;i>validLine;i--)
                {
                    int tmp=Border[RIGHT][inflection_A]-5*repair_Cnt;
                    if(tmp>Border[LEFT][i])
                        Border[RIGHT][i]=tmp;
                    repair_Cnt++;
                }
            }
            else
            {
                repair_Cnt=1;
                for(int i=inflection_A-1;i>validLine;i--)
                {
                    int tmp=Border[LEFT][inflection_A]+5*repair_Cnt;
                    if(tmp<Border[RIGHT][i])
                        Border[LEFT][i]=tmp;
                    repair_Cnt++;
                }
            }
        }
        break;
    case 6://����4����
        if(directRing_Sign==LEFT)
        {
            if(inflection_C!=255)
            {
                repair_Cnt=1;
                for(int i=inflection_C+1;i<High;i++)
                {
                    int16 tmp=Border[LEFT][inflection_C]-0.4*repair_Cnt;
                    if(tmp<0)
                        tmp=0;
                    Border[LEFT][i]=tmp;
                    repair_Cnt++;
                }
            }
        }
        else
        {
            if(inflection_C!=255)
            {
                repair_Cnt=1;
                for(int i=inflection_C+1;i<High;i++)
                {
                    int16 tmp=Border[RIGHT][inflection_C]+0.4*repair_Cnt;
                    if(tmp>Width-1)
                        tmp=Width-1;
                    Border[RIGHT][i]=tmp;
                    repair_Cnt++;
                }
            }
        }
        break;
    }
}

int16 get_triangle(uint8 threshold,int sign)
{
    int16 cnt=0,last_cnt=-1,last_last_cnt=-1;
    index_t=High-1;//cnt=1����ǰ�д��ڰ�->�ڡ���->�׵�����
    int16 white_to_black=0,black_to_white=0;

    for(int16 i=High-4;i>validLine+3;i--)
    {
        if(i-3>validLine+3
           &&track_Width[i]<=track_Width[i+1]
           && track_Width[i]<track_Width[i+3]
           && track_Width[i]<=track_Width[i-1]
           && track_Width[i]<track_Width[i-3]
           )
        {
            index_t=i;
            break;
        }
    }
    for(int16 i=index_t-1;i>=15;i--)
    {
        cnt=0;
        white_to_black=0;
        black_to_white=0;
        for(int16 j=Border[LEFT][index_t]+2; j<Border[RIGHT][index_t]; j++)
        {
            if(mt9v03x_image[i][j]<threshold && mt9v03x_image[i][j-1]>threshold && white_to_black==0)
                white_to_black++;
            if(mt9v03x_image[i][j]>threshold && mt9v03x_image[i][j-1]<threshold && black_to_white==0 && white_to_black!=0)
                black_to_white++;
        }
        if(white_to_black==1 && black_to_white==1)
            cnt=1;
        if(sign==0)//�ڰ׺ڡ��װװ�
            if(cnt!=0 && last_cnt==0)
                return i;
        if(sign==1)//�ڰ׺ڡ��ڰ׺ڡ��װװ�
            if(cnt!=0 && last_cnt!=0 && last_last_cnt==0)
                return i-1;
        last_last_cnt=last_cnt;
        last_cnt=cnt;
    }
    return -1;
}

int16 is_Branch(uint8 threshold,int sign)
{
    triangle_index = get_triangle(threshold,1);//��ȡ�����
    if(sign==0)//������
    {
        if(shortest_col_index>74 && shortest_col_index<114 && triangle_index>=15)
        {
            for (int16 i = High - 1; i > validLine + 4; i--)
                if (
                    (i-4>validLine+4)
                    &&(Border[LEFT][i]>Border[LEFT][i-1])
                    && (Border[LEFT][i]>Border[LEFT][i-4])
                    && (Border[RIGHT][i]<Border[RIGHT][i-1])
                    && (Border[RIGHT][i]<Border[RIGHT][i-4])

                    && (islost_record[LEFT][i] == 0)
                    && (islost_record[RIGHT][i] == 0)
                    && (islost_record[LEFT][i-1] == 0)
                    && (islost_record[RIGHT][i-1] == 0)
                    && (islost_record[LEFT][i-2] == 0)
                    && (islost_record[RIGHT][i-2] == 0)
                    && (islost_record[LEFT][i-3] == 0)
                    && (islost_record[RIGHT][i-4] == 0)
                    )
                        return 1;
        }
    }
    else if(sign==1)
    {
        if(shortest_col_index>74 && shortest_col_index<114 && triangle_index>=15)
            return 1;
    }
    return -1;
}

void judge_Branch(uint8 threshold)//ʶ���·���ж�����״̬
{
    switch (stateBranch_Sign)
    {
    case 0:
        if (left_Delta_Exceed_Cnt<=2
            &&right_Delta_Exceed_Cnt<=2
            &&is_Branch(threshold,0) != -1)
            stateBranch_Sign = 1;
        break;
    case 1:
        triangle_index = get_triangle(threshold,1);
        if(triangle_index>=33||validLine>=26)//31 24(�ٶ�110), 29 22(�ٶ�120)
        {
            rotate_Angle=0;
            stateBranch_Sign=2;
            transinfo=44+directBranch_Sign;//��֪������ʼ��ת
        }
        break;
    case 2:
        //��ת�У���ʱ��Ϊ״̬3���жϺ�������
        transinfo=5;
        break;
    case 3:
        if (left_Delta_Exceed_Cnt<=2
            &&right_Delta_Exceed_Cnt<=2
            &&is_Branch(threshold,1) != -1)
            stateBranch_Sign = 4;
        break;
    case 4:
        triangle_index = get_triangle(threshold,1);
        if(triangle_index>=46||validLine>=36)
        {
            rotate_Angle=0;
            stateBranch_Sign=5;
            transinfo=44+directBranch_Sign;//��֪������ʼ��ת
        }
        break;
    case 5:
        //��ת�У���ʱ��Ϊ״̬0���жϺ�������
        transinfo=5;
        break;
    }
}

uint8 find_frontzebra_inflection(int16 dir)
{
	if (dir == LEFT)//�󳵿�
	{
		for (int16 i =validLine; i < High - 1; i++)
			if (Border[LEFT][i] - Border[LEFT][i + 1] > 15
			    &&Border[RIGHT][i]<=Border[RIGHT][i+1]
			    &&track_Width[i]>10)
			{
				inflection_frontZebra = i;
				return 1;
			}
	}
	else if (dir == RIGHT)//�ҳ���
	{
		for (int16 i =validLine; i < High - 1; i++)
			if (Border[RIGHT][i] - Border[RIGHT][i + 1] < -15
			    &&Border[LEFT][i]>=Border[LEFT][i+1]
			    &&track_Width[i]>10)
			{
				inflection_frontZebra = i;
				return 1;
			}
	}
	inflection_frontZebra = 255;
	return 0;
}

uint8 find_rearzebra_inflection(int16 dir)
{
	if (dir == LEFT)//�󳵿�
	{
		for (int16 i = validLine; i < High - 3; i++)//
			if (Border[LEFT][i] - Border[LEFT][i + 3] > 30)//
			{
				inflection_rearZebra = i;
				return 1;
			}
	}
	else if (dir == RIGHT)//�ҳ���
	{
		for (int16 i = High - 1; i > validLine + 3; i--)
			if (Border[RIGHT][i] - Border[RIGHT][i - 3] < -30)
			{
				inflection_rearZebra = i;
				return 1;
			}
	}
	inflection_rearZebra = 255;
	return 0;
}

int16 is_Zebra_In()
{
    if(index_Garage>=5)
        return 1;
    return 0;
}

int16 find_OutGarage_Start()
{
    if(directGarage_Sign==RIGHT)
    {
        for(int16 i=High-1;i>validLine;i--)
            if(Border[LEFT][i]-Border[LEFT][i-1]>10)
                return Border[LEFT][i];
    }
    else if(directGarage_Sign==LEFT)
    {
        for(int16 i=High-1;i>validLine;i--)
            if(Border[RIGHT][i]-Border[RIGHT][i-1]<-10)
                return Border[RIGHT][i];
    }
    return -1;
}

int16 is_Zebra_Out(uint8 threshold)
{
    int16 centre=(Border[LEFT][High-1]+Border[RIGHT][High-1])<<1;
    int16 cnt=0;
    for(int16 j=centre;j>Border[LEFT][High-1];j--)
    {
        int16 tmp=0;
        for(int16 i=High-1;i>validLine+1;i--)
            if((mt9v03x_image[i][j]<threshold&&mt9v03x_image[i-1][j]>threshold)
              ||(mt9v03x_image[i][j]>threshold&&mt9v03x_image[i-1][j]<threshold))
                tmp++;
        if(tmp>cnt)
            cnt=tmp;
    }
    for(int16 j=centre+1;j<Border[RIGHT][High-1];j++)
    {
        int16 tmp=0;
        for(int16 i=High-1;i>validLine+1;i--)
            if((mt9v03x_image[i][j]<threshold&&mt9v03x_image[i-1][j]>threshold)
              ||(mt9v03x_image[i][j]>threshold&&mt9v03x_image[i-1][j]<threshold))
                tmp++;
        if(tmp>cnt)
            cnt=tmp;
    }
    return cnt;
}

void repair_Garage_Out()
{
    if(directGarage_Sign==RIGHT)
    {
        repair_Cnt=1;
        Border[LEFT][High-1]=55;
        for(int16 i=High-2;i>validLine;i--)
        {
            int16 tmp=Border[LEFT][High-1]+17*repair_Cnt;//10
            if(tmp>Width-2)
                tmp=Width-2;
            Border[LEFT][i]=tmp;
            repair_Cnt++;
        }
    }
    else if(directGarage_Sign==LEFT){

        repair_Cnt=1;
        Border[RIGHT][High-1]=131;
        for(int16 i=High-2;i>validLine;i--)
        {
            int16 tmp=Border[RIGHT][High-1]-17*repair_Cnt;//10
            if(tmp<0)
                tmp=0;
            Border[RIGHT][i]=tmp;
            repair_Cnt++;
        }
    }
}

void judge_Garage_In()//ʶ�𳵿�
{
	switch (stateGarage_Sign)
	{
	/*case 0:
        if (find_frontzebra_inflection(directGarage_Sign) == 1
            && find_rearzebra_inflection(directGarage_Sign) == 1)
            stateGarage_Sign = 1;
		break;*/
	case 0:
		if (find_frontzebra_inflection(directGarage_Sign) == 1
			&& inflection_frontZebra >= 37)//15
			{
		        rotate_Angle=0;
		        stateGarage_Sign=1;
		        transinfo=11+directGarage_Sign;
			}
		break;
	case 1:
	    transinfo=5;
	    break;
	}
}

void repair_Garage_In(uint8 threshold)//���ⲹ��
{
	switch (stateGarage_Sign)
	{
	case 2:
        repair_Cnt = 1;
        if(directGarage_Sign==RIGHT)
        {
            for (int i = inflection_frontZebra; i < High; i++)
            {
                int16 tmp = Border[RIGHT][inflection_frontZebra + 3] - 8 * repair_Cnt;//4.5
                if(tmp<0)
                    tmp=0;
                Border[LEFT][i]=tmp;
                repair_Cnt++;
            }
        }
        else if(directGarage_Sign==LEFT)
        {
            for (int i = inflection_frontZebra; i < High; i++)
            {
                int16 tmp = Border[LEFT][inflection_frontZebra + 3] + 8 * repair_Cnt;//4.5
                if(tmp>Width-2)
                    tmp=Width-2;
                Border[RIGHT][i]=tmp;
                repair_Cnt++;
            }
        }
        break;
	}
}
