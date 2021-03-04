/*
 * TrackGet.c
 *
 *  Created on: 2021��2��7��
 *      Author: 16917
 */
#include "headfile.h"
#include "TrackGet.h"

int16 centre_last=Width/2;//�ϴ�ɨ��õ�������λ��
uint8 a,b;//��ͱȼ���
int16 delta=1;//ÿ������ɨ�����
int16 threshold=100;//��ֵ
int16 EdgeL=0,EdgeR=Width-1,lastEdgeL=0,lastEdgeR=Width-1;//��¼���ұ߽�
int16 int_tmp1,int_tmp2;
int16 validLine;//��Чɨ���м���
int16 lostLeft_Sign,lostRight_Sign,leftLostLine_Cnt,rightLostLine_Cnt;//���߱�־
int16 track_Type;//����Ԫ�ر�־λ,0:ֱ��,1:���,2:ʮ��·��,3:����,4:��·,5:�µ�,6:����

int16 detectRing_Sign,stateRing_Sign,directRing_Sign;//Բ����ر�־λ
int16 stateBranch_Sign,encounter_Cnt;//��·��ر�־λ

extern uint8  mt9v03x_image[High][Width];
uint8 post_image[High][Width];

int16 Border[3][High];
int16 Lost;
int16 halftrack_Width[High];
int16 direct_Weight[High]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,//����Ȩ������
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

int16 default_judge_line=40;

uint8 absolute(uint8 a,uint8 b)
{
    if(a>b)
        return a-b;
    else
        return b-a;
}

void trackBorder_Get()//Ԥ����߽�
{
    memset(Border[0],0,High*sizeof(int));//���ݸ�λ
    memset(Border[1],Width/2,High*sizeof(int));
    memset(Border[2],Width-1,High*sizeof(int));

    validLine=High;//��Ч�г�ʼ��
    leftLostLine_Cnt=0;//����������ʼ��
    rightLostLine_Cnt=0;
    centre_last=Width>>1;
    for(int i=High-1;i>=0;i--)//ɨ�����ұ��߲���������
    {
        lostLeft_Sign=lostRight_Sign=0;//���߱�־��ʼ��
        for(int j=centre_last-delta;j>=0;j=j-delta)//ɨ����߽�
        {
            /*a=mt9v03x_image[i][j];
            b=mt9v03x_image[i][j+delta];
            if(absolute(a,b)/(a+b)*100>=threshold)//�ҵ������
            {
                EdgeL=j;
                tmp_threshold++;//
                break;
            }*/
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeL=j;
                break;
            }
            if(j-delta<0)//δ�ҵ������
            {
                EdgeL=0;
                lostLeft_Sign=1;
                leftLostLine_Cnt++;
            }
        }
        for(int j=centre_last+delta;j<Width;j=j+delta)//ɨ���ұ߽�
        {
            /*a=mt9v03x_image[i][j];
            b=mt9v03x_image[i][j-delta];
            if(absolute(a,b)/(a+b)*100>=threshold)//�ҵ������
            {
                EdgeR=j;
                tmp_threshold++;//
                break;
            }*/
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeR=j;
                break;
            }
            if(j+delta>=Width)//δ�ҵ������
            {
                EdgeR=Width-1;
                lostRight_Sign=1;
                rightLostLine_Cnt++;
            }
        }

        Border[LEFT][i]=EdgeL;//��߽�
        Border[RIGHT][i]=EdgeR;//�ұ߽�
        Border[CENTRE][i]=(EdgeL+EdgeR)>>1;//��ʱ����
        if(lostLeft_Sign==0&&lostRight_Sign==0)//���߶�δ���ߵ�����¸���centre_last����֮��������centre_last
            centre_last=(EdgeL+EdgeR)>>1;
        if(i-1>=0)//�����һ���Ƿ�Ϊ��Чͼ��
        {
            /*a=mt9v03x_image[i][centre_last];
            b=mt9v03x_image[i-1][centre_last];
            if(absolute(a,b)/(a+b)*100>=threshold)
            {
                validLine=i;
                break;
            }*/
            if(mt9v03x_image[i-1][centre_last]<threshold)
            {
                validLine=i;
                //ips114_showuint8(0,7,mt9v03x_image[i-1][centre_last]);
                //ips114_showint16(0,7,validLine);
                break;
            }
        }
    }
}

int16 centre_line_get()
{
    int16 tmp=0;
    //ips114_showint16(0,7,validLine);
    //ips114_showint16(0,7,Border[1][High-1]);
    //ips114_showint16(0,7,Border[1][High-2]);
    //ips114_showint16(0,7,centre_last);
    if(validLine<default_judge_line)
    {
        tmp=Border[CENTRE][default_judge_line]-Width/2;
    }
    else
    {
        tmp=Border[CENTRE][validLine]-Width/2;
    }
    return tmp;
}

/*void Elem_Judge()//�����ж�
{
    if(stateRing_Sign==0&&stateBranch_Sign==0)//����Բ���Ͳ�·��
    {
        if(isStraight())//ֱ��
            track_Type=0;
        else if(isCorner())//���
            track_Type=1;
        else if(isCross())//ʮ��·��
            track_Type=2;
        else if(isRound(stateRing_Sign))//Բ��
            track_Type=3;
        else if(isBranch(stateBranch_Sign))//��·
            track_Type=4;
        else if(isJump())//�µ�
            track_Type=5;
        else if(isGarage())//����
            track_Type=6;
    }
    else if(stateRing_Sign!=0)//��Բ���У�����isRing�����ı�״̬
        isRing(stateRing_Sign);
    else if(stateBranch_Sign!=0)//�ڲ�·�У�����isBranch�����ı�״̬
        isBranch(stateBranch_Sign);
}

int isStraight()//�Ƿ�Ϊֱ��
{

}

int isCross()//�Ƿ�Ϊʮ��
{

}

int isJump()//�Ƿ�Ϊ�µ�
{

}

int isRound(int stateRound_Sign)//�Ƿ�ΪԲ��
{

}

int isBranch(int stateBranch_Sign)//�Ƿ�Ϊ��·
{

}

int isGarage()
{

}

uint8 limit(uint8 a,uint8 min,uint8 max)
{
    if(a<min)
        a=min;
    if(a>max)
        a=max;
    return a;
}

float process_Normal()//һ�����
{
    float direct_Error=0,sum_A=0,sum_B=0;
    for(int i=High-1;i>=validLine;i--)
    {
        sum_A=sum_A+(Border[CENTRE][i]-Width/2)*direct_Weight;
        sum_B=sum_B+direct_Weight;
    }
    direct_Error=sum_A/sum_B;
    return direct_Error;
}

void process_Cross()//����ʮ��
{

}

void process_Jump()//�����µ�
{

}

//****************����Բ��*****************
uint8 r_jump_tmp,white_sum;
float process_Round()//����Բ��
{
    if(stateRing_Sign==1)//�뻷
    {
        r_jump_tmp=white_sum=0;
        if(detectRing_Sign==1)//��
        {
            for(int i=High-1;i>=validLine;i--)//Ѱ�ҹյ�
                if(Border[LEFT][i]-Border[LEFT][i-1]<-10)
                {
                    r_jump_tmp=i;
                    break;
                }
            for(int i=High-1;i>=validLine;i--)//���㶪������
                if(Border[LEFT][i]<5)
                    white_sum++;
            if(r_jump_tmp>35&&white_sum>(High-1-r_jump_tmp)*3/4)//���߽���Բ��
            {
                for(int i=High-1;i>=r_jump_tmp;i--)//����
                {
                    Border[RIGHT][i]=Border[RIGHT][High-1]+(i-High+1)*(float)(Border[RIGHT][High-1]-Border[LEFT][r_jump_tmp])/(float)(High-1-r_jump_tmp);
                    Border[CENTRE][i]=limit(Border[RIGHT][i]-halftrack_Width[i],0,Width-1);
                }
                validLine=r_jump_tmp;
                float direct_Error=0,sum_A=0,sum_B=0;
                for(int i=High-1;i>=validLine;i--)
                {
                    sum_A=(Border[CENTRE][i]-Width/2)*direct_Weight[i];
                    sum_B=sum_B+direct_Weight[i];
                }
                direct_Error=sum_A/sum_B;
            }
            else
            {

            }

        }
        if(detectRing_Sign==2)//�һ�
        {

        }
    }
    if(stateRing_Sign==2)//����
    {

    }
    if(stateRing_Sign==3)//����
    {
        if(detectRing_Sign==1)//��
        {

        }
        if(detectRing_Sign==2)//�һ�
        {

        }
    }
    if(stateRing_Sign==4)//�뿪Բ������Ҫ��ĳһ�߽�ӻ������һ��õ�����
    {
        if(detectRing_Sign==1)//��
        {

        }
        if(detectRing_Sign==2)//�һ�
        {

        }
    }
}
//****************����Բ��*****************


void process_branch()//�����·
{
    if(stateBranch_Sign==1)//���·
    {
        if(encounter_Cnt==1)//��һ������߲�·
        {

        }
        else if(encounter_Cnt==2)//�ڶ������ұ߲�·
        {

        }
    }
    if(stateBranch_Sign==3)//����·
    {
        if(encounter_Cnt==1)//��һ�δ���߳���·
        {

        }
        else if(encounter_Cnt==2)//�ڶ��δ��ұ߳���·
        {

        }
    }
}

void process_garage()//������
{

}
*/
