/*
 * TrackGet.c
 *
 *  Created on: 2021年2月7日
 *      Author: 16917
 */
#include "headfile.h"
#include "TrackGet.h"

int16 centre_last=Width/2;//上次扫描得到的中线位置
uint8 a,b;//差和比计算
int16 delta=1;//每隔几列扫描边线
int16 threshold=100;//阈值
int16 EdgeL=0,EdgeR=Width-1,lastEdgeL=0,lastEdgeR=Width-1;//记录左右边界
int16 int_tmp1,int_tmp2;
int16 validLine;//有效扫描行计数
int16 lostLeft_Sign,lostRight_Sign,leftLostLine_Cnt,rightLostLine_Cnt;//丢线标志
int16 track_Type;//赛道元素标志位,0:直线,1:弯道,2:十字路口,3:环岛,4:岔路,5:坡道,6:车库

int16 detectRing_Sign,stateRing_Sign,directRing_Sign;//圆环相关标志位
int16 stateBranch_Sign,encounter_Cnt;//岔路相关标志位

extern uint8  mt9v03x_image[High][Width];
uint8 post_image[High][Width];

int16 Border[3][High];
int16 Lost;
int16 halftrack_Width[High];
int16 direct_Weight[High]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,//方向权重数组
    1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

int16 default_judge_line=40;

uint8 absolute(uint8 a,uint8 b)
{
    if(a>b)
        return a-b;
    else
        return b-a;
}

void trackBorder_Get()//预处理边界
{
    memset(Border[0],0,High*sizeof(int));//数据复位
    memset(Border[1],Width/2,High*sizeof(int));
    memset(Border[2],Width-1,High*sizeof(int));

    validLine=High;//有效行初始化
    leftLostLine_Cnt=0;//丢线数量初始化
    rightLostLine_Cnt=0;
    centre_last=Width>>1;
    for(int i=High-1;i>=0;i--)//扫描左右边线并计算中线
    {
        lostLeft_Sign=lostRight_Sign=0;//丢线标志初始化
        for(int j=centre_last-delta;j>=0;j=j-delta)//扫描左边界
        {
            /*a=mt9v03x_image[i][j];
            b=mt9v03x_image[i][j+delta];
            if(absolute(a,b)/(a+b)*100>=threshold)//找到跳变点
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
            if(j-delta<0)//未找到跳变点
            {
                EdgeL=0;
                lostLeft_Sign=1;
                leftLostLine_Cnt++;
            }
        }
        for(int j=centre_last+delta;j<Width;j=j+delta)//扫描右边界
        {
            /*a=mt9v03x_image[i][j];
            b=mt9v03x_image[i][j-delta];
            if(absolute(a,b)/(a+b)*100>=threshold)//找到跳变点
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
            if(j+delta>=Width)//未找到跳变点
            {
                EdgeR=Width-1;
                lostRight_Sign=1;
                rightLostLine_Cnt++;
            }
        }

        Border[LEFT][i]=EdgeL;//左边界
        Border[RIGHT][i]=EdgeR;//右边界
        Border[CENTRE][i]=(EdgeL+EdgeR)>>1;//临时中线
        if(lostLeft_Sign==0&&lostRight_Sign==0)//两边都未丢线的情况下更新centre_last，反之继续沿用centre_last
            centre_last=(EdgeL+EdgeR)>>1;
        if(i-1>=0)//检测下一行是否为有效图像
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

/*void Elem_Judge()//赛道判断
{
    if(stateRing_Sign==0&&stateBranch_Sign==0)//不在圆环和岔路中
    {
        if(isStraight())//直道
            track_Type=0;
        else if(isCorner())//弯道
            track_Type=1;
        else if(isCross())//十字路口
            track_Type=2;
        else if(isRound(stateRing_Sign))//圆环
            track_Type=3;
        else if(isBranch(stateBranch_Sign))//岔路
            track_Type=4;
        else if(isJump())//坡道
            track_Type=5;
        else if(isGarage())//车库
            track_Type=6;
    }
    else if(stateRing_Sign!=0)//在圆环中，根据isRing函数改变状态
        isRing(stateRing_Sign);
    else if(stateBranch_Sign!=0)//在岔路中，根据isBranch函数改变状态
        isBranch(stateBranch_Sign);
}

int isStraight()//是否为直道
{

}

int isCross()//是否为十字
{

}

int isJump()//是否为坡道
{

}

int isRound(int stateRound_Sign)//是否为圆环
{

}

int isBranch(int stateBranch_Sign)//是否为岔路
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

float process_Normal()//一般情况
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

void process_Cross()//处理十字
{

}

void process_Jump()//处理坡道
{

}

//****************处理圆环*****************
uint8 r_jump_tmp,white_sum;
float process_Round()//处理圆环
{
    if(stateRing_Sign==1)//入环
    {
        r_jump_tmp=white_sum=0;
        if(detectRing_Sign==1)//左环
        {
            for(int i=High-1;i>=validLine;i--)//寻找拐点
                if(Border[LEFT][i]-Border[LEFT][i-1]<-10)
                {
                    r_jump_tmp=i;
                    break;
                }
            for(int i=High-1;i>=validLine;i--)//计算丢线数量
                if(Border[LEFT][i]<5)
                    white_sum++;
            if(r_jump_tmp>35&&white_sum>(High-1-r_jump_tmp)*3/4)//补线进入圆环
            {
                for(int i=High-1;i>=r_jump_tmp;i--)//补线
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
        if(detectRing_Sign==2)//右环
        {

        }
    }
    if(stateRing_Sign==2)//环内
    {

    }
    if(stateRing_Sign==3)//出环
    {
        if(detectRing_Sign==1)//左环
        {

        }
        if(detectRing_Sign==2)//右环
        {

        }
    }
    if(stateRing_Sign==4)//离开圆环，需要以某一边界加或减赛道一半得到中线
    {
        if(detectRing_Sign==1)//左环
        {

        }
        if(detectRing_Sign==2)//右环
        {

        }
    }
}
//****************处理圆环*****************


void process_branch()//处理岔路
{
    if(stateBranch_Sign==1)//入岔路
    {
        if(encounter_Cnt==1)//第一次走左边岔路
        {

        }
        else if(encounter_Cnt==2)//第二次走右边岔路
        {

        }
    }
    if(stateBranch_Sign==3)//出岔路
    {
        if(encounter_Cnt==1)//第一次从左边出岔路
        {

        }
        else if(encounter_Cnt==2)//第二次从右边出岔路
        {

        }
    }
}

void process_garage()//处理车库
{

}
*/
