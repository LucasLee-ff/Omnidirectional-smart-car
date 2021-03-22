/*
 * TrackGet.c
 *
 *  Created on: 2021年2月7日
 *      Author: 16917
 */
#include "headfile.h"
#include "TrackGet.h"

int16 delta=1;//每隔几列扫描边线
int16 EdgeL,EdgeR,lastEdgeL,lastEdgeR;//记录左右边界
int16 validLine,searsh_mid_line;;//有效扫描行计数
int16 lostLeft_Sign,lostRight_Sign,leftLostLine_Cnt,rightLostLine_Cnt;//丢线标志
int8 stateRing_Sign=0,directRing_Sign=-1;//圆环相关标志位
int8 stateBranch_Sign=0,encounter_Cnt=-1;//岔路相关标志位
int8 stateGarage_Sign=0,directGarage_Sign=-1;//车库相关标志位

int16 repair_Cnt;

extern uint8  mt9v03x_image[High][Width];

track_Type_Enum track=Straight;

int16 Border[3][High];
int16 halftrack_Width[High];
uint8 islost_record[3][High];
int16 direct_Weight[High]=
{50,48,46,44,42,
 40,38,36,34,32,
 30,28,26,24,22,
 20,18,16,14,12,
 10,8,6,4,2,
 15,15,15,15,15,
 0,0,0,0,0,
 0,0,0,0,0,
 0,0,0,0,0,
 0,0,0,0,0};
/*
 {10,9,8,5,4,3,2,1,1,1,1,1,1,1,1,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
 */
int16 default_judge_line=40;
uint8 centre_Delta_Exceed_Cnt;//圆环识别辅助位


uint8 absolute(uint8 a,uint8 b)//绝对值函数
{
    if(a>b)
        return a-b;
    else
        return b-a;
}
uint8 OTSU(uint8 *pre_image)//动态计算阈值
{
    uint16 piexlCount[256];
    uint8 threshold_Max=200,threshold_Min=70;

    for(int i=0;i<256;i++)//初始化数组
        piexlCount[i]=0;

    for(int i=0;i<High;i++)//统计每个灰度有多少个像素
        for(int j=0;j<Width;j++)
        {
            int tmp=*(pre_image+i*Width+j);
            piexlCount[tmp]++;
        }

    uint8 threshold;//阈值
    int32 deltaMax=0,deltaTmp;//类间方差
    for(int i=threshold_Min;i<threshold_Max;i++)//寻找使类间方差最大的阈值
    {
        int32 N0,N1,U0,U1,U0tmp,U1tmp;
        N0=N1=U0=U1=U0tmp=U1tmp=0;
        for(int j=0;j<256;j++)
        {
            if(j<=i)//较暗部分
            {
                N0=N0+piexlCount[j];
                U0tmp=U0tmp+j*piexlCount[j];
            }
            else//较亮部分
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

uint8 trackBorder_Get_Cnt=0;
int16 last_start;
void trackBorder_Get(uint8 threshold)//预处理边界
{
    memset(Border[LEFT],0,High*sizeof(int));//数据复位
    memset(Border[CENTRE],Width/2,High*sizeof(int));
    memset(Border[RIGHT],Width-1,High*sizeof(int));

    centre_Delta_Exceed_Cnt=0;
    leftLostLine_Cnt=0;//丢线数量初始化
    rightLostLine_Cnt=0;
    lostLeft_Sign=0;
    lostRight_Sign=0;
    validLine=High-1;

    int d_neg=-25,d_pos=25;//-15 15
    if(stateRing_Sign>=3&&stateRing_Sign<=4&&directRing_Sign==LEFT)
        d_neg=-90;
    else if(stateRing_Sign>=3&&stateRing_Sign<=4&&directRing_Sign==RIGHT)
        d_pos=90;

    for(int i=Width/2+d_neg;i<Width/2+d_pos;i++)//获取预备最远有效行的列数及其长度
    {
        int j=High-1;
        while(mt9v03x_image[j][i]>threshold&&j>=0)
            j--;
        if(j-1<validLine)
        {
            validLine=j-1;
            searsh_mid_line=i;
        }
    }

    for(int i=High-1;i>=validLine;i--)//扫描左右边线并计算中线
    {
        for(int j=searsh_mid_line-delta;j>=0;j=j-delta)//扫描左边界
        {
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeL=j;
                islost_record[LEFT][i]=0;
                break;
            }
            else if(j==0)//未找到跳变点
            {
                EdgeL=0;
                lostLeft_Sign=1;
                islost_record[LEFT][i]=1;
                leftLostLine_Cnt++;
                break;
            }
        }
        for(int j=searsh_mid_line+delta;j<Width-1;j=j+delta)//扫描右边界
        {
            if(mt9v03x_image[i][j]<threshold)
            {
                EdgeR=j;
                islost_record[RIGHT][i]=0;
                break;
            }
            else if(j==Width-2)//未找到跳变点
            {
                EdgeR=Width-2;
                lostRight_Sign=1;
                islost_record[RIGHT][i]=1;
                rightLostLine_Cnt++;
                break;
            }
        }

        Border[LEFT][i]=EdgeL;//左边界
        Border[RIGHT][i]=EdgeR;//右边界
        Border[CENTRE][i]=(EdgeL+EdgeR)>>1;//临时中线
        if(i<High-1)
        {
            int16 tmp=Border[CENTRE][i]-Border[CENTRE][i+1];
            if(tmp>22||tmp<-22)
                centre_Delta_Exceed_Cnt++;
        }
    }
}



uint8 is_Straight(uint8 start_Index,uint8 end_Index,uint8 side)//判断某一边从起点到终点是否为直线
{
    int white_Cnt=0;
    if(start_Index==end_Index)
        return 1;
    else
    {
        uint8 start=start_Index<end_Index?start_Index:end_Index;
        uint8 end=start_Index>end_Index?start_Index:end_Index;
        int8 delta=Border[side][start]-Border[side][start+1];
        for(uint8 i=start+1;i<end;i++)
        {
            int8 tmp=Border[side][i]-Border[side][i+1];
            if(Border[side][i]==0||Border[side][i]==Width-1)
                white_Cnt++;
            if(white_Cnt>10)
                return 0;
            if(tmp-delta>1||tmp-delta<-1)
                return 0;
        }
        return 1;
    }
}

uint8 find_Zebra()
{

}

int16 inflection_A,inflection_B,inflection_C;//由近到远定义三个拐点

uint8 find_Inflection_C(int16 start,int16 end,int dir)//远处拐点
{
    if(dir==LEFT)
    {
        for(int16 i=start+5;i<=end-3;i++)
        {
            if((Border[LEFT][i]-Border[LEFT][i+1]>5)
            &&(Border[LEFT][i]-Border[LEFT][i+2]>5)
            &&(Border[LEFT][i]-Border[LEFT][i+3]>5))
            {
                if(i-start<10&&islost_record[LEFT][i+3]==1)
                {
                    inflection_C=i;
                    return 1;
                }
                else
                {
                    inflection_C=i;
                    return 1;
                }
            }
        }
    }
    else
    {
        for(int16 i=start+5;i<=end-3;i++)
        {
            if((Border[RIGHT][i]-Border[RIGHT][i+1]<-5)
            &&(Border[RIGHT][i]-Border[RIGHT][i+2]<-5)
            &&(Border[RIGHT][i]-Border[RIGHT][i+3]<-5))
            {
                if(i-start<10&&islost_record[RIGHT][i+3]==1)
                {
                    inflection_C=i;
                    return 1;
                }
                else
                {
                    inflection_C=i;
                    return 1;
                }
            }
        }
    }
    inflection_C=255;
    return 0;
}

uint8 find_Inflection_B(int16 start,int16 end,int dir)//中间拐点
{
    if(dir==LEFT)
    {
        for(int16 i=end-1;i>=start+1;i--)
            if(//(islost_record[LEFT][i-1]==0)
            //&&(islost_record[LEFT][i]==0)
            //&&(islost_record[LEFT][i+1]==0)
            /*&&*/(Border[LEFT][i]-Border[LEFT][i-2]>0)
            &&(Border[LEFT][i]-Border[LEFT][i+2]>0))
            {
                inflection_B=i;
                return 1;
            }
    }
    else
    {
        for(int16 i=end-1;i>=start+1;i--)
            if(/*(islost_record[RIGHT][i-1]==0)
            &&(islost_record[RIGHT][i]==0)
            &&(islost_record[RIGHT][i+1]==0)
            &&*/(Border[RIGHT][i]-Border[RIGHT][i-2]<0)
            &&(Border[RIGHT][i]-Border[RIGHT][i+2]<0))
            {
                inflection_B=i;
                return 1;
            }
    }
    inflection_B=255;
    return 0;
}

uint8 find_Inflection_A(int16 start,int16 end,int dir,int sign)//近处拐点
{
    if(dir==LEFT)
    {
        if(sign==0)//进环找A点
        {
            for(int16 i=end-1;i>=start;i--)
                if((islost_record[LEFT][i]==0)
                &&(islost_record[LEFT][i-1]==1)
                &&(Border[LEFT][i]-Border[LEFT][i-1]>10))
                {
                    inflection_A=i;
                    return 1;
                }
        }
        else if(sign==1)//出环找A点
        {
            for(int16 i=end;i>=start+3;i--)
                if((Border[RIGHT][i]-Border[RIGHT][i-1]<0)
                &&(Border[RIGHT][i]-Border[RIGHT][i-2]<0)
                &&(Border[RIGHT][i]-Border[RIGHT][i-3]<0))
                {
                    inflection_A=i;
                    return 1;
                }
        }
    }
    else
    {
        if(sign==0)//进环找A点
        {
            for(int16 i=end-1;i>=start;i--)
                if((islost_record[RIGHT][i]==0)
                &&(islost_record[RIGHT][i-1]==1)
                &&(Border[RIGHT][i]-Border[RIGHT][i-1]<-10))
                {
                    inflection_A=i;
                    return 1;
                }
        }
        else if(sign==1)//出环找A点
        {
            for(int16 i=end;i>=start+6;i--)
                if((Border[LEFT][i]-Border[LEFT][i-2]>0)
                &&(Border[LEFT][i]-Border[LEFT][i-4]>0)
                &&(Border[LEFT][i]-Border[LEFT][i-6]>0))
                {
                    inflection_A=i;
                    return 1;
                }
        }
    }
    inflection_A=255;
    return 0;
}

int16 centre_line_get()//获取中线偏差值
{
    int16 sum_A=0,sum_B=0;
    int16 cnt=1;
    float k=1.6;

    Border[CENTRE][High-1]=(Border[LEFT][High-1]+Border[RIGHT][High-1])>>1;

    for(int i=High-2;i>=validLine;i--)
    {
        Border[CENTRE][i]=(Border[LEFT][i]+Border[RIGHT][i])>>1;

        if(stateRing_Sign==2)
        {
            if(directRing_Sign==LEFT&&Border[CENTRE][i]-Border[CENTRE][i+1]>5)
            {
                int16 tmp=Border[CENTRE][i+cnt]-k*cnt;
                if(tmp<0)
                    tmp=0;
                Border[CENTRE][i]=tmp;
                cnt++;
            }
            if(directRing_Sign==RIGHT&&Border[CENTRE][i]-Border[CENTRE][i+1]<-5)
            {
                int16 tmp=Border[CENTRE][i+cnt]+k*cnt;
                if(tmp>Width-1)
                    tmp=Width-1;
                Border[CENTRE][i]=tmp;
                cnt++;
            }
        }

        sum_A=sum_A+direct_Weight[High-1-i]*(Border[CENTRE][i]-Width/2);
        sum_B=sum_B+direct_Weight[High-1-i];
    }
    return sum_A/sum_B;
}

void judge_Ring()//识别圆环并判断所处状态
{
    switch(stateRing_Sign)
    {
    case 0://起始状态，检测是否遇到圆环
        if(centre_Delta_Exceed_Cnt>=2)//可能是圆环
        {
            if(is_Straight(High-1, Default_Straight_Judge, LEFT)
                &&find_Inflection_A(validLine, High-1, RIGHT,0))//是右环？
            {
                stateRing_Sign=1;
                track=Ring;
                directRing_Sign=RIGHT;
            }
            else if(is_Straight(High-1, Default_Straight_Judge, RIGHT)
                &&find_Inflection_A(validLine, High-1, LEFT,0))//是左环？
            {
                stateRing_Sign=1;
                track=Ring;
                directRing_Sign=LEFT;
            }
        }
        break;
    case 1://补第1根线状态，检测不到A拐点时进入下一阶段
        if(find_Inflection_A(validLine, High-1, directRing_Sign,0)==0)
            stateRing_Sign=2;
        break;
    case 2://补第2根线状态，检测不到C拐点时进入下一状态
        if(find_Inflection_C(validLine, High-1, directRing_Sign)==0)
            stateRing_Sign=3;
        break;
    case 3://环内状态，检测到A拐点时代表该出环补线
        if(directRing_Sign==LEFT
        &&(find_Inflection_A(validLine, High-1, LEFT,1)==1))
            stateRing_Sign=4;
        else if(directRing_Sign==RIGHT
        &&(find_Inflection_A(validLine, High-1, RIGHT,1)==1))
            stateRing_Sign=4;
        break;
    case 4://补第3根线状态，能检测到C拐点代表已出环
        if((validLine<20)&&(directRing_Sign==LEFT)&&(find_Inflection_A(validLine, High-1, LEFT, 1)==0))
            stateRing_Sign=5;
        else if((validLine<20)&&(directRing_Sign==RIGHT)&&(find_Inflection_A(validLine, High-1, RIGHT, 1)==0))
            stateRing_Sign=5;
        break;
    case 5://补第4根线状态，当左右两边最近行都不丢线代表离开圆环
        if(islost_record[LEFT][40]==0&&islost_record[RIGHT][40]==0)
        {
                stateRing_Sign=0;
                track=Straight;
        }
        break;
    }
    return;
}

void repair_Ring(uint8 threshold)//圆环补线
{
    switch(stateRing_Sign)
    {
    case 1://补第1根线
        if(directRing_Sign==LEFT)//左环补线
        {
            int16 i=inflection_A-1;
            repair_Cnt=1;
            int16 tmp=Border[LEFT][inflection_A]+0.7*repair_Cnt;
            while(i>=0&&tmp>Border[LEFT][i])
            {
                Border[LEFT][i]=tmp;
                repair_Cnt++;
                i--;
                tmp=Border[LEFT][inflection_A]+0.7*repair_Cnt;
            }
        }
        else //右环补线
        {
            int16 i=inflection_A-1;
            repair_Cnt=1;
            int16 tmp=Border[RIGHT][inflection_A]-0.6*repair_Cnt;
            while(i>=0&&tmp<Border[RIGHT][i])
            {
                Border[RIGHT][i]=tmp;
                repair_Cnt++;
                i--;
                tmp=Border[RIGHT][inflection_A]-0.6*repair_Cnt;
            }
        }
        break;
    case 2://补第2根线
        if(directRing_Sign==LEFT)//左环补线
        {
            int16 i=inflection_C+1;
            repair_Cnt=1;
            float k;
            k=2.7;//1.8
            while(i<High&&Border[LEFT][inflection_C]+k*repair_Cnt<Border[RIGHT][i])//补C拐点的线
            {
               int16 tmp=Border[LEFT][inflection_C]+k*repair_Cnt;//系数待定
               if(tmp>Width-1)
                   tmp=Width-1;
               Border[RIGHT][i]=tmp;
               repair_Cnt++;
               i++;
            }

            if(find_Inflection_B(inflection_C+2, High-1, LEFT)==1)//能找到B拐点，则补B拐点的线
            {
               repair_Cnt=1;
               for(i=inflection_B+1;i<High;i++)
               {
                   int16 tmp=Border[LEFT][inflection_B]-1.2*repair_Cnt;//系数待定
                   if(tmp<0)
                       tmp=0;
                   Border[LEFT][i]=tmp;
                   repair_Cnt++;
               }
            }
        }
        else//右环补线
        {
            int16 i=inflection_C+1;
            repair_Cnt=1;
            float k;
            k=2.7;//2

            while(i<High&&Border[RIGHT][inflection_C]-k*repair_Cnt>Border[LEFT][i])//补C拐点的线
            {
               int16 tmp=Border[RIGHT][inflection_C]-k*repair_Cnt;//系数待定
               if(tmp<0)
                   tmp=0;
               Border[LEFT][i]=tmp;
               repair_Cnt++;
               i++;
            }

            if(find_Inflection_B(inflection_C+2, High-1, RIGHT)==1)//能找到B拐点，则补B拐点的线
            {
               repair_Cnt=1;
               for(i=inflection_B+1;i<High;i++)
               {
                   int16 tmp=Border[RIGHT][inflection_B]+1.2*repair_Cnt;//系数待定
                   if(tmp>Width-1)
                       tmp=Width-1;
                   Border[RIGHT][i]=tmp;
                   repair_Cnt++;
               }
            }

        }
        break;
    case 5://补第4根线
        if(directRing_Sign==LEFT)
        {
            if(find_Inflection_C(validLine, High-1, LEFT)==1)
            {
                repair_Cnt=1;
                for(int i=inflection_C+1;i<High;i++)
                {
                    int16 tmp=Border[LEFT][inflection_C]-1.1*repair_Cnt;
                    if(tmp<0)
                        tmp=0;
                    Border[LEFT][i]=tmp;
                    repair_Cnt++;
                }
            }
        }
        else
        {
            if(find_Inflection_C(validLine, High-1, RIGHT)==1)
            {
                repair_Cnt=1;
                for(int i=inflection_C+1;i<High;i++)
                {
                    int16 tmp=Border[RIGHT][inflection_C]+1.1*repair_Cnt;
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

void judge_Branch()//识别岔路并判断所处状态
{

}

void repair_Branch()//岔路补线
{

}

void judge_Garage()//识别车库
{
    if(directGarage_Sign==Garage_Out)//先写右出库
    {
        switch(stateGarage_Sign)
        {
        case 1://直走阶段
            if(Border[LEFT][30]==0)//直走到一定程度开始转弯
                stateGarage_Sign=2;
            break;
        case 2://补左线阶段
            if(find_Zebra()==0)
                stateGarage_Sign=0;
            break;
        }
    }
    else if(directGarage_Sign==Garage_In) {

    }
}

void repair_Garage()//车库补线
{
    if(directGarage_Sign==Garage_Out)
    {
        int16 inf;
        switch(stateGarage_Sign)
        {
        case 1://补一条直线
            for(int16 i=High-1;i>=validLine;i--)
            {
                Border[LEFT][i]=0;
                Border[LEFT][i]=Width-1;
            }
            break;
        case 2:
            for(int16 i=High-1;i>validLine;i--)//先找左拐点
                if(Border[LEFT][i]-Border[LEFT][i-1]>5)
                {
                    inf=i;
                    break;
                }

        }
    }
    else if(directGarage_Sign==Garage_In) {

    }
}

float cal_Curvature()
{
    float a_x=High-1,a_y=Border[CENTRE][High-1];
    float b_x=(High-1+validLine)>>1,b_y=Border[CENTRE][(High-1+validLine)>>1];
    float c_x=validLine,c_y=Border[CENTRE][validLine];

    float tmp,s,a,b,c;
    tmp=a_x*b_y+b_x*c_y+c_x*a_y-a_y*b_x-b_y*c_x-c_y*a_x;
    if(tmp<0)
        tmp=-tmp;
    s=tmp/2;
    a=sqrtf((b_x-c_x)*(b_x-c_x)+(b_y-c_y)*(b_y-c_y));
    b=sqrtf((a_x-c_x)*(a_x-c_x)+(a_y-c_y)*(a_y-c_y));
    c=sqrtf((a_x-b_x)*(a_x-b_x)+(a_y-b_y)*(a_y-b_y));
    return (a*b*c)/(4*s);
}

