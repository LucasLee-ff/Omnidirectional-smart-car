/*
 * TrackGet.h
 *
 *  Created on: 2021Äê2ÔÂ7ÈÕ
 *      Author: 16917
 */



#ifndef CODE_TRACKGET_H_
#define CODE_TRACKGET_H_

#define LEFT 0
#define CENTRE 1
#define RIGHT 2
#define High 50
#define Width 188
#define Default_Straight_Judge 20

#define Garage_Out 0
#define Garage_In 1

typedef enum{
    Straight=0,
    Corner,
    Crossing,
    Ring,
    Branch,
    Ramp,
    Garage,
    Stop
}track_Type_Enum;

void trackBorder_Get(uint8 threshold);
void specialElem_Judge();
int16 centre_line_get();
uint8 OTSU(uint8 *pre_image);
void judge_Ring();
void repair_Ring(uint8 threshold);
int16 is_Branch(uint8 threshold,int sign);
void judge_Branch(uint8 threshold);
int16 is_Zebra_In();
int16 is_Zebra_Out(uint8 threshold);
int16 find_OutGarage_Start();
void judge_Garage_In();
void repair_Garage_In(uint8 threshold);
void repair_Garage_Out();
void judge_Cross();
uint8 find_Inflection_A(int16 start,int16 end,int dir,int sign);
uint8 find_Inflection_B(int16 start,int16 end,int dir);
uint8 find_Inflection_C(int16 start,int16 end,int dir,int sign);
uint8 is_Straight(uint8 start_Index,uint8 end_Index,uint8 side);

#endif /* CODE_TRACKGET_H_ */
