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



void trackBorder_Get(uint8 threshold);
void specialElem_Judge();
int16 centre_line_get();
uint8 OTSU(uint8 *pre_image);
float Regression(uint8 *Border);

#endif /* CODE_TRACKGET_H_ */
