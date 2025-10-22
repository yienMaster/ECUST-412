#ifndef __IMAGE_H__
#define __IMAGE_H__
#include "zf_common_headfile.h"


#define BRIGHTNESS_MIDDLE_THRESHOLD         (240)
#define BRIGHTNESS_SURROUNDING_THRESHOLD         (160)


typedef struct
{
        int16 row;
        int16 col;
}Point_Structure;


extern Point_Structure point_up,point_down,point_middle;
extern uint8 img_data[MT9V03X_H][MT9V03X_W];
extern uint8 point_distance,target_point_distance;
extern uint8 stop_count,stop_flag;
extern uint8 find_point_flag;


uint8 Find_Point(void);


//void findTwoBrightestPoint(void);


#endif
