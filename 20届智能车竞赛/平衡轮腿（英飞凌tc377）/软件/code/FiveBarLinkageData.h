/*
 * 鏂囦欢鍚�: FiveBarLinkageData.h
 * 鐢熸垚鏃ユ湡: 24-Mar-2025 17:50:17
 * 鎻忚堪: 瀛樺偍浜旀潌鏈烘瀯鐨勮绠楃粨鏋�
 */



#ifndef FIVE_BAR_LINKAGE_DATA_H
#define FIVE_BAR_LINKAGE_DATA_H

#define NUM_X_VALUES 101
#define NUM_Y_VALUES 121
#define NUM_ANGLES_PER_DATA 2
void servo_control(float x, float y, int *leg1, int *leg2) ;
extern float FiveBarData[NUM_X_VALUES][NUM_Y_VALUES][NUM_ANGLES_PER_DATA];


#endif // FIVE_BAR_LINKAGE_DATA_H
