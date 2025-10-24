#ifndef _SLOPE_INTERCEPT_H_
#define _SLOPE_INTERCEPT_H_

void calculate_s_i(uint8 start_, uint8 end_, uint8 *border, float *slope_rate, float *intercept);
float Slope_Calculate(uint8 begin_, uint8 end_, uint8 *border);
float Slope_Calculate_xychange(uint8 begin_, uint8 end_, uint8 *border);
float calculate_slope_improved(uint8 *border, uint8 begin, uint8 end);
#endif
