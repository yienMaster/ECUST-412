#ifndef CODE_LEG_ADAPTIVE_H_
#define CODE_LEG_ADAPTIVE_H_

#include "zf_common_headfile.h"
#include "FiveBarLinkageData.h"
#include"math.h"
#include"engine.h"
#include"pid.h"


/*      /  |
 *   |     |  pitch > 0
 *   |     |
 *   ×ó    ÓÒ
 */

void roll_control(float leg_target,float leg_error);
void leg_roll_high(float leg_target,float angle_error);
void leg_roll_control(float leg_target, float angle_error);
#endif /* CODE_ENGINE_H_ */
