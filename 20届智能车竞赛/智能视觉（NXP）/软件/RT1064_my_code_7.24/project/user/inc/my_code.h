#ifndef _MY_CODE_H_
#define _MY_CODE_H_

#include "zf_common_headfile.h"

//===============================var defines=======================================================================================================================================

#define PIT_CH                  (PIT_CH0 )                                // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
#define PIT_PRIORITY            (PIT_IRQn)                                // 对应周期中断的中断编号 

#define BUZZER_PIN							(B14)

#define MAX_DUTY            		(50 ) 
#define CHANNEL_NUMBER          (3)
#define MOTOR1_DIR              (C7 )
#define MOTOR1_PWM              (PWM2_MODULE0_CHA_C6)											//motor_ul
#define MOTOR2_DIR              (C9 )
#define MOTOR2_PWM              (PWM2_MODULE1_CHA_C8)											//motor_md
#define MOTOR3_DIR              (C11 )
#define MOTOR3_PWM              (PWM2_MODULE2_CHA_C10)										//motor_ur


#define ENCODER1_QUADDEC                 (QTIMER1_ENCODER1)               // 正交编码器对应使用的编码器接口 这里使用QTIMER1的ENCODER1
#define ENCODER1_QUADDEC_A               (QTIMER1_ENCODER1_CH1_C0)        // A 相对应的引脚
#define ENCODER1_QUADDEC_B               (QTIMER1_ENCODER1_CH2_C1)        // B 相对应的引脚

#define ENCODER2_QUADDEC                 (QTIMER1_ENCODER2)
#define ENCODER2_QUADDEC_A               (QTIMER1_ENCODER2_CH1_C2)        // A 相对应的引脚
#define ENCODER2_QUADDEC_B               (QTIMER1_ENCODER2_CH2_C24)       // B 相对应的引脚

#define ENCODER3_QUADDEC                 (QTIMER2_ENCODER1)
#define ENCODER3_QUADDEC_A               (QTIMER2_ENCODER1_CH1_C3)        // A 相对应的引脚
#define ENCODER3_QUADDEC_B               (QTIMER2_ENCODER1_CH2_C4)        // B 相对应的引脚


#define UART_BAUDRATE            (DEBUG_UART_BAUDRATE)                     // 默认 115200

#define UART1_INDEX            	 (UART_1)                           			
#define UART1_TX_PIN             (UART1_TX_B12)                           
#define UART1_RX_PIN             (UART1_RX_B13)                           

//openart mini
#define UART2_INDEX							 (UART_2)
#define UART2_TX_PIN						 (UART2_TX_B18)														
#define UART2_RX_PIN						 (UART2_RX_B19)

//mcx vision use flexio.h
#define UART3_INDEX							 (SCC8660_FLEXIO_COF_UART)
#define UART3_TX_PIN						 (UART4_TX_C16)
#define UART3_RX_PIN						 (UART4_RX_C17)

//uart4 to mt9v03x definition in zf_device_mt9v03x.h

//#define MT9V03X_COF_UART        (UART_5     )                                   // 配置摄像头所使用到的串口
//#define MT9V03X_COF_BAUR        (9600       )                                   // 总钻风配置串口波特率
//#define MT9V03X_COF_UART_TX     (UART5_RX_C29)                                  // 总钻风 UART-TX 引脚 要接在单片机 RX 上
//#define MT9V03X_COF_UART_RX     (UART5_TX_C28)                                  // 总钻风 UART-RX 引脚 要接在单片机 TX 上

#define UART1_PRIORITY           (LPUART1_IRQn)                                  // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体
#define UART2_PRIORITY           (LPUART2_IRQn)
#define UART3_PRIORITY           (LPUART3_IRQn)
#define UART4_PRIORITY           (LPUART4_IRQn)

//=======================================================================================================================================================================================





//==========================headfile includes=============================================================================================================================================

#include "fill_line.h"
#include "image.h"
#include "lost_line.h"
#include "slope_intercept.h"
#include "pid.h"
#include "attitude_compute.h"
#include "my_pit.h"
#include "openart.h"
#include "motor.h"
#include "encoder.h"
#include "keys.h"
#include "center_line_follow.h"
#include "stdio.h"
#include "mcx_vision.h"
#include "circle.h"

#endif
