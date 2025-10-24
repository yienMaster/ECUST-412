#include "my_code.h"
#include "math.h"

void motor_init()
{
		gpio_init(MOTOR1_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR1_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    
		gpio_init(MOTOR2_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR2_PWM, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0

    gpio_init(MOTOR3_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);                            // GPIO 初始化为输出 默认上拉输出高
    pwm_init(MOTOR3_PWM, 17000, 0);  

}
void set_motor1_vel(int vel)
{
		//PWM_DUTY_MAX == 10000
		limit_a_b(vel, -PWM_DUTY_MAX / 4 * 3, PWM_DUTY_MAX / 4 * 3);
		if(vel >= 0)
		{
			gpio_set_level(MOTOR1_DIR, GPIO_LOW);
			pwm_set_duty(MOTOR1_PWM, vel);
		}
		else
		{
			gpio_set_level(MOTOR1_DIR, GPIO_HIGH);
			pwm_set_duty(MOTOR1_PWM, -vel);
		}
}
void set_motor2_vel(int vel)
{
		limit_a_b(vel, -PWM_DUTY_MAX / 4 * 3, PWM_DUTY_MAX / 4 * 3);
		if(vel >= 0)
		{
			gpio_set_level(MOTOR2_DIR, GPIO_LOW);
			pwm_set_duty(MOTOR2_PWM, vel);
		}
		else
		{
			gpio_set_level(MOTOR2_DIR, GPIO_HIGH);
			pwm_set_duty(MOTOR2_PWM, -vel);
		}
}
void set_motor3_vel(int vel)
{
		limit_a_b(vel, -PWM_DUTY_MAX / 4 * 3, PWM_DUTY_MAX / 4 * 3);
		if(vel >= 0)
		{
			gpio_set_level(MOTOR3_DIR, GPIO_LOW);
			pwm_set_duty(MOTOR3_PWM, vel);
		}
		else
		{
			gpio_set_level(MOTOR3_DIR, GPIO_HIGH);
			pwm_set_duty(MOTOR3_PWM, -vel);
		}
}
/*
MOTOR_PINS
-------------------------------------------------
|3																							|
|																								|
|																								|
|																							2 |
|																								|
|																								|
|1																							|
-------------------------------------------------
*/