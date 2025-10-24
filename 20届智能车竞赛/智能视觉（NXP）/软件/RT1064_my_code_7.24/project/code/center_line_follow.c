#include "my_code.h"
#include "math.h"

#define RAD_TO_DEG (180.0 / M_PI)
#define MOVE_RIGHT 		0
#define MOVE_FORWARD 	90
#define MOVE_LEFT 		180
float zp = 1.0f;
char motor1_vel[20],motor2_vel[20],motor3_vel[20];
char e1_data[20], e2_data[20], e3_data[20];
float d_ul = 0.94;
//float d_md = 1.05;
float d_md = 1.0;
float d_ur =1.0;
char s_rate[20], s_angle[20], dev[20];
uint8 limit = 15;
float slope_limit = 15.0f;
uint8 gyro_devision = 45;
uint8 devision = 10;
pid_controller cl_angle_pid;//center line angle pid
pid_controller dv_pid;			//deviation pid
pid_controller mcx_pid;
pid_controller cl_angle_pid_mini;
//斜率转换角度函数
float slope_to_angle(float slope) 
{
	// 处理垂直线（斜率无限大）的情况
	if (isinf(slope)) return 90.0;
	
	float angle_rad = atan(slope);
	float angle_deg = angle_rad * RAD_TO_DEG;
	
	// 转换为0-180度范围
	if (angle_deg < 0) angle_deg += 180.0;
	
	return angle_deg;
}

void centerline_follow(uint8 *center_line, uint8* move_angle,int16* turn_angle)
{
	int16 i;
	float slope_rate, slope_angle, deviation = 0,move_deviation;
	int16 start_row, end_row;
	start_row = image_h - 80;
	end_row = image_h - 60;
//	slope_rate = calculate_slope();
//	slope_rate = calculate_slope_improved(center_line, start_row, end_row);
//	slope_angle = slope_to_angle(slope_rate);

	
	// 使用加权平均计算deviation
	deviation = 0;
	float weight = 0.85f;
	// 计算两个区域的点数
	int points_lower = end_row - start_row;
	int points_bottom = image_h - 3 - end_row + 1;
	int total_points = points_lower + points_bottom;
	
	for(i = start_row; i < end_row; i++) {
		deviation += center_line[i] * weight;
	}
	
	for(i = end_row; i <= image_h - 3; i++) {
		deviation += center_line[i] * (1.0f - weight);
//		if(i>=image_h-40)
			move_deviation += center_line[i];
	} 
	
	// 计算加权平均
	deviation = deviation / (weight * points_lower + (1.0f - weight) * points_bottom);
	move_deviation  = move_deviation/58;
//	sprintf(dev, "%.2f", deviation);
//	tft180_show_string(80, 20, dev);
	
	if(deviation <= 94 + devision &&deviation >= 94 - devision)
	{
		*move_angle = 90 + (int16)PID_control(0, move_deviation - 94, &dv_pid);
		*turn_angle = (int16)PID_control(0, deviation - 94, &cl_angle_pid_mini);
	}
	else        
	{
		*move_angle = 90 ;
		*turn_angle = (int16)PID_control(0, deviation - 94, &cl_angle_pid);
	}
	
	
//	if		 ((deviation > image_w / 2 + limit) && fabs(slope_angle - 90) < slope_limit) *move_angle = 90 - devision_vel;				//如果截距在图像中心的右边就向右平移
//	else if((deviation < image_w / 2 - limit) && fabs(slope_angle - 90) < slope_limit) *move_angle = 90 + devision_vel;				//如果截距在图像中心的左边就向左平移
//	else *move_angle = MOVE_FORWARD;																					//截距在图像中心附近就不平移
//	tft180_show_string(90,60,s_rate);
//	tft180_show_string(90,80,s_angle);

}


void move_ctrl(int move_speed, uint8 move_angle, int16 turn_angle)
{
	float motor_ul_speed, motor_ur_speed, motor_md_speed;
	float z_speed = 0.0f;
	z_speed = turn_angle;//move_angle == 90 is forward
	if(z_speed >  180) z_speed -= 360;
	if(z_speed < -180) z_speed += 360;
	motor_md_speed =   -(cosf(ANGLE_TO_RAD(0  + move_angle))) * move_speed - z_speed * zp - (imu660ra_gyro_z / gyro_devision);
	motor_ur_speed =   -(cosf(ANGLE_TO_RAD(60 + move_angle))) * move_speed + z_speed * zp + (imu660ra_gyro_z / gyro_devision);
	motor_ul_speed = 	(cosf(ANGLE_TO_RAD(60 - move_angle))) * move_speed - z_speed * zp - (imu660ra_gyro_z / gyro_devision);
////	motor_md_speed =   (cosf(ANGLE_TO_RAD(0  + move_angle))) * move_speed + z_speed * zp + (imu660ra_gyro_z / gyro_devision);
////	motor_ur_speed =   (cosf(ANGLE_TO_RAD(60 + move_angle))) * move_speed - z_speed * zp - (imu660ra_gyro_z / gyro_devision);
////	motor_ul_speed = 	-(cosf(ANGLE_TO_RAD(60 - move_angle))) * move_speed + z_speed * zp + (imu660ra_gyro_z / gyro_devision);
	motor_ul_speed *= d_ul;
	motor_md_speed *= d_md;
	motor_ur_speed *= d_ur;
//	sprintf(motor1_vel,"%.2f",motor_md_speed);
//	sprintf(motor2_vel,"%.2f",motor_ur_speed);
//	sprintf(motor3_vel,"%.2f",motor_ul_speed);
	encoder1_data =  -get_encoder1_data();
	encoder2_data = 	get_encoder2_data();
	encoder3_data =  -get_encoder3_data();
	limit_edata();
	//velocity control
//	int vel1 = (int)PID_control(-50, encoder1_data, &motor1_pid);
//	int vel2 = (int)PID_control(-50, encoder2_data, &motor2_pid);
//	int vel3 = (int)PID_control(0, encoder3_data, &motor3_pid);
	
	int vel1 = (int)PID_control(motor_md_speed, encoder1_data, &motor1_pid);
	int vel2 = (int)PID_control(motor_ur_speed, encoder2_data, &motor2_pid);
	int vel3 = (int)PID_control(motor_ul_speed, encoder3_data, &motor3_pid);
//	if(vel1 == 5000 || vel2 == 5000 || vel3 == 5000) gpio_set_level(BUZZER_PIN,1);
//	else gpio_set_level(BUZZER_PIN,0);
		
//	tft180_show_string(30, 70 , motor1_vel);
//	tft180_show_string(30, 90 , motor2_vel);
//	tft180_show_string(30, 110, motor3_vel);
	
	//set motor velocity
	set_motor1_vel(vel1);
	set_motor2_vel(vel2);
	set_motor3_vel(vel3);
}
