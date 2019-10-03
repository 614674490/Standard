#ifndef _aimauto_h
#define _aimauto_h

#include "sys.h"
typedef struct//此结构体有问题，暂未使用
{
	float data_speed_raw;    		 //速度原始值
	float data_speed_filter; 		 //速度滤波值
	float data_raw;              //数据原始值
	float data_delay;       		 //数据延时值
	float data_filter;      		 //数据滤波值
	float data_save;             //数据保存值
	int16_t data_rec;            //数据接收值
	float data_tran;             //数据转换值
	float data_offset;           //调节视觉与陀螺仪数据的峰峰值 使相位保持一致
	float expect_value;          //计算后的期望值
}AIMAUTO_DATA;   

void aimauto_control(void);

extern AIMAUTO_DATA aimauto_yaw;
extern AIMAUTO_DATA aimauto_pitch;
extern AIMAUTO_DATA aimauto_dist;

extern int opencam_flag,patrol_flag;

extern int16_t yaw_rec,pitch_rec,dist_rec,thelta_raw;
extern float y_angle,p_angle,yaw_delay,y_angle_offset,yaw_speed_filter,dist_filter,yaw_offset;
extern float pitch_delay,p_angle_offset,pitch_speed_filter,pitch_camera_filter;
extern float prediction_testy;
#endif



