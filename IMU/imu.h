#ifndef _IMU_H
#define _IMU_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "math.h"
#include "sys.h"
#define M_PI  3.1415926535f


void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //������̬
void GetPitchYawGxGyGz(void);
//extern int16_t MPU6500_FIFO[6][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
//extern int16_t HMC5883_FIFO[3][11];//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�

extern volatile float angle[3];
extern volatile float yaw_angle,pitch_angle,roll_angle,mpu_temp; //ʹ�õ��ĽǶ�ֵ

extern float force;
extern float ang_save[1024];
extern float diff_save[512];
extern u8 deal;

extern volatile float yaw_temp;

typedef struct 
{
   float   x_now;
	 float  x_last;
   float  x_next;	
   float  y_now;
	 float  y_last;
	 float  y_next;

}Diff_ANGLE;
extern volatile float yaw_temp;
extern volatile float last_yaw_temp;
extern volatile float yaw_angle_last;
extern Diff_ANGLE Diff_Yaw;
extern volatile float yaw_filter; 
extern int yaw_count ;
void MPU_Get_Temperature(void);
void IMU_Init(void);
float  diff_convert_yaw(float init_data,Diff_ANGLE *imu_type);
#endif

