#ifndef _aimauto_h
#define _aimauto_h

#include "sys.h"
typedef struct//�˽ṹ�������⣬��δʹ��
{
	float data_speed_raw;    		 //�ٶ�ԭʼֵ
	float data_speed_filter; 		 //�ٶ��˲�ֵ
	float data_raw;              //����ԭʼֵ
	float data_delay;       		 //������ʱֵ
	float data_filter;      		 //�����˲�ֵ
	float data_save;             //���ݱ���ֵ
	int16_t data_rec;            //���ݽ���ֵ
	float data_tran;             //����ת��ֵ
	float data_offset;           //�����Ӿ������������ݵķ��ֵ ʹ��λ����һ��
	float expect_value;          //����������ֵ
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



