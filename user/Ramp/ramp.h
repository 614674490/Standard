#ifndef __ramp_H
#define __ramp_H

#include "stm32f4xx_hal.h"
#include "sys.h"
#include "tim.h"

typedef enum       //ö������
{
	CHASSIS_RAMP_FB = 0,    //ǰ��
	CHASSIS_RAMP_RL = 1,    //����
	FRICTION_RAMP1 = 2,      //����   δ�����ô�
	PITCH_RAMP = 3,         //��̨P����
	YAW_RAMP = 4,           //Y��
	ROTATE = 5,             //ת��
	FRICTION_RAMP2=6,
}ramp_t;	 
//��һ���͵����������ڸú��� �ڶ�����ֵ����confg.h
typedef struct{
	u8 chassis_fb;
	u8 friction1;
	u8 friction2;
	u8 pitch;
	u8 yaw;
	u8 chassis_rl;
	u8 rotate;
}Flag_t;                  //��־λ

typedef struct{
	u32 chassis_fb;
	u32 friction1;
	u32 friction2;
	u32 pitch;
	u32 yaw;
	u32 chassis_rl;
	u32 rotate;
}Value_t;                 //����Ŀ���ٶȵ�ʱ��

typedef struct{
	u32 chassis_fb;
	u32 friction1;
	u32 friction2;
	u32 pitch;
	u32 yaw;
	u32 chassis_rl;
	u32 rotate;
}Get_t;                       //��ȡ�ļ�����ֵ

extern ramp_t ramp;
extern Flag_t flag ;
extern Value_t value ;
extern Get_t get ;

float Slope(u32 maximum_value, ramp_t ramp);
void ResetSlope(ramp_t ramp);


#endif 




