#ifndef _gimbal_h
#define _gimbal_h
#include "remote.h"
#define Shoot_Count_Time  100    //射击计数缓冲次数
#define Fuzzy_Open  1
#define Fuzzy_Close 0
#define PTZ_DEBUG   2
void Gimbal_ControlData_Get(void);
void Gimbal_Init(void);
void Gimbal_Control(void);
void Gimbal_Data_Send(void);
void Friction_2312_Init(void);
void Friction_ON(int pwm);
void Friction_OFF(void) ;  //摩擦轮关闭
int System_identification(int Period[],float Omg[],int amplitude,int sampletimes);
void Friction_420S_Control(u32 soft_js_time,int period,int pwm);
void Friction_420S_Init(void);
void Friction_420S_ON(int pwm);
void Friction_420S_OFF(void);
void Friction_Init(void);
void Gimbal_Debug(void);
extern u32 expect_speed;
extern int Period[60];
extern float Omg[60];
extern volatile float temp_PTZ_Position_P,temp_PTZ_Speed_P;
extern volatile float temp_PTZ_Position_Y,temp_PTZ_Speed_Y;
extern u8 gimbal_power;
extern int sin_data;
extern u8 friction_init_flag;
#endif



