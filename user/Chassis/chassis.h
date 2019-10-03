#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "stm32f4xx_hal.h"   //下面的int....要用到此库
#include "remote.h"

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.01f

//摇摆原地不动摇摆最大角度(度)
#define TWIST_NO_MOVE_MAX_ANGLE 55.0f
//摇摆原地不动摇摆最大小角度(度)
#define TWIST_NO_MOVE_MIN_ANGLE 40.0f
//摇摆过程底盘运动最大角度(度)
#define TWIST_MOVE_ANGLE    20.0f

//摇摆最大周期(ms)
#define TWIST_MAX_T 2000.0f

//摇摆最大]小周期(ms)
#define TWIST_MIN_T 800.0f

#define TO_MIDDLE_SPEED  2800 //归中速度
extern int power_limit_c;

typedef enum
{
	POWER_NULL=0,  //不进行功率控制
	POWER_NORMAL=1,  //功率闭环正常控制
	POWER_DANGER=2,  //功率危险控制
	POWER_EXCEPT=3,  //功率异常控制
}POWER_CONTROL_STATUS;
	
//底盘速度
typedef enum
{
	NORMAL_SPEED=7000, 
	MIDDLE_SPEED=8000,  
	HIGH_SPEED=12000,     
}CHASSIS_WALK_SPEED;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

typedef __packed struct
{
    int16_t Chassis_Motor_PID_Expect_1;
    int16_t Chassis_Motor_PID_Expect_2;
    int16_t Chassis_Motor_PID_Expect_3;
		int16_t Chassis_Motor_PID_Expect_4;
}Chassis_Motor_PID_Expect;

typedef __packed struct
{
    int16_t Chassis_Motor_Actual_Speed_1;
	  int16_t Chassis_Motor_Torque_Current_1;  //3508转矩电流
    int16_t Chassis_Motor_Actual_Speed_2;
	  int16_t Chassis_Motor_Torque_Current_2;  //3508转矩电流
    int16_t Chassis_Motor_Actual_Speed_3;
	  int16_t Chassis_Motor_Torque_Current_3;  //3508转矩电流
		int16_t Chassis_Motor_Actual_Speed_4;
	  int16_t Chassis_Motor_Torque_Current_4;  //3508转矩电流
}Chassis_Motor_Actual_Speed;

void PID_Expect(void);
void Chassis_Speed_Ref_Init(void);
void Chassis_Motor_Control(void);
void Chassis_Rotate_Control(void);
void Chassis_Power_Control(void);
void Chassis_Data_Send(void);
float Get_chassis_motor_power(int speed,int current );
float Get_Power(void);
float Set_twist_angle(float A,float T);
void Set_random_twist(float *A,float *T,float A_max,float T_max);
extern ChassisSpeed_Ref_t          Chassis_Speed_Ref;

extern Chassis_Motor_PID_Expect    Chassis_Motor_PID_Ref;

extern Chassis_Motor_Actual_Speed  Chassis_Motor_Actual_Speed_ref;
extern int rorate_slow,twist_slow;
extern int expect_sum;
extern float power_offset;
extern volatile float y_ecdangsav ;     //扭腰过程中Y轴角度缓存变量
extern float rotate_k;
extern CHASSIS_WALK_SPEED Chassis_Walk_Speed;
extern POWER_CONTROL_STATUS power_control_status;
#endif
