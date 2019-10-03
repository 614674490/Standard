#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "stm32f4xx_hal.h"   //�����int....Ҫ�õ��˿�
#include "remote.h"

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.01f

//ҡ��ԭ�ز���ҡ�����Ƕ�(��)
#define TWIST_NO_MOVE_MAX_ANGLE 55.0f
//ҡ��ԭ�ز���ҡ�����С�Ƕ�(��)
#define TWIST_NO_MOVE_MIN_ANGLE 40.0f
//ҡ�ڹ��̵����˶����Ƕ�(��)
#define TWIST_MOVE_ANGLE    20.0f

//ҡ���������(ms)
#define TWIST_MAX_T 2000.0f

//ҡ�����]С����(ms)
#define TWIST_MIN_T 800.0f

#define TO_MIDDLE_SPEED  2800 //�����ٶ�
extern int power_limit_c;

typedef enum
{
	POWER_NULL=0,  //�����й��ʿ���
	POWER_NORMAL=1,  //���ʱջ���������
	POWER_DANGER=2,  //����Σ�տ���
	POWER_EXCEPT=3,  //�����쳣����
}POWER_CONTROL_STATUS;
	
//�����ٶ�
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
	  int16_t Chassis_Motor_Torque_Current_1;  //3508ת�ص���
    int16_t Chassis_Motor_Actual_Speed_2;
	  int16_t Chassis_Motor_Torque_Current_2;  //3508ת�ص���
    int16_t Chassis_Motor_Actual_Speed_3;
	  int16_t Chassis_Motor_Torque_Current_3;  //3508ת�ص���
		int16_t Chassis_Motor_Actual_Speed_4;
	  int16_t Chassis_Motor_Torque_Current_4;  //3508ת�ص���
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
extern volatile float y_ecdangsav ;     //Ť��������Y��ǶȻ������
extern float rotate_k;
extern CHASSIS_WALK_SPEED Chassis_Walk_Speed;
extern POWER_CONTROL_STATUS power_control_status;
#endif
