#ifndef __PID_H
#define __PID_H
#include "stm32f4xx_hal.h"
#include "filter.h"

#define fp32 float
typedef struct PID_PARAMETER
{
	kalman_filter_t pid_kalman_filter;
	int Expect_Value;                    //设定量
	float error_now;             
	float error_last;             
	float error_inter;  
  float error_max;                       //静态误差阈值
	float inter_max;                     //积分饱和阈值
	float Kp,Ki,Kd;  
	float ec;
  float Kpout,Kiout,Kdout;
	float inter_limit;  //积分限幅
	
	float derivative;      //微分项  微分先行参数 只用于底盘
  float lastPv;     //前一拍的测量值
  float gama;      //微分先行滤波系数	
	float last_derivative;  //前一次微分项

	uint32_t current_time;  //当前时间
	uint32_t last_time;     //上次时间
	uint32_t time;          //时间间隔
	
	int pid_out; 
  int pid_filter_out;	
	int pid_filter_filter_out;
}PID;
typedef struct PID_INCREASE
{
	float ec;
	float filiter_out;
	float current;     //当前 用于返回实际输出值
	float expect;      //期望值
	float Kp,Ki,Kd;
	float error_now;
	float error_next;
	float error_last;
	float increament;
}PID_ADD;   //增量式PID



extern int32_t ecd_value;

extern PID_ADD Power_limit,MPU_temp;
extern PID Chassis_Motor_PID_1,Chassis_Motor_PID_2,Chassis_Motor_PID_3,Chassis_Motor_PID_4;
extern PID Chassis_Motor_ECD;    //角度
extern PID Hit_speed,Hit_position;
extern PID Chassis_Motor_PID_rotate;           //转向PID
extern PID PTZ_Motor_PID_Position_Y,PTZ_Motor_PID_Speed_Y,PTZ_Motor_PID_Position_P,PTZ_Motor_PID_Speed_P,Power_PID;
extern float mpu_pwm;

void PID_Limit_Out(PID *pid,int MaxOut);
void PID_Control(float current, float expected,PID* motor_type);
void Pid_Kalman(PID* pid);
void pid_kalman_init(void);
void PID_Parment_Clear(PID *pid);
void PID_ADD_Parment_Clear(PID_ADD *pid_add);
void PID_Init(void);
void PID_Clear(void);
float PID_Increment(float current,float expect,PID_ADD* PID);//陀螺仪温度
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
#endif

