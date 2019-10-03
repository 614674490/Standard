#ifndef __ramp_H
#define __ramp_H

#include "stm32f4xx_hal.h"
#include "sys.h"
#include "tim.h"

typedef enum       //枚举类型
{
	CHASSIS_RAMP_FB = 0,    //前后
	CHASSIS_RAMP_RL = 1,    //左右
	FRICTION_RAMP1 = 2,      //发射   未发现用处
	PITCH_RAMP = 3,         //云台P轴电机
	YAW_RAMP = 4,           //Y轴
	ROTATE = 5,             //转弯
	FRICTION_RAMP2=6,
}ramp_t;	 
//第一个和第三个仅用于该函数 第二个的值来自confg.h
typedef struct{
	u8 chassis_fb;
	u8 friction1;
	u8 friction2;
	u8 pitch;
	u8 yaw;
	u8 chassis_rl;
	u8 rotate;
}Flag_t;                  //标志位

typedef struct{
	u32 chassis_fb;
	u32 friction1;
	u32 friction2;
	u32 pitch;
	u32 yaw;
	u32 chassis_rl;
	u32 rotate;
}Value_t;                 //到达目标速度的时间

typedef struct{
	u32 chassis_fb;
	u32 friction1;
	u32 friction2;
	u32 pitch;
	u32 yaw;
	u32 chassis_rl;
	u32 rotate;
}Get_t;                       //获取的计数器值

extern ramp_t ramp;
extern Flag_t flag ;
extern Value_t value ;
extern Get_t get ;

float Slope(u32 maximum_value, ramp_t ramp);
void ResetSlope(ramp_t ramp);


#endif 




