#include "ramp.h"
#include "sys.h"
#include "tim.h"
//清零
Flag_t flag  ={0,0,0,0,0,0,0};
Value_t value={0,0,0,0,0,0,0};
Get_t    get= {0,0,0,0,0,0,0};
ramp_t ramp;
/**************************实现函数********************************************
*函数原型:	  float Slope(u32 maximum_value, ramp_t ramp)
*功　　能:	  使电机速度缓慢提升到设定值
输入参数：  maximum_value：到达目标速度的时间=maximum_value*10us
           ramp：当前斜坡类型
输出参数： 输出比例系数
*******************************************************************************/
float Slope(u32 maximum_value, ramp_t ramp)
{
	switch(ramp)
	{
		case CHASSIS_RAMP_FB:
		{
			value.chassis_fb=maximum_value;
			if(!flag.chassis_fb)
			{
				get.chassis_fb=Get_Time5_Micros();
				flag.chassis_fb=1;
			}
			if(Get_Time5_Micros()>(value.chassis_fb+get.chassis_fb))
				return 1;
			else
				return ((Get_Time5_Micros()-get.chassis_fb)/(value.chassis_fb*1.0f));
		}
		case CHASSIS_RAMP_RL:
		{
			value.chassis_rl=maximum_value;
			if(!flag.chassis_rl)
			{
				get.chassis_rl=Get_Time5_Micros();
				flag.chassis_rl=1;
			}
			if(Get_Time5_Micros()>(value.chassis_rl+get.chassis_rl))
				return 1;
			else
				return ((Get_Time5_Micros()-get.chassis_rl)/(value.chassis_rl*1.0f));
		}
		case FRICTION_RAMP1:
		{
			value.friction1=maximum_value;
			if(!flag.friction1)
			{
				get.friction1=Get_Time5_Micros();
				flag.friction1=1;
			}
			if(Get_Time5_Micros()>(value.friction1+get.friction1))
				return 1;
			else
				return ((Get_Time5_Micros()-get.friction1)/(value.friction1*1.0f));
		}
		case FRICTION_RAMP2:
		{
			value.friction2=maximum_value;
			if(!flag.friction2)
			{
				get.friction2=Get_Time5_Micros();
				flag.friction2=1;
			}
			if(Get_Time5_Micros()>(value.friction2+get.friction2))
				return 1;
			else
				return ((Get_Time5_Micros()-get.friction2)/(value.friction2*1.0f));
		}
		case PITCH_RAMP:
		{
			value.pitch=maximum_value;
			if(!flag.pitch)
			{
				get.pitch=Get_Time5_Micros();
				flag.pitch=1;
			}
			if(Get_Time5_Micros()>(value.pitch+get.pitch))
				return 1;
			else
				return ((Get_Time5_Micros()-get.pitch)/(value.pitch*1.0f));
		}
		case YAW_RAMP:
		{
			value.yaw=maximum_value;
			if(!flag.yaw)
			{
				get.yaw=Get_Time5_Micros();
				flag.yaw=1;
			}
			if(Get_Time5_Micros()>(value.yaw+get.yaw))
				return 1;
			else
				return ((Get_Time5_Micros()-get.yaw)/(value.yaw*1.0f));
		}
		case ROTATE:
		{
			value.rotate=maximum_value;
			if(!flag.rotate)
			{
				get.rotate=Get_Time5_Micros();
				flag.rotate=1;
			}
			if(Get_Time5_Micros()>(value.rotate+get.rotate))
				return 1;
			else
				return ((Get_Time5_Micros()-get.rotate)/(value.rotate*1.0f));
		}
		default:
			return 0;
	}
}
/**************************实现函数********************************************
*函数原型:	  void ResetSlope(ramp_t ramp)
*功　　能:	  释放斜坡函数
输入参数：  无
输出参数：  无
*******************************************************************************/
void ResetSlope(ramp_t ramp)//释放只是为了让下次在进斜坡
{
	switch (ramp)
	{
		case CHASSIS_RAMP_FB:
		{
			flag.chassis_fb=0;
		}break;
		case CHASSIS_RAMP_RL:
		{
			flag.chassis_rl=0;
		}break;
		case FRICTION_RAMP1:
		{
			flag.friction1=0;
		}break;
		case FRICTION_RAMP2:
		{
			flag.friction2=0;
		}break;
		case PITCH_RAMP:
		{
			flag.pitch=0;
		}break;
		case YAW_RAMP:
		{
			flag.yaw=0;
		}break;
		case ROTATE:
		{
			flag.rotate=0;
		}break;
	}
}


