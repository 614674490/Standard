#include "capacity.h"
#include "rtos.h"

/**超级电容相关数据***/
float CAP_Power=0;
float charge_ma=0;
uint16_t charge_PWM=0;
uint8_t change_flag=0;
uint8_t charge_flag=0;
float CAP_Now_Power=0;
float CAP_Tar_Power=0;
int16_t Chassis_Power=0;
int16_t Chassis_Power_Last=0;
float vel=0;

//计算超级电容的充电功率 
void Capacity_Charge_Power()
{
	if(SupperCapacitorSwitch==1&&Capacitor_voltage<23.5)  //如果电容正在放电 则对超级电容进行满功率充电
	{
		CAP_Power+=PID_Increment(Total_power,70);   // ****************感觉关系不大(微调)
	}
	else if(Chassis_Power<60&&Chassis_Power_Last<55)  //如果底盘动 小于80W 充电功率=75-底盘功率  控制影响功率环
	{
		CAP_Now_Power=Total_power-Chassis_Power;
		CAP_Tar_Power=75-Chassis_Power;
		if(__fabs(CAP_Now_Power)<5||__fabs(CAP_Now_Power)>23.8||CAP_Now_Power<0)  //消除电流计毛刺
		{
			CAP_Now_Power=0;
		}
		if(CAP_Tar_Power>75)  //充电功率限幅
		{
			CAP_Tar_Power=75;
		}
		if(CAP_Now_Power<23)  //恒流模式下进行控制
		{
			CAP_Power+=PID_Increment(CAP_Now_Power,CAP_Tar_Power);
		}
	}
	else   //如果电压较大，则为恒压充电，控制无效
		CAP_Power=0;
	if(CAP_Power<0)
		CAP_Power=0;
	if(SupperCapacitorSwitch==1&&Chassis_Power>50)  //当前为超级电容供电，并且在跑，则小功率充电
	{
		if(CAP_Power>50)
			CAP_Power=50;
	}
	else
	{
		if(CAP_Power>150)   
			CAP_Power=150;
	}
}

//控制充放电电流
void Charge_Current()
{
		if(Capacitor_voltage<5)
		{
			charge_ma=3;
		}
		else if(Capacitor_voltage>23)//21.05
		{
			charge_ma=0.2;
		}
		else
		{
			charge_ma=CAP_Power/Capacitor_voltage;  //计算充电电流
		}
		if(charge_ma>5)
			charge_ma=5;
		if(charge_ma<0.1)
		{
			charge_PWM=0;
			TIM4->CCR3 = charge_PWM;
		}
		else
		{
			charge_PWM=(uint16_t) (840.6886f*charge_ma-15.7334f);
			TIM4->CCR3 = charge_PWM;
		}
}

//电源切换
void Power_Switch()
{


		if(SupperCapacitorSwitch==1)  //超级电容供电  
		{
			if(Capacitor_voltage>13.0f)
				CAPACITY_START();
			else
				POWER_START();
			if(Total_power>77)    //总功率超功率  不能是底盘超功率时再进行充电的限制
				CHARGE_STOP();    //停止充电
			else
				CHARGE_START();
		}
		else
		{
			POWER_START();   //电源供电 与CAPACITY_START()公用一个IO口
			if(Total_power>77)  //总功率超功率  不能是底盘超功率时再进行充电的限制
				CHARGE_STOP();    //停止充电
			else
				CHARGE_START();
		}
}

