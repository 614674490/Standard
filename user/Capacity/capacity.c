#include "include.h"
u8 cap_status = 0 ; //超级电容反馈的电容开启状态
u8 cap_flag=0;
u8 send_data[8]={0xaa,0,0,0,0,0,0,0x55};
float pow_sum=0;
float pow_ave=0;
u8 pow_i=0;
int32_t vel_sum=0;
int16_t vel_ave=0;
u8 vel_i=0;
float power_buffer_real=0;      //实时缓冲能量
float power_buffer_threshold=0; //缓冲能量阈值

float power_target=0;        //目标功率 根据当前缓冲能量而不断变化
float power_expect=0;        //输出功率
float Last_expect=0;        //上次输出功率
float power_actual=0;       //实际功率
float power_last=0;         //上次功率
float Power_Inc=0;         //功率增量
float current_expect=0;     //期望电流
float bili = 0;             //电流分配比例
u16 charge_data=0;


void Capacity_Data_Send()
{
	pow_sum+=power_actual;
	pow_i++;
	if(pow_i>7)
	{
		pow_ave=(int16_t)pow_sum>>3; 
		pow_i=0;
		pow_sum=0;
	}
	vel_sum+=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1;
	vel_i++;
	if(vel_i>7)
	{
		vel_ave=(int16_t)vel_sum>>3;
		vel_i=0;
		vel_sum=0;
	}
	send_data[1]=charge_data>>8;
	send_data[2]=charge_data;
	send_data[3]=cap_flag;
	send_data[4]=(int16_t)(pow_ave)>>8;
	send_data[5]=(int16_t)(pow_ave);
	send_data[6]=(int8_t)(vel_ave/64);	
	HAL_UART_Transmit(&huart7,send_data,8,3);
}

void Power_Limit(float power_actual_target)
{
				power_expect=Last_expect+PID_Increment(power_actual,power_actual_target,&Power_limit); //增量式计算功率期望值
				if(power_expect<0)       
				{
					power_expect=0;  
				}
				if(ext_power_heat_data.chassis_volt!=0)
					current_expect=power_expect/ext_power_heat_data.chassis_volt;  //计算电流期望值(A) P=UI
				else 
					current_expect=power_expect/24.0f;
				Last_expect=power_expect;
				if(power_expect<0.01f)
				{
					bili=0;
				}
				else
				{
					bili=(current_expect*CURRENT_OFFSET/(expect_sum+50));    //计算每个轮的电流值   +50 防止分母为零
				}
				Chassis_Motor_PID_1.pid_out = (bili * Chassis_Motor_PID_1.pid_out); //按比例分配给底盘电机
				Chassis_Motor_PID_2.pid_out = (bili * Chassis_Motor_PID_2.pid_out);
				Chassis_Motor_PID_3.pid_out = (bili * Chassis_Motor_PID_3.pid_out);
				Chassis_Motor_PID_4.pid_out = (bili * Chassis_Motor_PID_4.pid_out);
				power_last=power_actual;
			  Power_Inc=power_actual-power_last;
}



