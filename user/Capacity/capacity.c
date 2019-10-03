#include "include.h"
u8 cap_status = 0 ; //�������ݷ����ĵ��ݿ���״̬
u8 cap_flag=0;
u8 send_data[8]={0xaa,0,0,0,0,0,0,0x55};
float pow_sum=0;
float pow_ave=0;
u8 pow_i=0;
int32_t vel_sum=0;
int16_t vel_ave=0;
u8 vel_i=0;
float power_buffer_real=0;      //ʵʱ��������
float power_buffer_threshold=0; //����������ֵ

float power_target=0;        //Ŀ�깦�� ���ݵ�ǰ�������������ϱ仯
float power_expect=0;        //�������
float Last_expect=0;        //�ϴ��������
float power_actual=0;       //ʵ�ʹ���
float power_last=0;         //�ϴι���
float Power_Inc=0;         //��������
float current_expect=0;     //��������
float bili = 0;             //�����������
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
				power_expect=Last_expect+PID_Increment(power_actual,power_actual_target,&Power_limit); //����ʽ���㹦������ֵ
				if(power_expect<0)       
				{
					power_expect=0;  
				}
				if(ext_power_heat_data.chassis_volt!=0)
					current_expect=power_expect/ext_power_heat_data.chassis_volt;  //�����������ֵ(A) P=UI
				else 
					current_expect=power_expect/24.0f;
				Last_expect=power_expect;
				if(power_expect<0.01f)
				{
					bili=0;
				}
				else
				{
					bili=(current_expect*CURRENT_OFFSET/(expect_sum+50));    //����ÿ���ֵĵ���ֵ   +50 ��ֹ��ĸΪ��
				}
				Chassis_Motor_PID_1.pid_out = (bili * Chassis_Motor_PID_1.pid_out); //��������������̵��
				Chassis_Motor_PID_2.pid_out = (bili * Chassis_Motor_PID_2.pid_out);
				Chassis_Motor_PID_3.pid_out = (bili * Chassis_Motor_PID_3.pid_out);
				Chassis_Motor_PID_4.pid_out = (bili * Chassis_Motor_PID_4.pid_out);
				power_last=power_actual;
			  Power_Inc=power_actual-power_last;
}



