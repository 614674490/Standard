#include "capacity.h"
#include "rtos.h"

/**���������������***/
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

//���㳬�����ݵĳ�繦�� 
void Capacity_Charge_Power()
{
	if(SupperCapacitorSwitch==1&&Capacitor_voltage<23.5)  //����������ڷŵ� ��Գ������ݽ��������ʳ��
	{
		CAP_Power+=PID_Increment(Total_power,70);   // ****************�о���ϵ����(΢��)
	}
	else if(Chassis_Power<60&&Chassis_Power_Last<55)  //������̶� С��80W ��繦��=75-���̹���  ����Ӱ�칦�ʻ�
	{
		CAP_Now_Power=Total_power-Chassis_Power;
		CAP_Tar_Power=75-Chassis_Power;
		if(__fabs(CAP_Now_Power)<5||__fabs(CAP_Now_Power)>23.8||CAP_Now_Power<0)  //����������ë��
		{
			CAP_Now_Power=0;
		}
		if(CAP_Tar_Power>75)  //��繦���޷�
		{
			CAP_Tar_Power=75;
		}
		if(CAP_Now_Power<23)  //����ģʽ�½��п���
		{
			CAP_Power+=PID_Increment(CAP_Now_Power,CAP_Tar_Power);
		}
	}
	else   //�����ѹ�ϴ���Ϊ��ѹ��磬������Ч
		CAP_Power=0;
	if(CAP_Power<0)
		CAP_Power=0;
	if(SupperCapacitorSwitch==1&&Chassis_Power>50)  //��ǰΪ�������ݹ��磬�������ܣ���С���ʳ��
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

//���Ƴ�ŵ����
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
			charge_ma=CAP_Power/Capacitor_voltage;  //���������
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

//��Դ�л�
void Power_Switch()
{


		if(SupperCapacitorSwitch==1)  //�������ݹ���  
		{
			if(Capacitor_voltage>13.0f)
				CAPACITY_START();
			else
				POWER_START();
			if(Total_power>77)    //�ܹ��ʳ�����  �����ǵ��̳�����ʱ�ٽ��г�������
				CHARGE_STOP();    //ֹͣ���
			else
				CHARGE_START();
		}
		else
		{
			POWER_START();   //��Դ���� ��CAPACITY_START()����һ��IO��
			if(Total_power>77)  //�ܹ��ʳ�����  �����ǵ��̳�����ʱ�ٽ��г�������
				CHARGE_STOP();    //ֹͣ���
			else
				CHARGE_START();
		}
}

