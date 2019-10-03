#ifndef _capacity_h
#define _capacity_h
#include "sys.h"

extern u8 send_data[8];       //��������Ϣ���͵�C8T6
extern u8 cap_flag;   //���ݳ�ŵ��־λ
extern float pow_sum; 
extern float pow_ave;   //ƽ������
extern u8 pow_i;
extern int32_t vel_sum;  
extern int16_t vel_ave;  //ƽ���ٶ�
extern u8 vel_i;

extern float power_expect;        //��������
extern float Last_expect;        //�ϴ���������
extern float power_actual;       //ʵ�ʹ���
extern float power_last;         //�ϴι���
extern float Power_Inc;         //��������
extern float current_expect;     //��������
extern float bili;             //�����������
extern u16 charge_data;

extern float power_buffer_real;      //ʵʱ��������
extern float power_buffer_threshold; //����������ֵ
extern float power_target;           //Ŀ�깦�� ���ݵ�ǰ�������������ϱ仯
extern u8 cap_status ;
void Capacity_Data_Send(void);
void Power_Limit(float power_actual_target);


#endif

