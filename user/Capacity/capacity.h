#ifndef _capacity_h
#define _capacity_h
#include "sys.h"

extern u8 send_data[8];       //将电容信息发送到C8T6
extern u8 cap_flag;   //电容充放电标志位
extern float pow_sum; 
extern float pow_ave;   //平均功率
extern u8 pow_i;
extern int32_t vel_sum;  
extern int16_t vel_ave;  //平均速度
extern u8 vel_i;

extern float power_expect;        //期望功率
extern float Last_expect;        //上次期望功率
extern float power_actual;       //实际功率
extern float power_last;         //上次功率
extern float Power_Inc;         //功率增量
extern float current_expect;     //期望电流
extern float bili;             //电流分配比例
extern u16 charge_data;

extern float power_buffer_real;      //实时缓冲能量
extern float power_buffer_threshold; //缓冲能量阈值
extern float power_target;           //目标功率 根据当前缓冲能量而不断变化
extern u8 cap_status ;
void Capacity_Data_Send(void);
void Power_Limit(float power_actual_target);


#endif

