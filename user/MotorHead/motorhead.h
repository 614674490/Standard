#ifndef _MOTORHEAD_H_
#define _MOTORHEAD_H_
#include "stm32f4xx_hal.h"
#include "sys.h"
#include "pid.h"
//这两个值是测出来的  前面的7500和5000是随便给的值 主要用于过零点的检测
//#define       YAW_INITIAL_VALUE       3695 //Yaw轴初始值
//#define      PITCH_INITIAL_VALUE     6450 //Pitch轴初始值
typedef struct{
volatile	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
volatile	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
volatile	float ecd_angle;											//角度
volatile float rad_angle;                        //弧度
volatile float sina;                        
volatile float cosa;     
  int32_t torque;                     //电流值	
	int32_t temperature;
}Encoder;


extern Encoder mechanical_angle_PTZ_P;
extern Encoder mechanical_angle_PTZ_Y;
extern volatile Encoder Dial_the_motor;

extern int Dial_motor_speed_ref;
extern int	Dial_motor_speed_fdb;
extern uint8_t prepare_flag;
extern int32_t position_hit;    //在remote函数 控制拨弹电机的转动角度 由遥控器和键盘控制
extern int16_t yaw_middle_value;
extern float raw_angle;                  //用于速度分解的变量
extern int yaw_motor_speed_ref;       //Y轴电机角度
extern int yaw_motor_speed_ref_filter;       //Y轴电机角度
extern int pitch_motor_speed_ref;            //P轴电机角速度
extern int pitch_motor_speed_ref_filter;   
void GetEncoderBias(volatile Encoder *v);
void EncoderProcess(volatile Encoder *v);
void EncoderProcessHIT(volatile Encoder *v);
void GetEncoderBias_Y(volatile Encoder *v);
void GetEncoderBias_P(volatile Encoder *v);
void GetY_sin_cos(volatile Encoder *v);
void PTZ_Parameter_Init(volatile Encoder *v);
void safe_troque(PID *PTZ, volatile Encoder *v,int Torque_max, int tim);

#endif
