#ifndef _MOTORHEAD_H_
#define _MOTORHEAD_H_
#include "stm32f4xx_hal.h"
#include "sys.h"
#include "pid.h"
//������ֵ�ǲ������  ǰ���7500��5000��������ֵ ��Ҫ���ڹ����ļ��
//#define       YAW_INITIAL_VALUE       3695 //Yaw���ʼֵ
//#define      PITCH_INITIAL_VALUE     6450 //Pitch���ʼֵ
typedef struct{
volatile	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
volatile	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
volatile	float ecd_angle;											//�Ƕ�
volatile float rad_angle;                        //����
volatile float sina;                        
volatile float cosa;     
  int32_t torque;                     //����ֵ	
	int32_t temperature;
}Encoder;


extern Encoder mechanical_angle_PTZ_P;
extern Encoder mechanical_angle_PTZ_Y;
extern volatile Encoder Dial_the_motor;

extern int Dial_motor_speed_ref;
extern int	Dial_motor_speed_fdb;
extern uint8_t prepare_flag;
extern int32_t position_hit;    //��remote���� ���Ʋ��������ת���Ƕ� ��ң�����ͼ��̿���
extern int16_t yaw_middle_value;
extern float raw_angle;                  //�����ٶȷֽ�ı���
extern int yaw_motor_speed_ref;       //Y�����Ƕ�
extern int yaw_motor_speed_ref_filter;       //Y�����Ƕ�
extern int pitch_motor_speed_ref;            //P�������ٶ�
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
