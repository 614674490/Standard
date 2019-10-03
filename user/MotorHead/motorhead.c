#include "stm32f4xx_hal.h"
#include "include.h"
extern volatile float temp_PTZ_Position_P,temp_PTZ_Speed_P;
extern volatile float temp_PTZ_Position_Y,temp_PTZ_Speed_Y;
int32_t position_hit;
Encoder mechanical_angle_PTZ_P;
Encoder mechanical_angle_PTZ_Y;
uint8_t prepare_flag=0;                   //��̨�����ʼ����λ�ù��У���־int32_t position_hit = 0;
volatile Encoder Dial_the_motor;

int Dial_motor_speed_ref = 0;    //��������������ٶ�
int Dial_motor_speed_fdb = 0;    //���������ʵ���ٶ�
int yaw_motor_speed_ref=0;       //Y�������ٶ�
int yaw_motor_speed_ref_filter=0;   
int pitch_motor_speed_ref=0;            //P�������ٶ�
int pitch_motor_speed_ref_filter=0;   
int16_t yaw_middle_value=YAW_INITIAL_VALUE;

void GetEncoderBias_Y(volatile Encoder *v)   //��ʼ��ʱʹ��
{
	v->ecd_bias = yaw_middle_value;//YAW�����ֵ  ����ֵ�趨Ϊ��ʼƫ��
	v->ecd_value = v->ecd_bias;
	v->raw_value=v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
}

void GetEncoderBias_P(volatile Encoder *v)
{
 // mechanical_angle_PTZ_Y.raw_value;
	//mechanical_angle_PTZ_P.raw_value;
//	v->ecd_bias = PITCH_INITIAL_VALUE/*Slope(20000,PITCH)*/;//Pitch�����ֵ   ����ֵ�趨Ϊ��ʼƫ��
	v->ecd_value = v->ecd_bias;
	v->raw_value=v->ecd_bias;
	v->last_raw_value = v->ecd_bias;  
           
}
void GetEncoderBias(volatile Encoder *v)
{

	v->ecd_bias = (CAN_Message.CAN1_Data[0]<<8)|CAN_Message.CAN1_Data[1];  //�����ʼ������ֵ��Ϊƫ��  
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
           
}

/*���³����ǰѷ������Ǳ���ֵ�ĵ��ת��Ϊ�ٶȣ���Ҫ�õĵ����6623,6500*/
void EncoderProcess(volatile Encoder *v)
{
	if(soft_js_time>=1500&&soft_js_time<=4200)   
	{
		prepare_flag=1;
	}
	else
	{
		prepare_flag=0;
	}
	v->last_raw_value = v->raw_value;
	v->raw_value = (CAN_Message.CAN1_Data[0]<<8)|CAN_Message.CAN1_Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff <-7500)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		if(!prepare_flag)
			v->round_cnt++;    
	}
	else if(v->diff>7500)
	{
		if(!prepare_flag)
			v->round_cnt--; 
	}	
	if(v->raw_value-v->ecd_bias<-5000&&prepare_flag==1)//YAW��̨���λ�þ���������  ���������й����о����������� ��ֹ���� 
  {
	 
		v->raw_value+=8192; //����ǰֵ��һȦ

  }
  else if (v->raw_value-v->ecd_bias>5000&&prepare_flag==1)//YAW��̨���λ�þ��������� 
  {
    v->raw_value-=8192;  //����ǰֵ��һȦ
  }
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;//�����ܵľ��������б���ֵ
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;//������ֵת��Ϊ�Ƕ�
	
}

void EncoderProcessHIT(volatile Encoder *v)   //��������ı������
{
	v->last_raw_value = v->raw_value;
	v->raw_value = (CAN_Message.CAN1_Data[0]<<8)|CAN_Message.CAN1_Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -5500)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>5500)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;  //�����ܵľ��������б���ֵ
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;  //������ֵת��Ϊ�Ƕ�
		
}
/*************�������Һ�����ֵ*****************/
/*���룺�����������Ƕ�ֵ                      */
/*����������                                */
/*���ܣ�����sin(raw_angle) cos(raw_angle)     */
void GetY_sin_cos(volatile Encoder *v)
{
	v->rad_angle=(v->ecd_angle/180.0f)*PI;
	v->sina=arm_sin_f32(v->rad_angle);
	v->cosa=arm_cos_f32(v->rad_angle);
}

/************��̨��������**************/
void PTZ_Parameter_Init(volatile Encoder *v)
{
	
	v->raw_value=0;   									//���������������ԭʼֵ
	v->last_raw_value=0;								//��һ�εı�����ԭʼֵ
	v->ecd_value=0;                       //��������������ı�����ֵ
	v->diff=0;													//���α�����֮��Ĳ�ֵ
	v->ecd_bias=0;											//��ʼ������ֵ	
	v->ecd_angle=0;	
	v->ecd_raw_rate=0;
	v->round_cnt=0;
}

/*              ��̨��ת��������                          */
//PTZ:����������� Torque_now����ǰ���� 
//Torque_max�������� tim��������
//��ע ���¶ȹ���ʱҲ��رյ��
void safe_troque(PID *P, volatile Encoder *v,int Torque_max, int tim)
{
	static int cnt_ms=0;
	if(__fabs(v->torque)>=Torque_max)
	{
		cnt_ms++;
	}
	else
		cnt_ms=0;
	if(cnt_ms>=tim/*||v->temperature>=40*/)//Ť�س�����������¶ȹ���
	{
		P->Kd=0;
		P->Ki=0;
		P->Kp=0;
	}
}




