#include "stm32f4xx_hal.h"
#include "include.h"
extern volatile float temp_PTZ_Position_P,temp_PTZ_Speed_P;
extern volatile float temp_PTZ_Position_Y,temp_PTZ_Speed_Y;
int32_t position_hit;
Encoder mechanical_angle_PTZ_P;
Encoder mechanical_angle_PTZ_Y;
uint8_t prepare_flag=0;                   //云台电机初始化（位置归中）标志int32_t position_hit = 0;
volatile Encoder Dial_the_motor;

int Dial_motor_speed_ref = 0;    //拨弹电机的期望速度
int Dial_motor_speed_fdb = 0;    //拨弹电机的实际速度
int yaw_motor_speed_ref=0;       //Y轴电机角速度
int yaw_motor_speed_ref_filter=0;   
int pitch_motor_speed_ref=0;            //P轴电机角速度
int pitch_motor_speed_ref_filter=0;   
int16_t yaw_middle_value=YAW_INITIAL_VALUE;

void GetEncoderBias_Y(volatile Encoder *v)   //初始化时使用
{
	v->ecd_bias = yaw_middle_value;//YAW电机初值  将中值设定为初始偏差
	v->ecd_value = v->ecd_bias;
	v->raw_value=v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
}

void GetEncoderBias_P(volatile Encoder *v)
{
 // mechanical_angle_PTZ_Y.raw_value;
	//mechanical_angle_PTZ_P.raw_value;
//	v->ecd_bias = PITCH_INITIAL_VALUE/*Slope(20000,PITCH)*/;//Pitch电机初值   将中值设定为初始偏差
	v->ecd_value = v->ecd_bias;
	v->raw_value=v->ecd_bias;
	v->last_raw_value = v->ecd_bias;  
           
}
void GetEncoderBias(volatile Encoder *v)
{

	v->ecd_bias = (CAN_Message.CAN1_Data[0]<<8)|CAN_Message.CAN1_Data[1];  //保存初始编码器值作为偏差  
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
           
}

/*以下程序是把反馈量是编码值的电机转化为速度，需要用的电机有6623,6500*/
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
	if(v->diff <-7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		if(!prepare_flag)
			v->round_cnt++;    
	}
	else if(v->diff>7500)
	{
		if(!prepare_flag)
			v->round_cnt--; 
	}	
	if(v->raw_value-v->ecd_bias<-5000&&prepare_flag==1)//YAW云台电机位置经过电机零点  解决电机归中过程中经过零点的现象 防止出错 
  {
	 
		v->raw_value+=8192; //将当前值加一圈

  }
  else if (v->raw_value-v->ecd_bias>5000&&prepare_flag==1)//YAW云台电机位置经过电机零点 
  {
    v->raw_value-=8192;  //将当前值减一圈
  }
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;//计算总的经过的所有编码值
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;//将编码值转换为角度
	
}

void EncoderProcessHIT(volatile Encoder *v)   //拨弹电机的编码计算
{
	v->last_raw_value = v->raw_value;
	v->raw_value = (CAN_Message.CAN1_Data[0]<<8)|CAN_Message.CAN1_Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -5500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
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
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;  //计算总的经过的所有编码值
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;  //将编码值转换为角度
		
}
/*************计算正弦和余弦值*****************/
/*输入：编码器连续角度值                      */
/*输出：无输出                                */
/*功能：计算sin(raw_angle) cos(raw_angle)     */
void GetY_sin_cos(volatile Encoder *v)
{
	v->rad_angle=(v->ecd_angle/180.0f)*PI;
	v->sina=arm_sin_f32(v->rad_angle);
	v->cosa=arm_cos_f32(v->rad_angle);
}

/************云台参数归零**************/
void PTZ_Parameter_Init(volatile Encoder *v)
{
	
	v->raw_value=0;   									//编码器不经处理的原始值
	v->last_raw_value=0;								//上一次的编码器原始值
	v->ecd_value=0;                       //经过处理后连续的编码器值
	v->diff=0;													//两次编码器之间的差值
	v->ecd_bias=0;											//初始编码器值	
	v->ecd_angle=0;	
	v->ecd_raw_rate=0;
	v->round_cnt=0;
}

/*              云台堵转保护函数                          */
//PTZ:保护电机类型 Torque_now：当前电流 
//Torque_max：最大电流 tim：计数器
//备注 当温度过高时也会关闭电机
void safe_troque(PID *P, volatile Encoder *v,int Torque_max, int tim)
{
	static int cnt_ms=0;
	if(__fabs(v->torque)>=Torque_max)
	{
		cnt_ms++;
	}
	else
		cnt_ms=0;
	if(cnt_ms>=tim/*||v->temperature>=40*/)//扭矩持续过大或者温度过高
	{
		P->Kd=0;
		P->Ki=0;
		P->Kp=0;
	}
}




