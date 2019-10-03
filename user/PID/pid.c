#include "stm32f4xx_it.h"
#include "include.h"

PID Chassis_Motor_PID_1,Chassis_Motor_PID_2,Chassis_Motor_PID_3,Chassis_Motor_PID_4,Power_PID;   
PID Hit_speed,Hit_position;            
PID Chassis_Motor_PID_rotate;         
PID PTZ_Motor_PID_Position_Y,PTZ_Motor_PID_Speed_Y,PTZ_Motor_PID_Position_P,PTZ_Motor_PID_Speed_P; 
PID_ADD Power_limit;
PID_ADD MPU_temp;
float mpu_pwm=0;

//ǰ�������� ���������ֵƫ�����
/*����ǰ���������Ľṹ��*/
typedef struct{
 float rin;
 float lastRin;
 float perrRin;
 float alpha,beta;
 float result;
}FFC;
 
/*ʵ��ǰ��������*/
float FeedforwardController(FFC *vFFC,float setdata)
{
  float result;
 
	vFFC->rin=setdata;
  vFFC->result=vFFC->alpha*(vFFC->rin-vFFC->lastRin)+vFFC->beta*(vFFC->rin-2*vFFC->lastRin+vFFC->perrRin);
  vFFC->perrRin= vFFC->lastRin;
  vFFC->lastRin= vFFC->rin;
  return result;
}


void PID_Init(void)  
{
	//���̵��  2.2 0.036 0     3.5 0.02 2
	Chassis_Motor_PID_1.error_last=0;
	Chassis_Motor_PID_1.error_now=0;
	Chassis_Motor_PID_1.error_inter=0;
	Chassis_Motor_PID_1.Kp=CHASSIS_MOTOR_KP;
	Chassis_Motor_PID_1.Ki=CHASSIS_MOTOR_KI;  
	Chassis_Motor_PID_1.Kd=CHASSIS_MOTOR_KD;
	Chassis_Motor_PID_1.current_time=0;
	Chassis_Motor_PID_1.last_time=0;
	Chassis_Motor_PID_1.time=0;
	Chassis_Motor_PID_1.inter_limit=10000;
	
	Chassis_Motor_PID_2.error_last=0;
	Chassis_Motor_PID_2.error_now=0;
	Chassis_Motor_PID_2.error_inter=0;
	Chassis_Motor_PID_2.Kp=CHASSIS_MOTOR_KP;      
	Chassis_Motor_PID_2.Ki=CHASSIS_MOTOR_KI;  
	Chassis_Motor_PID_2.Kd=CHASSIS_MOTOR_KD;
	Chassis_Motor_PID_2.current_time=0;
	Chassis_Motor_PID_2.last_time=0;
	Chassis_Motor_PID_2.time=0;
	Chassis_Motor_PID_2.inter_limit=10000;
	
	Chassis_Motor_PID_3.error_last=0;
	Chassis_Motor_PID_3.error_now=0;
	Chassis_Motor_PID_3.error_inter=0;
	Chassis_Motor_PID_3.Kp=CHASSIS_MOTOR_KP;      
	Chassis_Motor_PID_3.Ki=CHASSIS_MOTOR_KI;  
	Chassis_Motor_PID_3.Kd=CHASSIS_MOTOR_KD;
	Chassis_Motor_PID_3.current_time=0;
	Chassis_Motor_PID_3.last_time=0;
	Chassis_Motor_PID_3.time=0;
	Chassis_Motor_PID_3.inter_limit=10000;
	
	Chassis_Motor_PID_4.error_last=0;
	Chassis_Motor_PID_4.error_now=0;
	Chassis_Motor_PID_4.error_inter=0;
	Chassis_Motor_PID_4.Kp=CHASSIS_MOTOR_KP;      
	Chassis_Motor_PID_4.Ki=CHASSIS_MOTOR_KI;  
	Chassis_Motor_PID_4.Kd=CHASSIS_MOTOR_KD;
	Chassis_Motor_PID_4.current_time=0;
	Chassis_Motor_PID_4.last_time=0;
	Chassis_Motor_PID_4.time=0;
	Chassis_Motor_PID_4.inter_limit=10000;
	
	//λ��
	Hit_position.error_last=0;
	Hit_position.error_now=0;
	Hit_position.error_inter=0;
	Hit_position.Kp=HIT_POSITION_KP;       //1.2 0.01 0.15
	Hit_position.Ki=HIT_POSITION_KI;
	Hit_position.Kd=HIT_POSITION_KD;    //0.15
	Hit_position.current_time=0;
	Hit_position.last_time=0;
	Hit_position.time=0;
  Hit_position.inter_limit=10000;
	//�������   �ٶ�
	Hit_speed.error_last=0;
	Hit_speed.error_now=0;
	Hit_speed.error_inter=0;
	Hit_speed.Kp=HIT_SPEED_KP;    //2.3 0.02 1   2
	Hit_speed.Ki=HIT_SPEED_KI;    
	Hit_speed.Kd=HIT_SPEED_KD; 
	Hit_speed.current_time=0;
	Hit_speed.last_time=0;
	Hit_speed.time=0;
	Hit_speed.inter_limit=10000;
	//ת��PID ң������70 0 4000   ����
	Chassis_Motor_PID_rotate.error_inter=0;
	Chassis_Motor_PID_rotate.error_last=0;
	Chassis_Motor_PID_rotate.error_now=0;
	Chassis_Motor_PID_rotate.Kd=2500.0f;                //2500 0 65
	Chassis_Motor_PID_rotate.Ki=0.00f;        
	Chassis_Motor_PID_rotate.Kp=65.0f; 
	Chassis_Motor_PID_rotate.current_time=0;
	Chassis_Motor_PID_rotate.last_time=0;
	Chassis_Motor_PID_rotate.time=0;
  Chassis_Motor_PID_rotate.inter_limit=10000;
	//15 0.05 150 
  //     100 0 1000   15 0 800   100 0 1500 10 0 200     0 0 10
	PTZ_Motor_PID_Position_Y.error_inter=0; 
	PTZ_Motor_PID_Position_Y.error_last=0;
	PTZ_Motor_PID_Position_Y.error_now=0;
	PTZ_Motor_PID_Position_Y.ec=0;
	PTZ_Motor_PID_Position_Y.Kd=0;   
	PTZ_Motor_PID_Position_Y.Ki=0;     
	PTZ_Motor_PID_Position_Y.Kp=35;  
	PTZ_Motor_PID_Position_Y.current_time=0;
	PTZ_Motor_PID_Position_Y.last_time=0;
	PTZ_Motor_PID_Position_Y.time=0;
	PTZ_Motor_PID_Position_Y.inter_limit=10000;

	//�ٶ�  1000 0 35    0 0.4 90
	PTZ_Motor_PID_Speed_Y.error_inter=0;
	PTZ_Motor_PID_Speed_Y.error_last=0;
	PTZ_Motor_PID_Speed_Y.error_now=0;
	PTZ_Motor_PID_Speed_Y.ec=0;
	PTZ_Motor_PID_Speed_Y.Kd=0; 
	PTZ_Motor_PID_Speed_Y.Ki=0.05;
  PTZ_Motor_PID_Speed_Y.Kp=120;
  PTZ_Motor_PID_Speed_Y.current_time=0;
	PTZ_Motor_PID_Speed_Y.last_time=0;
	PTZ_Motor_PID_Speed_Y.time=0;	
  PTZ_Motor_PID_Speed_Y.inter_limit=10000;
	
	
	//��̨��� P λ�� 
	PTZ_Motor_PID_Position_P.error_inter=0;//32 0.05 3
	PTZ_Motor_PID_Position_P.error_last=0;
	PTZ_Motor_PID_Position_P.error_now=0;
	PTZ_Motor_PID_Position_P.Kd=0;     
	PTZ_Motor_PID_Position_P.Ki=0;    
	PTZ_Motor_PID_Position_P.Kp=PTZ_MOTOR_PITCH_POSITION_KP;
	PTZ_Motor_PID_Position_P.current_time=0;
	PTZ_Motor_PID_Position_P.last_time=0;
	PTZ_Motor_PID_Position_P.time=0;	
  PTZ_Motor_PID_Position_P.inter_limit=10000;
	//�ٶ�
	PTZ_Motor_PID_Speed_P.error_inter=0;  //23 0 3
	PTZ_Motor_PID_Speed_P.error_last=0;
	PTZ_Motor_PID_Speed_P.error_now=0;
	PTZ_Motor_PID_Speed_P.Kd=0;
	PTZ_Motor_PID_Speed_P.Ki=0;
	PTZ_Motor_PID_Speed_P.Kp=PTZ_MOTOR_PITCH_SPEED_KP;
	PTZ_Motor_PID_Speed_P.current_time=0;
	PTZ_Motor_PID_Speed_P.last_time=0;
	PTZ_Motor_PID_Speed_P.time=0;	
  PTZ_Motor_PID_Speed_P.inter_limit=10000;

	//����ʽPID Ki->��Ӧ�ٶ�  Kp->���ͳ���
	Power_limit.error_last=0;  // ���¹����ȶ� 60W 1.5 1.3 0.5 70 20 80;   
	Power_limit.error_now=0;
	Power_limit.Kp=POWER_KP;
	Power_limit.Ki=POWER_KI;  
	Power_limit.Kd=POWER_KD;

	//�������¶�
	MPU_temp.expect=45;
	MPU_temp.error_last=0;
	MPU_temp.error_now=0;
	MPU_temp.Kp=6;
	MPU_temp.Ki=10;
	MPU_temp.Kd=5;
}

//λ��ʽPID
void PID_Control(float current, float expected,PID* pid)  //&motor_type
{
	pid->current_time=HAL_GetTick();
	pid->time=pid->current_time-pid->last_time;
	if(pid->time==0)  //��ĸ����Ϊ0
		pid->time=1;
	pid->error_last=pid->error_now;
	pid->error_now=expected-current;
  pid->ec=(pid->error_now-pid->error_last)/pid->time;
	pid->last_time=pid->current_time;
	pid->Kpout=pid->Kp * pid->error_now;
	pid->Kdout=pid->Kd *pid->ec;
	//���ڴ�������ϵͳ���������������Խ�������ȡ�������ø����������Ϳ��Խ���ִ�л����Ķ���������
	//���� ������ϵͳ�ڵ���������ر��ǽӽ���̬ʱ�������ȶ�����������ȱ�������ڡ��������Ĵ��ڣ����ھ��Ⱦ���Ӱ����  �������ھ��� �����ȶ���
	if(pid==&Chassis_Motor_PID_1||pid==&Chassis_Motor_PID_2||pid==&Chassis_Motor_PID_3||pid==&Chassis_Motor_PID_4||pid==&Chassis_Motor_PID_rotate)
	{
		volatile int index=1;   //���ڻ��ַ���
		if(pid->error_inter>pid->inter_limit)  //���ֳ������� ��ֻ�ۼӸ��Ļ�����
		{
			if(_fabsf(pid->error_now)>3000)
				index=0;      //ƫ�����ֵ ȡ������
			else 
			{
				index=1;    
				if(pid->error_now<0)
					pid->error_inter+=pid->error_now;	
			}
		}
		else if(pid->error_inter<-pid->inter_limit)   //���ֳ������� ��ֻ�ۼ����Ļ�����
		{
			if(_fabsf(pid->error_now)>3000)
				index=0;      //ƫ�����ֵ ȡ������
			else
			{
				index=1;
				if(pid->error_now>0)
					pid->error_inter+=pid->error_now;	
			}

		}
		else     //����û�г���������
		{
			if(_fabsf(pid->error_now)>3000)
				index=0;
			else
			{
				index=1;
				pid->error_inter+=pid->error_now;	
			}
		}
		pid->Kiout=pid->Ki * pid->error_inter;
		pid->pid_out=pid->Kpout+pid->Ki*pid->error_inter+ \
	                    pid->Kd*pid->ec;
	}

	else
	{
		VAL_LIMIT(pid->error_inter,-pid->inter_limit,pid->inter_limit);
		pid->error_inter+=pid->error_now;	
		pid->Kiout=pid->Ki * pid->error_inter;
//    ΢������
//		float c1=0,c2=0,c3=0,temp=0;  
//		if(pid==&PTZ_Motor_PID_Position_P||pid==&PTZ_Motor_PID_Speed_P)
//		{
//				temp=pid->gama*pid->Kd+pid->Kp;   //��Kd+Kp   ��ĸ��
//				c3=pid->Kd/temp;                  //Kd/(��Kd+Kp)       //������ϵ�� *y(K-1)  �ϴ�ʵ��ֵ
//				c2=(pid->Kd+pid->Kp)/temp;        //(Kd+Kp)/(��Kd+Kp)  //�ڶ���ϵ�� *y(K)    ��ǰʵ��ֵ
//				c1=pid->gama*c3;                  //��Kd/(��Kd+Kp)      //��һ��ϵ�� *Ud(K-1) ��һ��΢����
//				pid->derivative=c1*pid->last_derivative+c2*current+c3*pid->lastPv;
//				pid->lastPv=current;
//			  pid->last_derivative=pid->derivative;
//				pid->pid_out=pid->Kp*pid->error_now+pid->Ki*pid->error_inter+ \
//													pid->derivative;
//		}
//		else
			pid->pid_out=pid->Kpout+pid->Kiout+ \
	                    pid->Kdout;

	}		
}

//����ʽPID
float PID_Increment(float current,float expect,PID_ADD* PID)//���ʻ����Ƶ�PID����
{
     PID->error_now=expect-current;
	   PID->ec=PID->error_now-PID->error_next;
	   PID->increament=PID->Kp*(PID->error_now-PID->error_next)+PID->Ki*(PID->error_now)+
	                          PID->Kd*(PID->error_now-2*PID->error_next+PID->error_last);
	   
     PID->error_last=PID->error_next;
	   PID->error_next=PID->error_now;
	   
	  return PID->increament;
}
/**
  * @brief PID����޷�����������                    
  */
void PID_Limit_Out(PID *pid,int MaxOut)
{
	pid->pid_out=(int)pid->pid_out;
	if(pid->pid_out>MaxOut)
		pid->pid_out=MaxOut;
	else if(pid->pid_out<-MaxOut)
		pid->pid_out=-MaxOut;
}

void PID_Clear(void)
{

	Chassis_Motor_PID_1.Kp=0;
	Chassis_Motor_PID_1.Ki=0;  
	Chassis_Motor_PID_1.Kd=0;

	
	Chassis_Motor_PID_2.Kp=0;      
	Chassis_Motor_PID_2.Ki=0;  
	Chassis_Motor_PID_2.Kd=0;

	
	Chassis_Motor_PID_3.Kp=0;      
	Chassis_Motor_PID_3.Ki=0;  
	Chassis_Motor_PID_3.Kd=0;


	Chassis_Motor_PID_4.Kp=0;      
	Chassis_Motor_PID_4.Ki=0;  
	Chassis_Motor_PID_4.Kd=0;

	

	Hit_position.Kp=0;
	Hit_position.Ki=0;
	Hit_position.Kd=0;   


	Hit_speed.Kp=0;  
	Hit_speed.Ki=0;    
	Hit_speed.Kd=0; 


	Chassis_Motor_PID_rotate.Kd=0;                
	Chassis_Motor_PID_rotate.Ki=0;        
	Chassis_Motor_PID_rotate.Kp=0; 


	PTZ_Motor_PID_Position_Y.Kd=0;   
	PTZ_Motor_PID_Position_Y.Ki=0;     
	PTZ_Motor_PID_Position_Y.Kp=0;  

	


	PTZ_Motor_PID_Speed_Y.Kd=0;
	PTZ_Motor_PID_Speed_Y.Ki=0;
  PTZ_Motor_PID_Speed_Y.Kp=0;		


	PTZ_Motor_PID_Position_P.Kd=0;     
	PTZ_Motor_PID_Position_P.Ki=0;    
	PTZ_Motor_PID_Position_P.Kp=0;

	//�ٶ�

	PTZ_Motor_PID_Speed_P.Kd=0;
	PTZ_Motor_PID_Speed_P.Ki=0;
	PTZ_Motor_PID_Speed_P.Kp=0;
	
	
	Power_limit.Kp=0;
	Power_limit.Ki=0;  
	Power_limit.Kd=0;
}

void PID_Parment_Clear(PID *pid)
{
	pid->current_time=0;
	pid->last_time=0;
	pid->time=0;
	
	pid->ec=0;
	pid->error_inter=0;
	pid->error_last=0;
	pid->error_now=0;
	
	pid->Kdout=0;
	pid->Kiout=0;
	pid->Kpout=0;
}
void PID_ADD_Parment_Clear(PID_ADD *pid_add)
{
	pid_add->ec=0;
	pid_add->error_last=0;
	pid_add->error_next=0;
	pid_add->error_now=0;
}

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}


