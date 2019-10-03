/***���̵�����������**/
#include "stm32f4xx_hal.h"
#include "include.h"
int Twist_delay = 0;                //Ť����ʱ
static fp32 twist_angle=0;          //Ť���Ƕ�
float power_offset=0;
int expect_sum=0;        //���ڼ���4������ĵ���֮��       
float Current_Motor14;   //��������������֮��
float Current_Motor23;
float Current_Motor1234; //�ܵ���
volatile float y_ecdangsav = 0;     //Ť��������Y��ǶȻ������
extern float changehead_rorate;
float rotate_k=NORMAL_ROTATE_K;                   //ת��ϵ��
int rorate_slow=0;
int twist_slow=0;
POWER_CONTROL_STATUS power_control_status=POWER_NORMAL;
CHASSIS_WALK_SPEED Chassis_Walk_Speed=NORMAL_SPEED;
ChassisSpeed_Ref_t          Chassis_Speed_Ref;                    //ң��������
Chassis_Motor_PID_Expect    Chassis_Motor_PID_Ref;                //���̵������ֵ                                                                                                                                    
																				                               //  1\      2/  �����ķ�ְڷ�λ��																
Chassis_Motor_Actual_Speed  Chassis_Motor_Actual_Speed_ref;    	       //  3/      4\

float chassis_wz_set_scale=0.0f;

void PID_Expect()                            //�õ����̵�����������ٶ�    ���õ��������ķ�� �������� ��������
{
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_1 = Chassis_Speed_Ref.forward_back_ref +
																										 Chassis_Speed_Ref.left_right_ref + Chassis_Speed_Ref.rotate_ref*(1.0f+chassis_wz_set_scale);
	
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_2 =  -Chassis_Speed_Ref.forward_back_ref+
																										 Chassis_Speed_Ref.left_right_ref +Chassis_Speed_Ref.rotate_ref*(1.0f+chassis_wz_set_scale);
	
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_3 = Chassis_Speed_Ref.forward_back_ref -
	                                                   Chassis_Speed_Ref.left_right_ref + Chassis_Speed_Ref.rotate_ref*(1.0f-chassis_wz_set_scale);

	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_4 = -Chassis_Speed_Ref.forward_back_ref-
																										 Chassis_Speed_Ref.left_right_ref+ Chassis_Speed_Ref.rotate_ref*(1.0f-chassis_wz_set_scale);
	if(key_mouse_inf.ctrl_flag&&!key_mouse_inf.z_flag)   //��ʼ���� ���ڲ�������
	{
		Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_1*=0.5;
		Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_2*=0.5;
		Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_3*=0.5;
		Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_4*=0.5;
	}
}

void Chassis_Speed_Ref_Init()
{
	Chassis_Speed_Ref.forward_back_ref=0;
	Chassis_Speed_Ref.left_right_ref=0;
	Chassis_Speed_Ref.rotate_ref=0;
	
	GimbalRef.pitch_angle_dynamic_ref=0;
	GimbalRef.yaw_angle_dynamic_ref=0;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Chassis_Motor_Control(void)
*��������:	 ���̵������
���������   û��
���������   û��
*******************************************************************************/
void Chassis_Motor_Control(void)
{
	if(GetInputMode()==OFFLINE)  //ң��������
	{
		Chassis_Speed_Ref.forward_back_ref=0;
    Chassis_Speed_Ref.left_right_ref=0;
	}

	PID_Expect();

  PID_Control(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1 ,
									Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_1,&Chassis_Motor_PID_1);
	
	PID_Control(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2 ,
									Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_2,&Chassis_Motor_PID_2);
	
	PID_Control(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3 ,
									Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_3,&Chassis_Motor_PID_3);
	
	PID_Control(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4 ,
									Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_4,&Chassis_Motor_PID_4);//���̵����PID

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Chassis_Rotate_Control(void)
*��������:	 ����Ť�� ת�����
���������   û��
���������   û��
*******************************************************************************/
void Chassis_Rotate_Control(void)
{
	if(rotate_revive_flag==1)  //����ǰ����
	{
		//Ŀ���Ǹ������
		if(mechanical_angle_PTZ_Y.raw_value<YAW_INITIAL_VALUE)
		{
			Chassis_Speed_Ref.rotate_ref=TO_MIDDLE_SPEED;   //���ù����ٶ�
		}
		else if(mechanical_angle_PTZ_Y.raw_value>YAW_INITIAL_VALUE)
		{
			Chassis_Speed_Ref.rotate_ref=-TO_MIDDLE_SPEED;   //���ù����ٶ�
		}
		
		if(__fabs(mechanical_angle_PTZ_Y.raw_value)>=(YAW_INITIAL_VALUE-10)&&mechanical_angle_PTZ_Y.raw_value<=(YAW_INITIAL_VALUE+10))
		{
			yaw_middle_value=mechanical_angle_PTZ_Y.raw_value;
			rotate_revive_flag=0;
			can_count_y=0;   //���¹���
			PTZ_Parameter_Init(&mechanical_angle_PTZ_Y);
		}
	}
	else{
	twist_angle=Set_twist_angle(TWIST_NO_MOVE_MAX_ANGLE,TWIST_MAX_T);   //����Ť���Ƕ�  Ĭ�� 55�� 1500ms
	if(!key_mouse_inf.no_twisted&&key_mouse_inf.save_flag)  //��¼Ť��ǰ��Y�����Ƕ�
	{
		y_ecdangsav= - mechanical_angle_PTZ_Y.ecd_angle;
	}
		if(key_mouse_inf.rotate==1)  //����
		{
			Chassis_Speed_Ref.rotate_ref=NORMAL_SPEED;
			if(CAP_volt<RT_CAP_TAL_THRESHOLD)
				rorate_slow=1;
			else
				rorate_slow=0;
		}
		else if((key_mouse_inf.rotate==0&&key_mouse_inf.rotate_to_middle==0))  //������ ������δ���
		{
			if(mechanical_angle_PTZ_Y.raw_value<YAW_INITIAL_VALUE)
			{
				Chassis_Speed_Ref.rotate_ref=Chassis_Speed_Ref.rotate_ref;   //���ù����ٶ�
			}
			else if(mechanical_angle_PTZ_Y.raw_value>YAW_INITIAL_VALUE)
			{
				Chassis_Speed_Ref.rotate_ref=--Chassis_Speed_Ref.rotate_ref;   //���ù����ٶ�
			}
			if(__fabs(mechanical_angle_PTZ_Y.raw_value)>=(YAW_INITIAL_VALUE-10)&&mechanical_angle_PTZ_Y.raw_value<=(YAW_INITIAL_VALUE+10))
				{
					yaw_middle_value=mechanical_angle_PTZ_Y.raw_value;
					key_mouse_inf.rotate=0;
					key_mouse_inf.rotate_to_middle=1; //�������
					can_count_y=0;   //���¹���
					PTZ_Parameter_Init(&mechanical_angle_PTZ_Y);
				}
		}
		else if(key_mouse_inf.no_twisted==0)  //Ť�� ��������
		{
			if(CAP_volt<RT_CAP_TAL_THRESHOLD)
				twist_slow=1;
			else
				twist_slow=0;
			key_mouse_inf.save_flag = 0;
			PID_Control(mechanical_angle_PTZ_Y.ecd_angle, y_ecdangsav + twist_angle, &Chassis_Motor_PID_rotate);
			kalman_filter_calc(&chassis_rotate_filter,Chassis_Motor_PID_rotate.ec,Chassis_Motor_PID_rotate.ec);
			Chassis_Motor_PID_rotate.pid_filter_out=Chassis_Motor_PID_rotate.Kpout+Chassis_Motor_PID_rotate.Kd*chassis_rotate_filter.filtered_value[0]+Chassis_Motor_PID_rotate.Kiout;
			Chassis_Speed_Ref.rotate_ref=3.2*Chassis_Motor_PID_rotate.pid_filter_out;                   
		}								
		else if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1) //�������� ��Ť��
		{//��Ϊ�� ��Ϊ��
		  PID_Control(mechanical_angle_PTZ_Y.ecd_angle, 0+changehead_rorate, &Chassis_Motor_PID_rotate);
			kalman_filter_calc(&chassis_rotate_filter,Chassis_Motor_PID_rotate.ec,Chassis_Motor_PID_rotate.ec);
			Chassis_Motor_PID_rotate.pid_filter_out=Chassis_Motor_PID_rotate.Kpout+Chassis_Motor_PID_rotate.Kd*chassis_rotate_filter.filtered_value[0]+Chassis_Motor_PID_rotate.Kiout;
			Chassis_Speed_Ref.rotate_ref=rotate_k*Chassis_Motor_PID_rotate.pid_filter_out;	
			if(__fabs(Chassis_Speed_Ref.rotate_ref)<60)
			{
				Chassis_Speed_Ref.rotate_ref=0;
			}
		}
	}
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Chassis_Power_Control(void)
*��������:	 ���̹��ʿ���
���������   û��
���������   û��
*******************************************************************************/
void Chassis_Power_Control(void)
{
	static int power_limit_value=0;
	if(key_mouse_inf.shift_flag==0||(CAP_volt<CAP_TAL_THRESHOLD&&CAP_volt>0)||rorate_slow==1||twist_slow==1||CAP_volt==0) //��������δ������߳������ݼ��뵫shiftδ����ʱ�������ʻ�
	{
		power_limit_value=POWER_NORMAL_LIMIT_VALUE;
		Current_Motor14=__fabs(Chassis_Motor_PID_1.pid_out)+__fabs(Chassis_Motor_PID_4.pid_out);
		Current_Motor23=__fabs(Chassis_Motor_PID_2.pid_out)+__fabs(Chassis_Motor_PID_3.pid_out);

		Current_Motor1234=Current_Motor14+Current_Motor23;
		if(Current_Motor1234<=20)
			expect_sum=0;     //������������
		else 
			expect_sum=Current_Motor1234;   //�ĸ�����ĵ���֮��
		
	  #if CHASSIS_MOTOR_TYPE==M3510
		 power_actual=actual_current*ext_power_heat_data.chassis_volt;   //����ʵ�ʹ��� �ɼ�����ֵ
	  #elif CHASSIS_MOTOR_TYPE==M3508
		 power_actual=Get_Power();
	  #endif
		if((power_actual>power_limit_value||Power_Inc>POWER_INCREASE)) 
		{
			power_expect=Last_expect+PID_Increment(power_actual,power_limit_value,&Power_limit); //����ʽ���㹦������ֵ
			if(power_expect<0)       
			{
				power_expect=0;  
			}
			if(ext_power_heat_data.chassis_volt!=0)
				current_expect=power_expect/ext_power_heat_data.chassis_volt;
			else
				current_expect=power_expect/POWER_VOLTAGE;
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
	}
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Chassis_Data_Send(void)
*��������:	 ���̵����������
���������   û��
���������   û��
*******************************************************************************/							
void Chassis_Data_Send(void)
{
	VAL_LIMIT(Chassis_Motor_PID_1.pid_out,-CHASSIS_MAX_OUT,CHASSIS_MAX_OUT);
	VAL_LIMIT(Chassis_Motor_PID_2.pid_out,-CHASSIS_MAX_OUT,CHASSIS_MAX_OUT);
	VAL_LIMIT(Chassis_Motor_PID_3.pid_out,-CHASSIS_MAX_OUT,CHASSIS_MAX_OUT);
	VAL_LIMIT(Chassis_Motor_PID_4.pid_out,-CHASSIS_MAX_OUT,CHASSIS_MAX_OUT);
	
	//1000 
	databuf[0]=Chassis_Motor_PID_1.pid_out>>8;
	databuf[1]=Chassis_Motor_PID_1.pid_out;
	
	databuf[2]=Chassis_Motor_PID_2.pid_out>>8;
	databuf[3]=Chassis_Motor_PID_2.pid_out;
	
	databuf[4]=Chassis_Motor_PID_3.pid_out>>8;
	databuf[5]=Chassis_Motor_PID_3.pid_out;
	
	databuf[6]=Chassis_Motor_PID_4.pid_out>>8;
	databuf[7]=Chassis_Motor_PID_4.pid_out;

#if DEBUG_MODE==0
	taskENTER_CRITICAL();
		(*Chaais_Send)(databuf);
	taskEXIT_CRITICAL();
#endif
}

/** 
 *@brief ��ȡÿ������Ĺ���
 *@param ���ת�� ת�ص���
 */
float Get_chassis_motor_power(int speed,int current )   //��ת�ص����õ�����ֵ
{  
	float power;
	power=power_offset/4.0f+            //���̾�ֹʱ ����ϵͳ����/4
	      1.571e-7*speed*speed+
	      2.248e-6*speed*current+
	      2.022e-8*current*current;
	kalman_filter_calc(&power_filter,power,power);
	return power_filter.filtered_value[0];  //���ع���ֵ		
}

/**
 *@brief ��ȡ�����ܹ��ʣ���װ����
 *@return �����ܹ���
 */
float Get_Power(void)
{
	float power;
	power=Get_chassis_motor_power(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1,Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_1) \
	      +Get_chassis_motor_power(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2,Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_2) \
	      +Get_chassis_motor_power(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3,Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_3) \
	      +Get_chassis_motor_power(Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4,Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_4) ;
	kalman_filter_calc(&power_filter,power,ext_power_heat_data.chassis_power);

	if(power_filter.filtered_value[0]<0)
		return power_offset;
	else
		return power_filter.filtered_value[0];
}

/**
 * @param Ť����ֵ(PI)>=PI/10  ���ڣ�ms��Ť��ģʽ ���Ť�� ����Ť�� ���Ť������ͣ
 * @return Ť���Ƕ�
 * A=TWIST_NO_MOVE_ANGLE  W=250.0f
 * Set_twist_angle(TWIST_NO_MOVE_ANGLE,250.0f)
 */
float Set_twist_angle(float A,float T)
{
	//ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
	static fp32 swing_time = 0.0f;
	//swing_angle �Ǽ�������ĽǶ�
	static fp32 swing_angle = 0.0f;
	//max_angle ��sin�����ķ�ֵ
	static fp32 max_angle = TWIST_NO_MOVE_MAX_ANGLE;
	//add_time ��ҡ�ڽǶȸı�Ŀ�����Խ��Խ��
	volatile static fp32 add_time = PI/(TWIST_MIN_T/2.0f);
  
	if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate==0)
	{
		swing_time=0;
		swing_angle=0;
	}
	else	
	{
		//sin����������2pi
		//�������߹����� ������Ҫ��ת���ʱ�� �ر����Ť�� ��СŤ������
		if (Chassis_Speed_Ref.forward_back_ref>500||Chassis_Speed_Ref.left_right_ref>500||__fabs(RC_CtrlData.mouse.x)>10)
		{
			max_angle = TWIST_MOVE_ANGLE;
			if (swing_time > 2 * PI)
			{
					swing_time -= 2 * PI;
			}
		}
		else
		{
				max_angle = TWIST_NO_MOVE_MAX_ANGLE;
				if (swing_time > 2 * PI)
				{
						swing_time -= 2 * PI;
						//Set_random_twist(&max_angle,&add_time,A,T);        //ÿ��һ�����ڱ任һ��Ť�����Ⱥ�Ƶ��
				}
		}
		
		swing_angle = max_angle * arm_sin_f32(swing_time);  //����ת���ɽǶ�
		swing_time += PI/(T/2.0f);;  //�㶨Ƶ��  800ms
	}
	return swing_angle;
}

/**
 *@brief ����Ť��������ں͸�ֵ
 *@param Ť����ֵ Ť������ ���ֵ ��Сֵ����
 */
void Set_random_twist(float *A,float *T,float A_max,float T_max)
{
	*A=set_random_float(TWIST_NO_MOVE_MIN_ANGLE,A_max);
	*T=PI/(set_random_float(TWIST_MIN_T,T_max)/2.0f);
}
