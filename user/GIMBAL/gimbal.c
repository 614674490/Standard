/**
 * ���ģ�Z���������ٶ� ȡ��  temp_PTZ_Position_Y temp_PTZ_Speed_P  GimbalRef.yaw_angle_dynamic_refȡ��
 */
#include "FreeRTOS.h"
#include "task.h"
#include "include.h"
u8 friction_init_flag=0;  //Ħ���ֳ�ʼ����־λ ��ʼ��ʱ��һ ���ɶ�Ħ���ֽ�����������
float yaw_filter_angle=0;
float yaw_filter_gyro=0;
volatile float temp_PTZ_Position_P,temp_PTZ_Speed_P;
volatile float temp_PTZ_Position_Y,temp_PTZ_Speed_Y;
u32 expect_speed=0;
u8 gimbal_power=0;
int sin_data=0;
float expect_p;
int Period[60]={1000,667,500,400,333,286,250,222,200,182,    
	167,154,143,133,125,118,111,105,100,95,91,87,83,80,77,74,
	71,69,67,65,63,61,59,57,56,54,53,51,50,45,42,38,36,33,29,
  26,24,22,20,19,17,16,15,14,12,11,10,9,8,5};
float Omg[60]={0.0063,0.0094,0.0126,0.0157,0.0189,0.022,
	0.0251,0.0283,0.0314,0.0345,0.0376,0.0408,0.0439,0.0472,
	0.0502,0.0532,0.0566,0.0598,0.0628,0.0661,0.069,0.0722,
	0.0757,0.0785,0.0816,0.0849,0.0885,0.091,0.0937,0.0966,
	0.0997,0.103,0.1064,0.1102,0.1121,0.1163,0.1185,0.1231,
	0.1256,0.1396,0.1495,0.1653,0.1744,0.1903,0.2166,0.2415,
  0.2617,0.2855,0.314,0.3305,0.3694,0.3925,0.4187,0.4486,0.5233,
  0.5709,0.628,0.6978,0.785,1.256};

float e_z=0,z1=0,z2=0,z3=0,wc=0,volt_old=0,err_p=0,volt=0;	
void ESO_set(float cur_position,float expect)
{
	wc=200.0f;
	err_p=expect-z1;
	e_z=z1-cur_position;
	z1=z1+0.001f*(z2-(3*wc-16)*e_z);
	z2=z2+0.001f*(z3-(3*wc*wc-16*(3*wc-16))*e_z-16*z2+110*(volt_old));
	z3=z3+0.001f*(-wc*wc*wc*e_z);
	volt=9*(16*err_p+1*(-z2));//16������Ӧ��
	volt=(volt-z3)/50;	//110
	volt_old=volt;
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_ControlData_Get(void)
*��������:	 ϵͳ��ʶ->������ѡ�����ź�
���������  ���� Ƶ�� ��ֵ ��������
���������  ���Ҽ����ź�
	��ע     ÿ�����ڶ���Ӧ��ͬ��Ƶ��
*******************************************************************************/
int System_identification(int Period[],float Omg[],int amplitude,int sampletimes)
{
	//static int sin_data=0;
	static int time=0,P_count=0;
	static int Index=0;
	if(Index<60)
	{
		sin_data=(int)(sin((0.0063)*time)*amplitude);
			time++;
		if(time>=1000)
		{
			time=0;
			P_count++;
		}
		if(P_count>=sampletimes)  //ÿ��ɼ�30�� ��60��  ���ڸò��� �����Ӳ�������
		{
			P_count=0;
			Index++;
		}
	}
	else
	{
		 sin_data=0;
	}
	return sin_data;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_ControlData_Get(void)
*��������:	 ��̨�������ݻ�ȡ
���������   û��
���������   û��
��ע��      GM6020��6623�ķ���ֵ�෴ ע�������ŵ�ѡȡ ѡȡ�����ᵼ����̨��ת
*******************************************************************************/
void Gimbal_ControlData_Get(void)
{	
	#if MPU_STATUS==MPU_UPSIDE_DOWN
	temp_PTZ_Position_P = - mechanical_angle_PTZ_P.ecd_angle;  
	temp_PTZ_Position_Y =  mechanical_angle_PTZ_Y.ecd_angle; 
	temp_PTZ_Speed_P =  -MPU6500_Real_Data.Gyro_X ;  
	temp_PTZ_Speed_Y =  -MPU6500_Real_Data.Gyro_Z;

	#elif  MPU_STATUS==MPU_NORMAL
	temp_PTZ_Position_P =  -mechanical_angle_PTZ_P.ecd_angle;  
	temp_PTZ_Position_Y =  mechanical_angle_PTZ_Y.ecd_angle; 
	temp_PTZ_Speed_P =  MPU6500_Real_Data.Gyro_X ;  
	temp_PTZ_Speed_Y =  -MPU6500_Real_Data.Gyro_Z; 
	#elif MPU_STATUS==MPU_USART

	temp_PTZ_Position_P =  mechanical_angle_PTZ_P.ecd_angle;  
	temp_PTZ_Position_Y =  -mechanical_angle_PTZ_Y.ecd_angle; 
	temp_PTZ_Speed_P =  -gyro_usart_y ; 
	temp_PTZ_Speed_Y =  -gyro_usart_z;
	#endif
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_Init(void)
*��������:	 ��̨����
���������   û��
���������   û��
*******************************************************************************/
void Gimbal_Init(void)
{
		Gimbal_ControlData_Get();
		kalman_filter_calc(&yaw_pid_kalman_filter,PTZ_Motor_PID_Position_Y.ec,PTZ_Motor_PID_Speed_Y.ec);//����΢���˲�
	
		/******************************************Y�����**************************************************************/
		ramp=YAW_RAMP;                               //б�º����е�ö��
		PID_Control((temp_PTZ_Position_Y*Slope(65000,ramp)),GimbalRef.yaw_angle_dynamic_ref,&PTZ_Motor_PID_Position_Y);
		
		PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
				
		PID_Control(temp_PTZ_Speed_Y,PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//�ٶȻ�
		#if MPU_STATUS==MPU_USART
			PTZ_Motor_PID_Speed_Y.pid_filter_out=-(PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1]);
		#else
		PTZ_Motor_PID_Speed_Y.pid_filter_out=PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1];
		#endif
	
    /******************************************P�����*******************************************************************/
		ramp=PITCH_RAMP;   //б��
		PID_Control(pitch_usart_angle*Slope(65000,ramp),PTZ_Motor_PID_Position_P.Expect_Value,&PTZ_Motor_PID_Position_P);//λ�û�					
    PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//�ٶȻ�	

    VAL_LIMIT(PTZ_Motor_PID_Speed_Y.pid_filter_out,-YAW_MAX_OUT,YAW_MAX_OUT);
	  PID_Limit_Out(&PTZ_Motor_PID_Speed_P,PITCH_MAX_OUT);
		
	#if   DEBUG_MODE==1
				
				databuf_PTZ[0]=0;
				databuf_PTZ[1]=0;
				databuf_PTZ[2]=0;
				databuf_PTZ[3]=0;
	#else
	#if INFANTRY==3
				databuf_PTZ[0]=PTZ_Motor_PID_Speed_Y.pid_filter_out>>8;
				databuf_PTZ[1]=PTZ_Motor_PID_Speed_Y.pid_filter_out;
				
	#else
				databuf_PTZ[0]=-PTZ_Motor_PID_Speed_Y.pid_filter_out>>8;
				databuf_PTZ[1]=-PTZ_Motor_PID_Speed_Y.pid_filter_out;
	#endif
				databuf_PTZ[2]=PTZ_Motor_PID_Speed_P.pid_out>>8; 
				databuf_PTZ[3]=PTZ_Motor_PID_Speed_P.pid_out;
	#endif
				databuf_PTZ[4]=0;
				databuf_PTZ[5]=0;
				
				CAN1_Send_Msg_PTZ(databuf_PTZ); 

	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_Control(void)
*��������:	 ��̨����
���������   û��
���������   û��	
��ע��PID���Ʋ����е�ʵ��ֵ������ֵҪֱ�Ӹ�ֵ ��鸳ֵ�����Щ���ӳ� ������̨��ֹʱ����΢��
     ʵ��ֵ�ķ���Ҫ�ͳ�ʼ��ʱ��ʵ��ֵ���򱣳�һ�� ������̨�����һ���Ƕȵ���ת
     P����뿨�����˲��󣬵�����Ծ�ź�ʱ��Ӱ��Y��Ŀ������˲����
     
*******************************************************************************/
void Gimbal_Control(void)
{ 
	
	expect_p=(RC_CtrlData.rc.ch2-1024)/6;//2Y 3P

	Gimbal_ControlData_Get();
	#if AIMAUTO_STATUS
	kalman_filter_calc(&yaw_kalman_filter,y_anglesave,yaw_speed_raw);
  kalman_filter_calc(&dist_kalman_filter,dist_raw,dist_speed_raw);
  kalman_filter_calc(&pitch_kalman_filter,p_anglesave,pitch_speed_raw);
	#endif
	kalman_filter_calc(&yaw_pid_kalman_filter,PTZ_Motor_PID_Position_Y.ec,PTZ_Motor_PID_Speed_Y.ec);//����΢���˲�
	    #if MPU_STATUS==MPU_UPSIDE_DOWN
	    /***************************Y�����***********************************************************************/
//	    if(((RC_CtrlData.mouse.x==0&&GetInputMode()==KEY_MOUSE_INPUT)||(RC_CtrlData.rc.ch2==1024&&GetInputMode()==REMOTE_INPUT))&&key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate==0)  //�����п���ʱ ������̨
//			{
//				GimbalRef.yaw_angle_dynamic_ref=yaw_angle-angle_offset;
//				PID_Control(temp_PTZ_Position_Y,temp_PTZ_Position_Y,&PTZ_Motor_PID_Position_Y);  //λ��PID
//				PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
//	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
//			}
		//	else
		//	{
				PID_Control(yaw_angle,angle_offset-GimbalRef.yaw_angle_dynamic_ref,&PTZ_Motor_PID_Position_Y);
				PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
								+PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
		//	}
	
			PID_Control(temp_PTZ_Speed_Y, PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//�ٶȻ�
	    PTZ_Motor_PID_Speed_Y.pid_filter_out=PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1];
			 /******************************P�����*************************************************************************/
	
			PID_Control(temp_PTZ_Position_P,GimbalRef.pitch_angle_dynamic_ref,&PTZ_Motor_PID_Position_P);//λ�û�
      PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//�ٶȻ�
	    #elif  MPU_STATUS==MPU_NORMAL||MPU_STATUS==MPU_USART

	    /***************************Y�����***********************************************************************/
				PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset+GimbalRef.yaw_angle_dynamic_ref+cameraopen_yaw;
				
				PID_Control(yaw_usart_angle,angle_offset+GimbalRef.yaw_angle_dynamic_ref+cameraopen_yaw+changehead_y,&PTZ_Motor_PID_Position_Y);
				PTZ_Motor_PID_Position_Y.pid_filter_out=(PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0]);
				VAL_LIMIT(PTZ_Motor_PID_Position_Y.pid_filter_out,-300,300)

			PID_Control(temp_PTZ_Speed_Y,PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//�ٶȻ�
			PTZ_Motor_PID_Speed_Y.pid_filter_out=-(PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1]);//6020���������ʱ��ת������˳ʱ��ת
     
			 /******************************P�����*************************************************************************/	
			PTZ_Motor_PID_Position_P.Expect_Value=GimbalRef.pitch_angle_dynamic_ref+pitch_camera_filter;
			VAL_LIMIT(PTZ_Motor_PID_Position_P.Expect_Value,PITCH_DOWM_LIMIT,PITCH_UP_LIMIT);			 
			PID_Control(pitch_usart_angle,PTZ_Motor_PID_Position_P.Expect_Value,&PTZ_Motor_PID_Position_P);//λ�û�					
      PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//�ٶȻ�	
			
	    #endif
			PID_Control(Dial_the_motor.ecd_angle,position_hit,&Hit_position);
			PID_Control(Dial_motor_speed_fdb,Hit_position.pid_out,&Hit_speed);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_Data_Send(void)
*��������:	 ��̨�����������
���������   û��
���������   û��
*******************************************************************************/
void Gimbal_Data_Send(void)
{
	PID_Limit_Out(&Hit_speed,HIT_MAX_OUT);
	VAL_LIMIT(PTZ_Motor_PID_Speed_Y.pid_filter_out,-YAW_MAX_OUT,YAW_MAX_OUT);
	VAL_LIMIT(PTZ_Motor_PID_Speed_P.pid_out,-5000,5000);

	#if INFANTRY==3
				databuf_PTZ[0]=PTZ_Motor_PID_Speed_Y.pid_filter_out>>8;
				databuf_PTZ[1]=PTZ_Motor_PID_Speed_Y.pid_filter_out;
	#else
				databuf_PTZ[0]=-PTZ_Motor_PID_Speed_Y.pid_filter_out>>8;
				databuf_PTZ[1]=-PTZ_Motor_PID_Speed_Y.pid_filter_out;
  #endif

	databuf_PTZ[2]=PTZ_Motor_PID_Speed_P.pid_out>>8; 
	databuf_PTZ[3]=PTZ_Motor_PID_Speed_P.pid_out;
	
//	databuf_PTZ[0]='c';                         //����У׼�����뿪�ص�6623��� ÿ�θ����������ʱ�������У׼

#if   DEBUG_MODE==1
			
			databuf_PTZ[0]=0;
			databuf_PTZ[1]=0;
			databuf_PTZ[2]=0;
			databuf_PTZ[3]=0; 
#endif
	databuf_PTZ[4]=Hit_speed.pid_out>>8;
	databuf_PTZ[5]=Hit_speed.pid_out;
	
	taskENTER_CRITICAL();
  CAN1_Send_Msg_PTZ(databuf_PTZ); 
  taskEXIT_CRITICAL();
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Friction_Init(void)
*��������:	 2312Ħ���ֳ�ʼ��
���������   û��
���������   û��
*******************************************************************************/
void Friction_2312_Init(void)
{	
	TIM_SetTIM4Compare1(900*Slope(FRICTION_OFF_TIME,FRICTION_RAMP1));
	TIM_SetTIM4Compare2(900*Slope(FRICTION_OFF_TIME,FRICTION_RAMP2));
	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Friction_Control(int pwm)
*��������:	 Ħ����PWM������
���������   PWM��С
���������   û�� 1900 1400 
*******************************************************************************/
void Friction_ON(int pwm)
{
	TIM_SetTIM4Compare1(pwm);
	TIM_SetTIM4Compare2(pwm);
}


void Friction_OFF(void)   //Ħ���ֹر�
{
	TIM_SetTIM4Compare1(900);
	TIM_SetTIM4Compare2(900);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Friction_420S_Init()
*��������:	 420S���У׼��ʼ��
���������   û��
���������   û��
*******************************************************************************/
void Friction_420S_Init(void)
{
	if(soft_js_time<1600)
	{
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value);   //��¼��С�����г�
		TIM_SetTIM4Compare2(Friction_420S_MIN_Value);
	}
	else if(soft_js_time<3200)
	{
		TIM_SetTIM4Compare1(Friction_420S_MAX_Value);  //��¼��������г�
		TIM_SetTIM4Compare2(Friction_420S_MAX_Value);
	}
	else
	{
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value);   //�ָ���ֵ
		TIM_SetTIM4Compare2(Friction_420S_MIN_Value);
	}
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Friction_420S_Control(int pwm)
*��������:	 420S���+2312��� PWM����
���������   ��ʱ���� ��������(ms) ������(1) PWM
���������   û��
��ע��      ��ʱ����1ms
*******************************************************************************/
void Friction_420S_Control(u32 soft_js_time,int period,int pwm)
{
	static int16_t add_pwm=0;
	static u8 flag=0;
	if(pwm>Friction_420S_MIN_Value)                      //����Ħ����
	{
		if(flag==0)
		{
			add_pwm=0;
			flag=1;
		}
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value+add_pwm);
	  TIM_SetTIM4Compare2(Friction_420S_MIN_Value+add_pwm);
	  if(soft_js_time%period==0&&((Friction_420S_MIN_Value+add_pwm)<pwm))   //2s +100
	  	add_pwm+=1;
	  else if((Friction_420S_MIN_Value+add_pwm)>=pwm)
	  	LASER_ON();
	}
	else                                                 //�ر�Ħ����
	{
		if(flag==1)
		{
			add_pwm=0;
			flag=0;
		}
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value+add_pwm);
	  TIM_SetTIM4Compare2(Friction_420S_MIN_Value+add_pwm);
	  if(soft_js_time%period==0&&((Friction_420S_MIN_Value+add_pwm)!=pwm))   //2s +100
	  	add_pwm-=1;
	  else if((Friction_420S_MIN_Value+add_pwm)==pwm)
	  	LASER_OFF();
	}
	
}

void Friction_420S_ON(int pwm)
{
	Friction_420S_Control(soft_js_time,Friction_420S_Period,pwm);
}

void Friction_420S_OFF()
{
	Friction_420S_Control(soft_js_time,Friction_420S_Period,Friction_420S_MIN_Value);
}

void Friction_Init(void)
{
	#if FRICTION_420S_STATUS               
	Friction_420S_Init();
	#else
	Friction_2312_Init();
	#endif
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	 void Gimbal_Debug(void)
*��������:	 ��̨���Գ���
���������   û��
���������   û��
*******************************************************************************/
void Gimbal_Debug(void)
{
	Gimbal_ControlData_Get();
	
//   PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset-GimbalRef.yaw_angle_dynamic_ref;       //��Ծ
	sin_data=System_identification(Period,Omg,20,30);
	 PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset-sin_data;  //���� 1000HZ 1ms�ɼ�һ��
	 
	 PID_Control(yaw_angle,PTZ_Motor_PID_Position_Y.Expect_Value,&PTZ_Motor_PID_Position_Y);
	 PID_Control(temp_PTZ_Speed_Y, PTZ_Motor_PID_Position_Y.pid_out,&PTZ_Motor_PID_Speed_Y);//�ٶȻ�
	 
	 PID_Control(temp_PTZ_Position_P,GimbalRef.pitch_angle_dynamic_ref,&PTZ_Motor_PID_Position_P);
	 PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);
	 
	 PID_Limit_Out(&PTZ_Motor_PID_Speed_Y,YAW_MAX_OUT);
	 PID_Limit_Out(&PTZ_Motor_PID_Speed_P,PITCH_MAX_OUT);

	 databuf_PTZ[0]=0>>8;  //����λ�û�
	 databuf_PTZ[1]=0;

	 databuf_PTZ[2]=0>>8; 
	 databuf_PTZ[3]=0;
	
		
	 taskENTER_CRITICAL();
	   CAN1_Send_Msg_PTZ(databuf_PTZ); 
	 taskEXIT_CRITICAL();
}

