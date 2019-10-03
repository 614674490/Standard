/**
 * 更改：Z轴重力加速度 取反  temp_PTZ_Position_Y temp_PTZ_Speed_P  GimbalRef.yaw_angle_dynamic_ref取反
 */
#include "FreeRTOS.h"
#include "task.h"
#include "include.h"
u8 friction_init_flag=0;  //摩擦轮初始化标志位 初始化时置一 不可对摩擦轮进行其他操作
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
	volt=9*(16*err_p+1*(-z2));//16增加响应，
	volt=(volt-z3)/50;	//110
	volt_old=volt;
}
/**************************实现函数********************************************
*函数原型:	 void Gimbal_ControlData_Get(void)
*功　　能:	 系统辨识->产生正选激励信号
输入参数：  周期 频率 幅值 采样次数
输出参数：  正弦激励信号
	备注     每个周期都对应不同的频率
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
		if(P_count>=sampletimes)  //每组采集30次 共60组  调节该参数 可增加测试数据
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


/**************************实现函数********************************************
*函数原型:	 void Gimbal_ControlData_Get(void)
*功　　能:	 云台控制数据获取
输入参数：   没有
输出参数：   没有
备注：      GM6020和6623的反馈值相反 注意正负号的选取 选取不当会导致云台疯转
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
/**************************实现函数********************************************
*函数原型:	 void Gimbal_Init(void)
*功　　能:	 云台归中
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Gimbal_Init(void)
{
		Gimbal_ControlData_Get();
		kalman_filter_calc(&yaw_pid_kalman_filter,PTZ_Motor_PID_Position_Y.ec,PTZ_Motor_PID_Speed_Y.ec);//两环微分滤波
	
		/******************************************Y轴控制**************************************************************/
		ramp=YAW_RAMP;                               //斜坡函数中的枚举
		PID_Control((temp_PTZ_Position_Y*Slope(65000,ramp)),GimbalRef.yaw_angle_dynamic_ref,&PTZ_Motor_PID_Position_Y);
		
		PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
				
		PID_Control(temp_PTZ_Speed_Y,PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//速度环
		#if MPU_STATUS==MPU_USART
			PTZ_Motor_PID_Speed_Y.pid_filter_out=-(PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1]);
		#else
		PTZ_Motor_PID_Speed_Y.pid_filter_out=PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1];
		#endif
	
    /******************************************P轴控制*******************************************************************/
		ramp=PITCH_RAMP;   //斜坡
		PID_Control(pitch_usart_angle*Slope(65000,ramp),PTZ_Motor_PID_Position_P.Expect_Value,&PTZ_Motor_PID_Position_P);//位置环					
    PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//速度环	

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

/**************************实现函数********************************************
*函数原型:	 void Gimbal_Control(void)
*功　　能:	 云台控制
输入参数：   没有
输出参数：   没有	
备注：PID控制参数中的实际值和期望值要直接赋值 简介赋值会造成些许延迟 导致云台静止时左右微动
     实际值的方向要和初始化时的实际值方向保持一致 否则云台会产生一定角度的旋转
     P轴加入卡尔曼滤波后，当给阶跃信号时会影响Y轴的卡尔曼滤波输出
     
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
	kalman_filter_calc(&yaw_pid_kalman_filter,PTZ_Motor_PID_Position_Y.ec,PTZ_Motor_PID_Speed_Y.ec);//两环微分滤波
	    #if MPU_STATUS==MPU_UPSIDE_DOWN
	    /***************************Y轴控制***********************************************************************/
//	    if(((RC_CtrlData.mouse.x==0&&GetInputMode()==KEY_MOUSE_INPUT)||(RC_CtrlData.rc.ch2==1024&&GetInputMode()==REMOTE_INPUT))&&key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate==0)  //不进行控制时 锁死云台
//			{
//				GimbalRef.yaw_angle_dynamic_ref=yaw_angle-angle_offset;
//				PID_Control(temp_PTZ_Position_Y,temp_PTZ_Position_Y,&PTZ_Motor_PID_Position_Y);  //位置PID
//				PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
//	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
//			}
		//	else
		//	{
				PID_Control(yaw_angle,angle_offset-GimbalRef.yaw_angle_dynamic_ref,&PTZ_Motor_PID_Position_Y);
				PTZ_Motor_PID_Position_Y.pid_filter_out=PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
								+PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0];
		//	}
	
			PID_Control(temp_PTZ_Speed_Y, PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//速度环
	    PTZ_Motor_PID_Speed_Y.pid_filter_out=PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1];
			 /******************************P轴控制*************************************************************************/
	
			PID_Control(temp_PTZ_Position_P,GimbalRef.pitch_angle_dynamic_ref,&PTZ_Motor_PID_Position_P);//位置环
      PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//速度环
	    #elif  MPU_STATUS==MPU_NORMAL||MPU_STATUS==MPU_USART

	    /***************************Y轴控制***********************************************************************/
				PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset+GimbalRef.yaw_angle_dynamic_ref+cameraopen_yaw;
				
				PID_Control(yaw_usart_angle,angle_offset+GimbalRef.yaw_angle_dynamic_ref+cameraopen_yaw+changehead_y,&PTZ_Motor_PID_Position_Y);
				PTZ_Motor_PID_Position_Y.pid_filter_out=(PTZ_Motor_PID_Position_Y.Kpout+PTZ_Motor_PID_Position_Y.Kiout
	            +PTZ_Motor_PID_Position_Y.Kd*yaw_pid_kalman_filter.filtered_value[0]);
				VAL_LIMIT(PTZ_Motor_PID_Position_Y.pid_filter_out,-300,300)

			PID_Control(temp_PTZ_Speed_Y,PTZ_Motor_PID_Position_Y.pid_filter_out,&PTZ_Motor_PID_Speed_Y);//速度环
			PTZ_Motor_PID_Speed_Y.pid_filter_out=-(PTZ_Motor_PID_Speed_Y.Kpout+PTZ_Motor_PID_Speed_Y.Kiout
	    +PTZ_Motor_PID_Speed_Y.Kd*yaw_pid_kalman_filter.filtered_value[1]);//6020输出正是逆时针转，负是顺时针转
     
			 /******************************P轴控制*************************************************************************/	
			PTZ_Motor_PID_Position_P.Expect_Value=GimbalRef.pitch_angle_dynamic_ref+pitch_camera_filter;
			VAL_LIMIT(PTZ_Motor_PID_Position_P.Expect_Value,PITCH_DOWM_LIMIT,PITCH_UP_LIMIT);			 
			PID_Control(pitch_usart_angle,PTZ_Motor_PID_Position_P.Expect_Value,&PTZ_Motor_PID_Position_P);//位置环					
      PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);//速度环	
			
	    #endif
			PID_Control(Dial_the_motor.ecd_angle,position_hit,&Hit_position);
			PID_Control(Dial_motor_speed_fdb,Hit_position.pid_out,&Hit_speed);
}

/**************************实现函数********************************************
*函数原型:	 void Gimbal_Data_Send(void)
*功　　能:	 云台电机电流发送
输入参数：   没有
输出参数：   没有
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
	
//	databuf_PTZ[0]='c';                         //用于校准带拨码开关的6623电调 每次更换电机或电调时必须进行校准

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

/**************************实现函数********************************************
*函数原型:	 void Friction_Init(void)
*功　　能:	 2312摩擦轮初始化
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Friction_2312_Init(void)
{	
	TIM_SetTIM4Compare1(900*Slope(FRICTION_OFF_TIME,FRICTION_RAMP1));
	TIM_SetTIM4Compare2(900*Slope(FRICTION_OFF_TIME,FRICTION_RAMP2));
	
}

/**************************实现函数********************************************
*函数原型:	 void Friction_Control(int pwm)
*功　　能:	 摩擦轮PWM波控制
输入参数：   PWM大小
输出参数：   没有 1900 1400 
*******************************************************************************/
void Friction_ON(int pwm)
{
	TIM_SetTIM4Compare1(pwm);
	TIM_SetTIM4Compare2(pwm);
}


void Friction_OFF(void)   //摩擦轮关闭
{
	TIM_SetTIM4Compare1(900);
	TIM_SetTIM4Compare2(900);
}

/**************************实现函数********************************************
*函数原型:	 void Friction_420S_Init()
*功　　能:	 420S电调校准初始化
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Friction_420S_Init(void)
{
	if(soft_js_time<1600)
	{
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value);   //记录最小油门行程
		TIM_SetTIM4Compare2(Friction_420S_MIN_Value);
	}
	else if(soft_js_time<3200)
	{
		TIM_SetTIM4Compare1(Friction_420S_MAX_Value);  //记录最大油门行程
		TIM_SetTIM4Compare2(Friction_420S_MAX_Value);
	}
	else
	{
		TIM_SetTIM4Compare1(Friction_420S_MIN_Value);   //恢复初值
		TIM_SetTIM4Compare2(Friction_420S_MIN_Value);
	}
}
/**************************实现函数********************************************
*函数原型:	 void Friction_420S_Control(int pwm)
*功　　能:	 420S电调+2312电机 PWM驱动
输入参数：   计时变量 递增周期(ms) 递增量(1) PWM
输出参数：   没有
备注：      定时周期1ms
*******************************************************************************/
void Friction_420S_Control(u32 soft_js_time,int period,int pwm)
{
	static int16_t add_pwm=0;
	static u8 flag=0;
	if(pwm>Friction_420S_MIN_Value)                      //开启摩擦轮
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
	else                                                 //关闭摩擦轮
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
/**************************实现函数********************************************
*函数原型:	 void Gimbal_Debug(void)
*功　　能:	 云台调试程序
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Gimbal_Debug(void)
{
	Gimbal_ControlData_Get();
	
//   PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset-GimbalRef.yaw_angle_dynamic_ref;       //阶跃
	sin_data=System_identification(Period,Omg,20,30);
	 PTZ_Motor_PID_Position_Y.Expect_Value=angle_offset-sin_data;  //正弦 1000HZ 1ms采集一次
	 
	 PID_Control(yaw_angle,PTZ_Motor_PID_Position_Y.Expect_Value,&PTZ_Motor_PID_Position_Y);
	 PID_Control(temp_PTZ_Speed_Y, PTZ_Motor_PID_Position_Y.pid_out,&PTZ_Motor_PID_Speed_Y);//速度环
	 
	 PID_Control(temp_PTZ_Position_P,GimbalRef.pitch_angle_dynamic_ref,&PTZ_Motor_PID_Position_P);
	 PID_Control(temp_PTZ_Speed_P,PTZ_Motor_PID_Position_P.pid_out,&PTZ_Motor_PID_Speed_P);
	 
	 PID_Limit_Out(&PTZ_Motor_PID_Speed_Y,YAW_MAX_OUT);
	 PID_Limit_Out(&PTZ_Motor_PID_Speed_P,PITCH_MAX_OUT);

	 databuf_PTZ[0]=0>>8;  //调节位置环
	 databuf_PTZ[1]=0;

	 databuf_PTZ[2]=0>>8; 
	 databuf_PTZ[3]=0;
	
		
	 taskENTER_CRITICAL();
	   CAN1_Send_Msg_PTZ(databuf_PTZ); 
	 taskEXIT_CRITICAL();
}

