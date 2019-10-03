/***底盘电机的相关配置**/
#include "stm32f4xx_hal.h"
#include "include.h"
int Twist_delay = 0;                //扭腰延时
static fp32 twist_angle=0;          //扭腰角度
float power_offset=0;
int expect_sum=0;        //用于计算4个电机的电流之和       
float Current_Motor14;   //底盘两侧电机电流之和
float Current_Motor23;
float Current_Motor1234; //总电流
volatile float y_ecdangsav = 0;     //扭腰过程中Y轴角度缓存变量
extern float changehead_rorate;
float rotate_k=NORMAL_ROTATE_K;                   //转向系数
int rorate_slow=0;
int twist_slow=0;
POWER_CONTROL_STATUS power_control_status=POWER_NORMAL;
CHASSIS_WALK_SPEED Chassis_Walk_Speed=NORMAL_SPEED;
ChassisSpeed_Ref_t          Chassis_Speed_Ref;                    //遥控器参数
Chassis_Motor_PID_Expect    Chassis_Motor_PID_Ref;                //底盘电机期望值                                                                                                                                    
																				                               //  1\      2/  麦克纳姆轮摆放位置																
Chassis_Motor_Actual_Speed  Chassis_Motor_Actual_Speed_ref;    	       //  3/      4\

float chassis_wz_set_scale=0.0f;

void PID_Expect()                            //得到底盘电机所期望的速度    采用的是麦克纳姆轮 两个左旋 两个右旋
{
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_1 = Chassis_Speed_Ref.forward_back_ref +
																										 Chassis_Speed_Ref.left_right_ref + Chassis_Speed_Ref.rotate_ref*(1.0f+chassis_wz_set_scale);
	
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_2 =  -Chassis_Speed_Ref.forward_back_ref+
																										 Chassis_Speed_Ref.left_right_ref +Chassis_Speed_Ref.rotate_ref*(1.0f+chassis_wz_set_scale);
	
	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_3 = Chassis_Speed_Ref.forward_back_ref -
	                                                   Chassis_Speed_Ref.left_right_ref + Chassis_Speed_Ref.rotate_ref*(1.0f-chassis_wz_set_scale);

	Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_4 = -Chassis_Speed_Ref.forward_back_ref-
																										 Chassis_Speed_Ref.left_right_ref+ Chassis_Speed_Ref.rotate_ref*(1.0f-chassis_wz_set_scale);
	if(key_mouse_inf.ctrl_flag&&!key_mouse_inf.z_flag)   //开始静走 用于补弹调整
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

/**************************实现函数********************************************
*函数原型:	 void Chassis_Motor_Control(void)
*功　　能:	 底盘电机控制
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Chassis_Motor_Control(void)
{
	if(GetInputMode()==OFFLINE)  //遥控器离线
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
									Chassis_Motor_PID_Ref.Chassis_Motor_PID_Expect_4,&Chassis_Motor_PID_4);//底盘电机的PID

}

/**************************实现函数********************************************
*函数原型:	 void Chassis_Rotate_Control(void)
*功　　能:	 底盘扭腰 转向控制
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Chassis_Rotate_Control(void)
{
	if(rotate_revive_flag==1)  //死亡前自旋
	{
		//目的是更快归中
		if(mechanical_angle_PTZ_Y.raw_value<YAW_INITIAL_VALUE)
		{
			Chassis_Speed_Ref.rotate_ref=TO_MIDDLE_SPEED;   //设置归中速度
		}
		else if(mechanical_angle_PTZ_Y.raw_value>YAW_INITIAL_VALUE)
		{
			Chassis_Speed_Ref.rotate_ref=-TO_MIDDLE_SPEED;   //设置归中速度
		}
		
		if(__fabs(mechanical_angle_PTZ_Y.raw_value)>=(YAW_INITIAL_VALUE-10)&&mechanical_angle_PTZ_Y.raw_value<=(YAW_INITIAL_VALUE+10))
		{
			yaw_middle_value=mechanical_angle_PTZ_Y.raw_value;
			rotate_revive_flag=0;
			can_count_y=0;   //重新归中
			PTZ_Parameter_Init(&mechanical_angle_PTZ_Y);
		}
	}
	else{
	twist_angle=Set_twist_angle(TWIST_NO_MOVE_MAX_ANGLE,TWIST_MAX_T);   //生成扭腰角度  默认 55度 1500ms
	if(!key_mouse_inf.no_twisted&&key_mouse_inf.save_flag)  //记录扭腰前的Y轴电机角度
	{
		y_ecdangsav= - mechanical_angle_PTZ_Y.ecd_angle;
	}
		if(key_mouse_inf.rotate==1)  //自旋
		{
			Chassis_Speed_Ref.rotate_ref=NORMAL_SPEED;
			if(CAP_volt<RT_CAP_TAL_THRESHOLD)
				rorate_slow=1;
			else
				rorate_slow=0;
		}
		else if((key_mouse_inf.rotate==0&&key_mouse_inf.rotate_to_middle==0))  //不自旋 但归中未完成
		{
			if(mechanical_angle_PTZ_Y.raw_value<YAW_INITIAL_VALUE)
			{
				Chassis_Speed_Ref.rotate_ref=Chassis_Speed_Ref.rotate_ref;   //设置归中速度
			}
			else if(mechanical_angle_PTZ_Y.raw_value>YAW_INITIAL_VALUE)
			{
				Chassis_Speed_Ref.rotate_ref=--Chassis_Speed_Ref.rotate_ref;   //设置归中速度
			}
			if(__fabs(mechanical_angle_PTZ_Y.raw_value)>=(YAW_INITIAL_VALUE-10)&&mechanical_angle_PTZ_Y.raw_value<=(YAW_INITIAL_VALUE+10))
				{
					yaw_middle_value=mechanical_angle_PTZ_Y.raw_value;
					key_mouse_inf.rotate=0;
					key_mouse_inf.rotate_to_middle=1; //归中完成
					can_count_y=0;   //重新归中
					PTZ_Parameter_Init(&mechanical_angle_PTZ_Y);
				}
		}
		else if(key_mouse_inf.no_twisted==0)  //扭腰 不能自旋
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
		else if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1) //自旋结束 不扭腰
		{//左为正 右为负
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

/**************************实现函数********************************************
*函数原型:	 void Chassis_Power_Control(void)
*功　　能:	 底盘功率控制
输入参数：   没有
输出参数：   没有
*******************************************************************************/
void Chassis_Power_Control(void)
{
	static int power_limit_value=0;
	if(key_mouse_inf.shift_flag==0||(CAP_volt<CAP_TAL_THRESHOLD&&CAP_volt>0)||rorate_slow==1||twist_slow==1||CAP_volt==0) //超级电容未加入或者超级电容加入但shift未按下时开启功率环
	{
		power_limit_value=POWER_NORMAL_LIMIT_VALUE;
		Current_Motor14=__fabs(Chassis_Motor_PID_1.pid_out)+__fabs(Chassis_Motor_PID_4.pid_out);
		Current_Motor23=__fabs(Chassis_Motor_PID_2.pid_out)+__fabs(Chassis_Motor_PID_3.pid_out);

		Current_Motor1234=Current_Motor14+Current_Motor23;
		if(Current_Motor1234<=20)
			expect_sum=0;     //期望电流置零
		else 
			expect_sum=Current_Motor1234;   //四个电机的电流之和
		
	  #if CHASSIS_MOTOR_TYPE==M3510
		 power_actual=actual_current*ext_power_heat_data.chassis_volt;   //计算实际功率 采集电流值
	  #elif CHASSIS_MOTOR_TYPE==M3508
		 power_actual=Get_Power();
	  #endif
		if((power_actual>power_limit_value||Power_Inc>POWER_INCREASE)) 
		{
			power_expect=Last_expect+PID_Increment(power_actual,power_limit_value,&Power_limit); //增量式计算功率期望值
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
				bili=(current_expect*CURRENT_OFFSET/(expect_sum+50));    //计算每个轮的电流值   +50 防止分母为零
			}
			Chassis_Motor_PID_1.pid_out = (bili * Chassis_Motor_PID_1.pid_out); //按比例分配给底盘电机
			Chassis_Motor_PID_2.pid_out = (bili * Chassis_Motor_PID_2.pid_out);
			Chassis_Motor_PID_3.pid_out = (bili * Chassis_Motor_PID_3.pid_out);
			Chassis_Motor_PID_4.pid_out = (bili * Chassis_Motor_PID_4.pid_out);
			power_last=power_actual;
			Power_Inc=power_actual-power_last;
		}
	}
}
/**************************实现函数********************************************
*函数原型:	 void Chassis_Data_Send(void)
*功　　能:	 底盘电机电流发送
输入参数：   没有
输出参数：   没有
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
 *@brief 获取每个电机的功率
 *@param 电机转速 转矩电流
 */
float Get_chassis_motor_power(int speed,int current )   //用转矩电流得到功率值
{  
	float power;
	power=power_offset/4.0f+            //底盘静止时 裁判系统功率/4
	      1.571e-7*speed*speed+
	      2.248e-6*speed*current+
	      2.022e-8*current*current;
	kalman_filter_calc(&power_filter,power,power);
	return power_filter.filtered_value[0];  //返回功率值		
}

/**
 *@brief 获取底盘总功率，封装函数
 *@return 底盘总功率
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
 * @param 扭腰幅值(PI)>=PI/10  周期（ms）扭腰模式 随机扭腰 常规扭腰 随机扭腰加暂停
 * @return 扭腰角度
 * A=TWIST_NO_MOVE_ANGLE  W=250.0f
 * Set_twist_angle(TWIST_NO_MOVE_ANGLE,250.0f)
 */
float Set_twist_angle(float A,float T)
{
	//摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
	static fp32 swing_time = 0.0f;
	//swing_angle 是计算出来的角度
	static fp32 swing_angle = 0.0f;
	//max_angle 是sin函数的幅值
	static fp32 max_angle = TWIST_NO_MOVE_MAX_ANGLE;
	//add_time 是摇摆角度改变的快慢，越大越快
	volatile static fp32 add_time = PI/(TWIST_MIN_T/2.0f);
  
	if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate==0)
	{
		swing_time=0;
		swing_angle=0;
	}
	else	
	{
		//sin函数不超过2pi
		//底盘行走过程中 或者需要大转弯的时候 关闭随机扭腰 减小扭腰幅度
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
						//Set_random_twist(&max_angle,&add_time,A,T);        //每隔一个周期变换一下扭腰幅度和频率
				}
		}
		
		swing_angle = max_angle * arm_sin_f32(swing_time);  //弧度转换成角度
		swing_time += PI/(T/2.0f);;  //恒定频率  800ms
	}
	return swing_angle;
}

/**
 *@brief 产生扭腰随机周期和赋值
 *@param 扭腰幅值 扭腰周期 最大值 最小值内置
 */
void Set_random_twist(float *A,float *T,float A_max,float T_max)
{
	*A=set_random_float(TWIST_NO_MOVE_MIN_ANGLE,A_max);
	*T=PI/(set_random_float(TWIST_MIN_T,T_max)/2.0f);
}
