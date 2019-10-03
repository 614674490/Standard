#include "include.h"
/*  遥控器 鼠标 键盘 控制函数 */
float  mouse_yaw;
int Cartridge_Flag=0;                                 //卡弹标志位
RC_Ctl_t RC_CtrlData={0};                             //遥控器发送数据结构体
Gimbal_Ref_t GimbalRef={0};                           //云台运动参数结构体                        
ShootingFreCon_t ShootingFreCon={1,0,0,240,240,4.0};  //实际热量 上限热量 每秒冷却值
OPERATE_MESSAGE  key_mouse_inf={1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1};  //鼠标 键盘 相关键值标志位
u8 friction_flag=0;                                   //摩擦轮开启标志位
REFEREE_LIMIT_MESSAGE refree_change_inf={0};          //摩擦轮速度 弹丸射频距离敏感度 热量结构体
FrictionWheelState_e friction_wheel_state=FRICTION_WHEEL_OFF; //枚举 摩擦轮状态
InputMode_e inputmode=OFFLINE;                        //遥控器当前模式 默认为离线
u8 ws_flag=0,ad_flag=0;      //w/s按下标志位 a/d按下标志位
u8 i = 0;                    //后面STOP停止发送数据和清空数据使用
int q_count = 0, e_count = 0, z_count=0,g_count=0,v_count=0;   //连发 扭腰（底盘动 云台不动） 的计数变量
u8 level=1;      //等级
int up_count;   //升级延时计数
int down_count;   ///降级延时计数
volatile float angle_offset=0;      //记录上电初始化时Y轴电机的初始编码值 便于云台的归中和锁死
volatile float pitch_angle_offset=0;      //记录上电初始化时Y轴电机的初始编码值 便于云台的归中和锁死
float distance;        //用于计算拨弹电机当前位置和目标位置的距离
int Dial_angle_offset=0;  //卡单角度补偿
float changehead_rorate,changehead_y;  //用于云台回马枪

/**************************实现函数********************************************
*函数原型:	   MouseShootControl(Mouse *mouse)
*功　　能:	   弹丸发射控制  
输入参数：   鼠标当前状态
输出参数：   没有
*******************************************************************************/
void MouseShootControl(Mouse *mouse)
{
	static int16_t closeDelayCount=0;   //右键关闭摩擦轮延时计数   从关闭当开启 长按右键 时间过长的话 摩擦轮又会关闭; 从开启到关闭 右键长按便会关闭 时间过长也会关闭

		switch (friction_wheel_state)  //根据摩擦轮当前状态进行切换
		{
			case FRICTION_WHEEL_OFF:
			{
				friction_flag=0;
				if(mouse->last_press_r==0&&mouse->press_r==1)  //从关闭到开始转动
				{
					Friction_OFF();
					friction_flag=0;
					friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
					closeDelayCount = 0;
				}
				LASER_OFF();
			}break;
			case FRICTION_WHEEL_START_TURNNING: 
			{
				friction_flag=0;
				if(mouse->press_r==1)                    //右键持续按下 关闭摩擦轮
				{
					closeDelayCount++;
				}
				else
				{
					closeDelayCount=0;
				}
				if(closeDelayCount>FRICITION_CLOSE_DELAY)   //按下时间达到关闭时长
				{
					friction_wheel_state=FRICTION_WHEEL_OFF;
					Friction_OFF();
				}
				else    //未关闭 则将状态切换到开启
				{
					//摩擦轮加速
					Friction_ON(refree_change_inf.fricition_speed);
					friction_wheel_state=FRICTION_WHEEL_ON;
					LASER_ON();
				}
			}break;
			case FRICTION_WHEEL_ON:
			{
				Friction_ON(refree_change_inf.fricition_speed);
			  friction_flag=1;
				if(mouse->press_r==1)
				{
					closeDelayCount++;
				}
				else
				{
					closeDelayCount=0;
				}
				if(closeDelayCount>FRICITION_CLOSE_DELAY) 
				{	
					friction_wheel_state=FRICTION_WHEEL_OFF;
					Friction_OFF();
				}
		 }break;
	 }
	//转动拨弹电机的前提是摩擦轮开启  不然弹丸
	if(mouse->last_press_l==0&&(mouse->press_l==1)&&friction_wheel_state==FRICTION_WHEEL_ON) 
	{
		Shoot_Control(key_mouse_inf.shoot_mode);
	}	
	cartridge();	
	mouse->last_press_r=mouse->press_r;  //记录上一次鼠标右键状态
	if(key_mouse_inf.con_shoot_flag==1)  //Q按下
		mouse->last_press_l=0;
	else
		mouse->last_press_l=mouse->press_l;  //记录上一次鼠标左键状态
}
/**************************实现函数********************************************
*函数原型:	   void MouseKeyControlProcess(Mouse *mouse, Key *key)
*功　　能:	   键鼠模式控制
输入参数：   鼠标与键盘的状态
输出参数：   没有
*******************************************************************************/
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	if(key->v &0x10)  //shift
	{
		key_mouse_inf.shift_flag=1;
		Chassis_Walk_Speed=HIGH_SPEED;
		rotate_k=FAST_ROTATE_K;
		if(CAP_volt==0)  //通信失败时 以中速行驶
			Chassis_Walk_Speed=MIDDLE_SPEED;
		else
			Chassis_Walk_Speed=HIGH_SPEED;
	}
	else
	{		
		key_mouse_inf.shift_flag=0;
		Chassis_Walk_Speed=NORMAL_SPEED;
		rotate_k=NORMAL_ROTATE_K;
	}
	//movement process
	if(key->v & 0x01)  //key: w
	{
		ramp=CHASSIS_RAMP_FB;
		key_mouse_inf.w_flag=1;
		if(key_mouse_inf.FB_flag==0)  //是不是重新按下
		{
			ResetSlope(ramp);	          //释放斜坡
			ResetSlope(CHASSIS_RAMP_RL);
			key_mouse_inf.FB_flag=1;
		}
    WalkControl(ramp);
		key_mouse_inf.play_flag=0;
		key_mouse_inf.w_flag=0;
		ws_flag=1;
		key_mouse_inf.FB_Press_flag=1;
	}
	else if(key->v & 0x02)  //key: s
	{
		ramp=CHASSIS_RAMP_FB;
		key_mouse_inf.s_flag=1;
		if(key_mouse_inf.play_flag==0)
		{
			ResetSlope(ramp);	          //释放斜坡
			ResetSlope(CHASSIS_RAMP_RL);   
			key_mouse_inf.play_flag=1;
		}
		WalkControl(ramp);
		key_mouse_inf.FB_flag=0;
		key_mouse_inf.s_flag=0;
		ws_flag=1;
		key_mouse_inf.FB_Press_flag=1;
	}
	else  //未按下
	{
		key_mouse_inf.play_flag=0;
    key_mouse_inf.FB_flag=0;
		ws_flag=0;
		if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1)  //不扭腰 不自旋
		{
			ramp = CHASSIS_RAMP_FB;
			if(key_mouse_inf.FB_Press_flag==1)  //判断上次是否按下 只释放一次斜坡
			{
				ResetSlope(ramp);               //如果没有按，释放斜坡
				key_mouse_inf.FB_Press_flag=0;
			}
			Chassis_Speed_Ref.forward_back_ref *= (1.0f-Slope(DECELERATE_TIME,ramp));
		}
	  else	if(ad_flag==0)   
		{
			if(key_mouse_inf.FB_Press_flag==1)
			{
				ResetSlope(CHASSIS_RAMP_FB);            
				key_mouse_inf.FB_Press_flag=0;
			}
			if(key_mouse_inf.LR_Press_flag==1)
			{
				ResetSlope(CHASSIS_RAMP_RL);            
				key_mouse_inf.LR_Press_flag=0;
			}
			Chassis_Speed_Ref.forward_back_ref *= (1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_FB));
			Chassis_Speed_Ref.left_right_ref *=(1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_RL));
	  }
	}
	if(key->v & 0x04)  //key: d
	{
		ramp=CHASSIS_RAMP_RL;
		key_mouse_inf.d_flag=1;
		if(key_mouse_inf.rl_flag==0)
		{
			ResetSlope(ramp);	
			ResetSlope(CHASSIS_RAMP_FB);
			key_mouse_inf.rl_flag=1;
		}
		WalkControl(ramp);
		key_mouse_inf.rr_flag = 0;
		key_mouse_inf.d_flag=0;
		ad_flag=1;
		key_mouse_inf.LR_Press_flag=1;
	}
	else if(key->v & 0x08)  //key:a
	{
		ramp = CHASSIS_RAMP_RL;
		key_mouse_inf.a_flag=1;
		if(key_mouse_inf.rr_flag==0)
		{
			ResetSlope(ramp);
			ResetSlope(CHASSIS_RAMP_FB);
			key_mouse_inf.rr_flag=1;
		}
		WalkControl(ramp);
		key_mouse_inf.rl_flag=0;
		key_mouse_inf.a_flag=0;
		ad_flag=1;
		key_mouse_inf.LR_Press_flag=1;
	}
  else  //未按下
	{
		key_mouse_inf.rl_flag=0;
		key_mouse_inf.rr_flag=0;
		ad_flag=0;
		if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1)   //非自旋状态
		{
		  ramp = CHASSIS_RAMP_RL;
			if(key_mouse_inf.LR_Press_flag==1)
			{
			  ResetSlope(ramp);
				key_mouse_inf.LR_Press_flag=0;
			}
			Chassis_Speed_Ref.left_right_ref *=(1.0f-Slope(DECELERATE_TIME,ramp));  //按键松下时也要使用斜坡函数
		}
		else if(ws_flag==0)                         //自旋且另两个按键未按下 防止出现按键失灵的情况以及画弧走现象
		{
			if(key_mouse_inf.FB_Press_flag==1)
			{
				ResetSlope(CHASSIS_RAMP_FB);            
				key_mouse_inf.FB_Press_flag=0;
			}
			if(key_mouse_inf.LR_Press_flag==1)
			{
				ResetSlope(CHASSIS_RAMP_RL);            
				key_mouse_inf.LR_Press_flag=0;
			}
			Chassis_Speed_Ref.forward_back_ref *= (1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_FB));
			Chassis_Speed_Ref.left_right_ref *=(1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_RL));
		}
	
	}
	if(ad_flag==0&&ws_flag==0)
	{
		if(key_mouse_inf.FB_Press_flag==1)
		{
			ResetSlope(CHASSIS_RAMP_FB);            
			key_mouse_inf.FB_Press_flag=0;
		}
		if(key_mouse_inf.LR_Press_flag==1)
		{
			ResetSlope(CHASSIS_RAMP_RL);            
			key_mouse_inf.LR_Press_flag=0;
		}
		Chassis_Speed_Ref.forward_back_ref *= (1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_FB));
		Chassis_Speed_Ref.left_right_ref *=(1.0f-Slope(ACCELERA_TIME,CHASSIS_RAMP_RL));

	}
	
	if(key->v & KEY_F)      //F 开舱门
	{
		  Magaine_ON();   //F按下 开舵机
			key_mouse_inf.f_flag=1;
	}
	else
	{
		
		Magaine_OFF();   //F松开 关闭弹仓
		key_mouse_inf.f_flag=0;
		ext_shoot_data.bullet_count=0;  //补弹时 之前的弹丸计数清零
	}
	
	if(key->v & KEY_C)     
	{
		if(key_mouse_inf.c_flag==0)
		{
			key_mouse_inf.shoot_mode=!key_mouse_inf.shoot_mode;
			key_mouse_inf.c_flag=1;
		}
	}
	else
	{
		key_mouse_inf.c_flag=0;
	}
	
	if(key->v & 0x20)  //ctrl
	{
		key_mouse_inf.ctrl_flag=1;
	}
	else
	{
		key_mouse_inf.ctrl_flag=0;
	}
	if(key->v & 0x40)  //q  暴击按键
	{
			refree_change_inf.fricition_speed=LOW_FRICITION_ON_SPEED;  //降低摩擦轮的转动速度
			refree_change_inf.one_bullet_heat=SLOW_ONEBULLET_HEAT;  //发射子弹的热量降低
			key_mouse_inf.con_shoot_flag=1;                   //射击标志位
	}
	else
	{//NORMAL_FRICITION_ON_SPEED
		if(key_mouse_inf.shoot_mode==1)  //单发模式
		{
			refree_change_inf.fricition_speed=NORMAL_FRICITION_ON_SPEED;  //摩擦轮的速度为高速
			refree_change_inf.one_bullet_heat=FAST_ONEBULLET_HEAT;  //发射子弹的热量较高
		}			
		else  //三连发
		{
			refree_change_inf.fricition_speed=MIDDLE_FRICITION_ON_SPEED;  //摩擦轮的速度为中速
			refree_change_inf.three_bullet_heat=FAST_THREEBULLET_HEAT;
		}
		key_mouse_inf.con_shoot_flag=0;
	}
	
	if(key->v & 0x80)  //e
	{
		//扭腰按键
		e_count++;
		if(e_count>KEY_PRESS_DELAY)      //按下一段时间后
		{
				key_mouse_inf.save_flag = !key_mouse_inf.save_flag;
		    key_mouse_inf.no_twisted = !key_mouse_inf.no_twisted;
			  if(key_mouse_inf.rotate==1&&key_mouse_inf.no_twisted==0) //自旋切换为扭腰
				{
					key_mouse_inf.last_rotate=0;
					key_mouse_inf.rotate=0;
					key_mouse_inf.rotate_to_middle=0;                      //发出结束自旋请求
				}
			  key_mouse_inf.last_no_twisted=key_mouse_inf.no_twisted;
			  e_count = -200;                 //防止重复按下
		}
  }
	else
	{
		e_count=0;
	}
	#if INFANTRY!=3
	if(key->v & KEY_V)  //V 小陀螺按键
	{
		v_count++;
		if(v_count>KEY_PRESS_DELAY)      //按下一段时间后
		{
			key_mouse_inf.rotate=!key_mouse_inf.rotate;  //开启/关闭自旋
			if(key_mouse_inf.rotate==1&&key_mouse_inf.last_rotate==0)   //由不自旋变为自旋
			{
				key_mouse_inf.rotate_to_middle=0;  //始终是归中未完成 在chassis.c中改变其值
				if(key_mouse_inf.no_twisted==0)  //扭腰切换为自旋
				{
					key_mouse_inf.no_twisted=1;
					key_mouse_inf.save_flag=0;
				}
			}
			key_mouse_inf.last_rotate=key_mouse_inf.rotate;
			v_count = -200;                 //防止重复按下
		}
  }
	else
	{
		v_count=0;
	}
  if(key->v & KEY_Z)  //z 切换头的方向
	{		
		changehead_y=180;
		if(mechanical_angle_PTZ_Y.ecd_angle>-177)
			changehead_rorate=mechanical_angle_PTZ_Y.ecd_angle;
		else
			changehead_rorate=-180;
  }
	else
	{
		changehead_y=0;
		if(mechanical_angle_PTZ_Y.ecd_angle<-70)
			changehead_rorate=mechanical_angle_PTZ_Y.ecd_angle;
		else
			changehead_rorate=0;
	}
	#endif
	if(key->v & KEY_G)                       //G键按下 程序复位 只有在车疯的情况下才能使用
	{
		g_count++;
		if(g_count>KEY_PRESS_DELAY)      //按下一段时间后
		{
			for(i = 0; i<8; i++)
			{
				databuf[i] = 0;
				databuf_PTZ[i] = 0;
			}
			(*Chaais_Send)(databuf);
			CAN1_Send_Msg_PTZ(databuf_PTZ);
			__set_FAULTMASK(1);//关闭所有中断
			NVIC_SystemReset();//复位函数
		}
	}
	else
		g_count=0;
	#if INFANTRY!=5
	if(key->v & KEY_R)//自瞄    每按下一次切换一次
	{
		if(key_mouse_inf.r_flag==0)
		{
			key_mouse_inf.r_flag=1;
			key_mouse_inf.autoaim_mode=!key_mouse_inf.autoaim_mode;
		}
	}
	else
	{
		key_mouse_inf.r_flag=0;
		pitch_camera_filter=0;
	}
	
		if(key->v & KEY_X)//巡逻
	{
		key_mouse_inf.x_flag=1;
	}
	else
	{
		key_mouse_inf.x_flag=0;
	}
	#endif
		//鼠标限幅
		mouse_yaw=mouse->x;
		VAL_LIMIT(mouse->x,-43,43);
		VAL_LIMIT(mouse->y,-120,120);
	if(key_mouse_inf.autoaim_mode==0||(y_angle==0&&p_angle==0))//未开启自瞄或者自瞄状态下未检测到目标 则改为手动控制云台
	{
		GimbalRef.pitch_angle_dynamic_ref-=mouse->y*MOUSE_TO_PITCH_ANGLE_INC_FACT;
		GimbalRef.yaw_angle_dynamic_ref+=mouse->x*MOUSE_TO_YAW_ANGLE_INC_FACT;
	}
	VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,PITCH_DOWM_LIMIT,PITCH_UP_LIMIT);
		MouseShootControl(mouse);
}


//输入模式设置 
void SetInputMode(Remote *rc)
{
	if(rc->s2==1)
		inputmode=REMOTE_INPUT;
	else if(rc->s2==3)
		inputmode=KEY_MOUSE_INPUT;
	else if(rc->s2==2)
		inputmode=STOP;
	else
		inputmode=OFFLINE;
}

InputMode_e GetInputMode()   //函数封装
{
	return inputmode;
}

/**
 *@brief 遥控器数据处理
 * const u8 *Msg(常量指针) : 指针指向的变量的值不可通过该指针修改，但是指针指向的值可以改变。 防止数据误操作
 * 而指针常量只能指向一个指针，不可指向其他指针 u8 *const Msg
 */
void Remote_Data_Process(const u8  *Msg)
{
			static u8 switch_flag=1;
			RC_CtrlData.rc.ch0 = ((int16_t)Msg[0] | ((int16_t)Msg[1] << 8)) & 0x07FF; 
			RC_CtrlData.rc.ch1 = (((int16_t)Msg[1] >> 3) | ((int16_t)Msg[2] << 5)) & 0x07FF;
			RC_CtrlData.rc.ch2 = (((int16_t)Msg[2] >> 6) | ((int16_t)Msg[3] << 2) |
													 ((int16_t)Msg[4] << 10)) & 0x07FF;
			RC_CtrlData.rc.ch3 = (((int16_t)Msg[4] >> 1) | ((int16_t)Msg[5]<<7)) & 0x07FF;

			RC_CtrlData.rc.s1 = ((Msg[5] >> 4) & 0x000C) >> 2;
			RC_CtrlData.rc.s2 = ((Msg[5] >> 4) & 0x0003);

			RC_CtrlData.mouse.x = ((int16_t)Msg[6]) | ((int16_t)Msg[7] << 8);
			RC_CtrlData.mouse.y = ((int16_t)Msg[8]) | ((int16_t)Msg[9] << 8);
			RC_CtrlData.mouse.z = ((int16_t)Msg[10]) | ((int16_t)Msg[11] << 8);      		

			RC_CtrlData.mouse.press_l = Msg[12];
			RC_CtrlData.mouse.press_r = Msg[13];
		
			RC_CtrlData.key.v = ((int16_t)Msg[14]|(int16_t)Msg[15]<<8);
			
			SetInputMode(&RC_CtrlData.rc);
			
			switch (GetInputMode())
			{
				case REMOTE_INPUT:
				{
					//遥控器控制模式
					if(switch_flag==0)
					{
						Friction_OFF();
						switch_flag=1;
					}
					RemoteControlProcess(&(RC_CtrlData.rc));
					expect_speed=0;
				}break;
				
				case KEY_MOUSE_INPUT:
				{
					//键鼠 控制模式
					//expect_speed=20;
					switch_flag=0;
					MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
				}break;
				case STOP:
				{
					//急停模式
					Friction_OFF();
					RemoteControlProcess(&(RC_CtrlData.rc));
					PID_Clear();
				}break;
				default:
					break;
		 }
}

//遥控器控制模式处理
void RemoteControlProcess(Remote *rc)
{
	Chassis_Speed_Ref.forward_back_ref=(rc->ch1-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*STICK_TO_CHASSIS_SPEED_REF_FACT;
  Chassis_Speed_Ref.left_right_ref  =(rc->ch0-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*STICK_TO_CHASSIS_SPEED_REF_FACT;
	rc_deadline_limit(Chassis_Speed_Ref.forward_back_ref, Chassis_Speed_Ref.forward_back_ref, RC_deadband);
  rc_deadline_limit(Chassis_Speed_Ref.forward_back_ref, Chassis_Speed_Ref.forward_back_ref, RC_deadband);
	

	GimbalRef.pitch_angle_dynamic_ref+=(rc->ch3-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*STICK_TO_PITCH_ANGLE_INC_FACT;
	GimbalRef.yaw_angle_dynamic_ref+=(rc->ch2-(int16_t)REMOTE_CONTROLLER_STICK_OFFSET)*STICK_TO_YAW_ANGLE_INC_FACT;
	VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,PITCH_DOWM_LIMIT,PITCH_UP_LIMIT);			 
	RemoteShootControl(rc->s1);
}

//遥控器摩擦轮
void RemoteShootControl(uint8_t val) 
{
	static u8 key_flag=1;
	static u8 hit_flag=0;
	static u16 Friction_wheel=0;
	static u16 shoot_count=0;
	if(key_flag&&val==1)
	{
		shoot_count=0;
		Hit_speed.pid_out=0;                             //拨弹电机停止转动
		key_flag=0;
		if(Friction_wheel%2==0)
		{
			friction_wheel_state=FRICTION_WHEEL_ON;
			#if FRICTION_420S_STATUS==0
			Friction_ON(NORMAL_FRICITION_ON_SPEED);
			LASER_ON();                                     //开启激光
			#endif
		}
		else 
		{
			friction_wheel_state=FRICTION_WHEEL_OFF;
			#if FRICTION_420S_STATUS==0
			Friction_OFF();
			LASER_OFF();
			#endif
		}
		hit_flag = !hit_flag;
    Friction_wheel++;   
	}
	if(val==3)
	{
		shoot_count=0;
		Hit_speed.pid_out=0;
		key_flag=1;
		position_hit+=0;  
		key_mouse_inf.con_shoot_flag=0;
	}
		
	if(val==2&&hit_flag)                //长时间处于下档 则连发
	{
		
		if(shoot_count<PRESS_LONG_TIME)
		{
		  shoot_count++;
			key_mouse_inf.con_shoot_flag=0;
		}
		else
		{
			key_mouse_inf.con_shoot_flag=1;
		}
		if(key_mouse_inf.con_shoot_flag&&Cartridge_Flag==0)
		{
			if(__fabs(Dial_the_motor.ecd_angle-position_hit)<SONEBULLET_DISTANCE)   
			{
					position_hit+=DIAL_ONE_POSITION;
					key_mouse_inf.shoot_flag=1;
					key_mouse_inf.shoot_flag_back_flag=1;
			}
		}
		else if(key_mouse_inf.last_val==3)
		{
			if(key_mouse_inf.shoot_mode==1&&__fabs(Dial_the_motor.ecd_angle-position_hit)<SONEBULLET_DISTANCE)//前一颗子弹未发出 该子弹不能发出
			{
			  position_hit+=DIAL_ONE_POSITION;   //单发1620 
			}
			key_mouse_inf.shoot_flag=1;
			key_mouse_inf.shoot_flag_back_flag=1;
		}
		cartridge();
	}
	key_mouse_inf.last_val=val;
}

/*******w a s d control**************/
/*------输入: ramp------------------*/
/*------输出：无--------------------*/
/*------功能：解算四轮速度----------*/
void WalkControl(ramp_t ramp)
{
	switch(ramp)
	{
		case CHASSIS_RAMP_FB:
		{
					if(key_mouse_inf.no_twisted==0||key_mouse_inf.rotate==1)   //正在自旋
					{
						if(key_mouse_inf.w_flag)   //此时w键按下
						{
						  Chassis_Speed_Ref.forward_back_ref = Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						  Chassis_Speed_Ref.left_right_ref = -Chassis_Walk_Speed * Slope(ACCELERA_TIME,CHASSIS_RAMP_RL)*mechanical_angle_PTZ_Y.sina;
						}
						else if(key_mouse_inf.s_flag)  //s
						{
							Chassis_Speed_Ref.forward_back_ref = -Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						  Chassis_Speed_Ref.left_right_ref = Chassis_Walk_Speed * Slope(ACCELERA_TIME,CHASSIS_RAMP_RL)*mechanical_angle_PTZ_Y.sina;
						}
					}
					else
					{
						if(key_mouse_inf.w_flag)
						{
						  Chassis_Speed_Ref.forward_back_ref = Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						}
						else if(key_mouse_inf.s_flag)
						{
							Chassis_Speed_Ref.forward_back_ref = -Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						}
					}
		}break;
		case CHASSIS_RAMP_RL:
		{
					if(key_mouse_inf.no_twisted==0||key_mouse_inf.rotate==1)   //正在自旋
					{
						if(key_mouse_inf.d_flag)   //此时d键按下  -/-  +/- -/+  +/+
						{
						  Chassis_Speed_Ref.forward_back_ref = -Chassis_Walk_Speed * Slope(ACCELERA_TIME,CHASSIS_RAMP_FB)*mechanical_angle_PTZ_Y.sina;
						  Chassis_Speed_Ref.left_right_ref = -Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						}
						else if(key_mouse_inf.a_flag)
						{
							Chassis_Speed_Ref.forward_back_ref = Chassis_Walk_Speed * Slope(ACCELERA_TIME,CHASSIS_RAMP_FB)*mechanical_angle_PTZ_Y.sina;
						  Chassis_Speed_Ref.left_right_ref = Chassis_Walk_Speed * Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						}
							
					}
					else
					{
						if(key_mouse_inf.d_flag)
						  Chassis_Speed_Ref.left_right_ref =-Chassis_Walk_Speed*Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
						else if(key_mouse_inf.a_flag)
							Chassis_Speed_Ref.left_right_ref =Chassis_Walk_Speed*Slope(ACCELERA_TIME,ramp)*mechanical_angle_PTZ_Y.cosa;
					}
					

		}break;
	default:
			break;
	}

	
}

/* ------------------------ */
/* 卡弹处理                  */
/* 不能频繁进入卡弹处理       */
/* ------------------------ */
void cartridge()
{
	static u8 bullet_count=0;
	static float last_angle=0;
	if(key_mouse_inf.con_shoot_flag==1)  //连发时采用不同方法处理卡单
	{
			if(Cartridge_Flag==0)  //连发 前提是不卡弹
			{
				if((Hit_position.pid_out>REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED)  //判断是否卡单
				{
					Cartridge_Flag=1;   //标志连发过程中卡弹
					last_angle=Dial_the_motor.ecd_angle;  //记录卡弹时的角度
				}
			}
			else if(Cartridge_Flag==1)  //可以连发 但此时卡弹 等倒弹完成后继续连发
			{
				position_hit=Dial_the_motor.ecd_angle-DIAL_ONE_POSITION*0.5;  //倒转50度
				if((last_angle-Dial_the_motor.ecd_angle)>(DIAL_ONE_POSITION*0.5)*0.2 )  //倒弹是否完成
					Cartridge_Flag=0;
				if((Hit_position.pid_out<-REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED)  //判断是否卡单
				{
					position_hit-=__fabs(last_angle-Dial_the_motor.ecd_angle);  //倒转50度
					Cartridge_Flag=0;   //标志倒弹时卡弹
				}
			}
  }
	else if((Hit_position.pid_out<-REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED&&key_mouse_inf.shoot_flag)
	{
		bullet_count++;
		if(bullet_count>50)
		{
			position_hit=Dial_the_motor.ecd_angle+DIAL_BACK_POSITION;
			key_mouse_inf.shoot_flag=0;
			key_mouse_inf.shoot_flag_back_flag=1;
			bullet_count=0;
		}
	}
	else if((Hit_position.pid_out>REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED&&key_mouse_inf.shoot_flag_back_flag)
	{
		bullet_count++;
		if(bullet_count>50)
		{
			position_hit=Dial_the_motor.ecd_angle-DIAL_BACK_POSITION;
			key_mouse_inf.shoot_flag_back_flag=0;
			key_mouse_inf.shoot_flag=1;
			bullet_count=0;
		}
	}
	else
		bullet_count=0;
}

/* ----------------------------------- */
/* 发弹控制                             */
/* ----------------------------------- */

void Shoot_Control(u8 shoot_mode)
{
	volatile static u16 bullet_heat=0;       
  volatile static u16 DIAL_POSITION=0;            //拨弹距离
  volatile static u16 bullet_dis=0;               //射击距离
  volatile static u16 BULLET_DISTANCE=0;          //距离敏感度 
	volatile static int level_last=0;
	if(shoot_mode||key_mouse_inf.con_shoot_flag)  //单发
	{
		DIAL_POSITION=DIAL_ONE_POSITION;
		bullet_heat=refree_change_inf.one_bullet_heat;
		bullet_dis=refree_change_inf.one_bullet_dis;
		BULLET_DISTANCE=SONEBULLET_DISTANCE;
		Hit_position.Ki=HIT_POSITION_KI;
		Hit_speed.Kp=HIT_SPEED_KP;
	}
	else
	{
		Hit_position.Ki=0.01;
		Hit_speed.Kp=3.1;
		DIAL_POSITION=DIAL_THREE_POSITION;
		bullet_heat=refree_change_inf.three_bullet_heat;
		bullet_dis=refree_change_inf.three_bullet_dis;
		BULLET_DISTANCE=STHREEBULLET_DISTANCE;
	}
	#if  SHOOT_FEEDBACK==1     //位置环+裁判系统 1220
				if(ext_power_heat_data.heat_count>ext_power_heat_data.heat_count_last)  //如果从裁判系统接收到了新值 
				{
					ext_power_heat_data.shoot_bullet_number=(max_heat-ext_power_heat_data.shooter_heat0)/bullet_heat;      //当前可发射弹丸数量
					if(ext_power_heat_data.shoot_bullet_number>1)  //如果剩余热量值可以发送一颗弹丸
					{
						distance=__fabs(Dial_the_motor.ecd_angle-position_hit);
						if(key_mouse_inf.con_shoot_flag&&__fabs(Dial_the_motor.ecd_angle-position_hit)<BULLET_DISTANCE)
						{
							position_hit+=DIAL_POSITION;
							//position_hit+=ext_power_heat_data.shoot_bullet_number*DIAL_POSITION;   //转动一个子弹的距离
						}
						else if(key_mouse_inf.con_shoot_flag==0&&__fabs(Dial_the_motor.ecd_angle-position_hit)<BULLET_DISTANCE)
							position_hit+=DIAL_POSITION;   //转动一个子弹的距离
						key_mouse_inf.shoot_flag=1;      //发射标志位置1
						key_mouse_inf.shoot_flag_back_flag=1;
						ext_power_heat_data.heat_count_last=ext_power_heat_data.heat_count;
					}
				 }
#elif SHOOT_FEEDBACK==2||SHOOT_FEEDBACK==3   //位置环+电流计
				 if(level!=level_last)
				{
						switch(level)
						{
							case 1: ShootingFreCon.cooling_heat=4.0f;   //每100ms热量冷却值
											ShootingFreCon.limit_heat=240;       //等级为1时的热量上限
											refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
											level_last=level;
							break;
							case 2: ShootingFreCon.cooling_heat=6.0f;//每100ms热量冷却值
											ShootingFreCon.limit_heat=360;    //等级为2时的热量上限
											refree_change_inf.one_bullet_dis=MONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=MTHREEBULLET_DISTANCE;
											level_last=level;
							break;
							case 3: ShootingFreCon.cooling_heat=8.0f;//每100ms热量冷却值
											ShootingFreCon.limit_heat=480;    //等级为3时的热量上限
											refree_change_inf.one_bullet_dis=LONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=LTHREEBULLET_DISTANCE;
											level_last=level;
							break;
						}
				}
				if(Get_Time5_Micros()>=ShootingFreCon.LastcountTime+10000)//系统每经过100ms就会刷新一次热量
				{
					ShootingFreCon.remain_heat+=(Get_Time5_Micros()-ShootingFreCon.LastcountTime)/10000.0*ShootingFreCon.cooling_heat;//热量冷却
					ShootingFreCon.LastcountTime=Get_Time5_Micros();   //更新计时
					if(ShootingFreCon.remain_heat>ShootingFreCon.limit_heat)  //热量限幅
						ShootingFreCon.remain_heat=ShootingFreCon.limit_heat;
				}
				if(ShootingFreCon.remain_heat>bullet_heat)  //如果当前的热量允许发射一颗子弹
				{
					ShootingFreCon.Shooting_Flag=1;
				}
				if(ShootingFreCon.Shooting_Flag)
				{
					if(key_mouse_inf.con_shoot_flag)
					{
						position_hit+=(ShootingFreCon.remain_heat/bullet_heat)*DIAL_POSITION;  
					}
					else
					{
						position_hit+=DIAL_POSITION;
					}
						key_mouse_inf.shoot_flag=1;
						key_mouse_inf.shoot_flag_back_flag=1;
						ShootingFreCon.Shooting_Flag=0;
						ShootingFreCon.remain_heat-=refree_change_inf.one_bullet_heat;//热量累加
						if(ShootingFreCon.remain_heat<0)//热量限幅
								ShootingFreCon.remain_heat=0;
			  }
				
#else //位置环 无裁判系统
				if(distance<BULLET_DISTANCE)   
				{
					position_hit+=DIAL_POSITION;
					key_mouse_inf.shoot_flag=1;
					key_mouse_inf.shoot_flag_back_flag=1;
				}
#endif              
}



/**
  * @brief          遥控器离线解决函数->重启串口的DMA接收
  * @author         lgy
  * @param[in]      参数空
  * @retval         返回空
  */
void Remote_Offline_Slove()
{
			HAL_UART_DMAStop(&huart1);    //停止DMA接收
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);  //空闲中断失能
			__HAL_UART_DISABLE(&huart1);
			
			__HAL_UART_ENABLE(&huart1);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //DBUS遥控器接收  空闲中断
      HAL_UART_Receive_DMA(&huart1, usart_dma_bf, BUFLEN);
}






