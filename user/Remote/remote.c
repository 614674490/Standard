#include "include.h"
/*  ң���� ��� ���� ���ƺ��� */
float  mouse_yaw;
int Cartridge_Flag=0;                                 //������־λ
RC_Ctl_t RC_CtrlData={0};                             //ң�����������ݽṹ��
Gimbal_Ref_t GimbalRef={0};                           //��̨�˶������ṹ��                        
ShootingFreCon_t ShootingFreCon={1,0,0,240,240,4.0};  //ʵ������ �������� ÿ����ȴֵ
OPERATE_MESSAGE  key_mouse_inf={1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1};  //��� ���� ��ؼ�ֵ��־λ
u8 friction_flag=0;                                   //Ħ���ֿ�����־λ
REFEREE_LIMIT_MESSAGE refree_change_inf={0};          //Ħ�����ٶ� ������Ƶ�������ж� �����ṹ��
FrictionWheelState_e friction_wheel_state=FRICTION_WHEEL_OFF; //ö�� Ħ����״̬
InputMode_e inputmode=OFFLINE;                        //ң������ǰģʽ Ĭ��Ϊ����
u8 ws_flag=0,ad_flag=0;      //w/s���±�־λ a/d���±�־λ
u8 i = 0;                    //����STOPֹͣ�������ݺ��������ʹ��
int q_count = 0, e_count = 0, z_count=0,g_count=0,v_count=0;   //���� Ť�������̶� ��̨������ �ļ�������
u8 level=1;      //�ȼ�
int up_count;   //������ʱ����
int down_count;   ///������ʱ����
volatile float angle_offset=0;      //��¼�ϵ��ʼ��ʱY�����ĳ�ʼ����ֵ ������̨�Ĺ��к�����
volatile float pitch_angle_offset=0;      //��¼�ϵ��ʼ��ʱY�����ĳ�ʼ����ֵ ������̨�Ĺ��к�����
float distance;        //���ڼ��㲦�������ǰλ�ú�Ŀ��λ�õľ���
int Dial_angle_offset=0;  //�����ǶȲ���
float changehead_rorate,changehead_y;  //������̨����ǹ

/**************************ʵ�ֺ���********************************************
*����ԭ��:	   MouseShootControl(Mouse *mouse)
*��������:	   ���跢�����  
���������   ��굱ǰ״̬
���������   û��
*******************************************************************************/
void MouseShootControl(Mouse *mouse)
{
	static int16_t closeDelayCount=0;   //�Ҽ��ر�Ħ������ʱ����   �ӹرյ����� �����Ҽ� ʱ������Ļ� Ħ�����ֻ�ر�; �ӿ������ر� �Ҽ��������ر� ʱ�����Ҳ��ر�

		switch (friction_wheel_state)  //����Ħ���ֵ�ǰ״̬�����л�
		{
			case FRICTION_WHEEL_OFF:
			{
				friction_flag=0;
				if(mouse->last_press_r==0&&mouse->press_r==1)  //�ӹرյ���ʼת��
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
				if(mouse->press_r==1)                    //�Ҽ��������� �ر�Ħ����
				{
					closeDelayCount++;
				}
				else
				{
					closeDelayCount=0;
				}
				if(closeDelayCount>FRICITION_CLOSE_DELAY)   //����ʱ��ﵽ�ر�ʱ��
				{
					friction_wheel_state=FRICTION_WHEEL_OFF;
					Friction_OFF();
				}
				else    //δ�ر� ��״̬�л�������
				{
					//Ħ���ּ���
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
	//ת�����������ǰ����Ħ���ֿ���  ��Ȼ����
	if(mouse->last_press_l==0&&(mouse->press_l==1)&&friction_wheel_state==FRICTION_WHEEL_ON) 
	{
		Shoot_Control(key_mouse_inf.shoot_mode);
	}	
	cartridge();	
	mouse->last_press_r=mouse->press_r;  //��¼��һ������Ҽ�״̬
	if(key_mouse_inf.con_shoot_flag==1)  //Q����
		mouse->last_press_l=0;
	else
		mouse->last_press_l=mouse->press_l;  //��¼��һ��������״̬
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:	   void MouseKeyControlProcess(Mouse *mouse, Key *key)
*��������:	   ����ģʽ����
���������   �������̵�״̬
���������   û��
*******************************************************************************/
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	if(key->v &0x10)  //shift
	{
		key_mouse_inf.shift_flag=1;
		Chassis_Walk_Speed=HIGH_SPEED;
		rotate_k=FAST_ROTATE_K;
		if(CAP_volt==0)  //ͨ��ʧ��ʱ ��������ʻ
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
		if(key_mouse_inf.FB_flag==0)  //�ǲ������°���
		{
			ResetSlope(ramp);	          //�ͷ�б��
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
			ResetSlope(ramp);	          //�ͷ�б��
			ResetSlope(CHASSIS_RAMP_RL);   
			key_mouse_inf.play_flag=1;
		}
		WalkControl(ramp);
		key_mouse_inf.FB_flag=0;
		key_mouse_inf.s_flag=0;
		ws_flag=1;
		key_mouse_inf.FB_Press_flag=1;
	}
	else  //δ����
	{
		key_mouse_inf.play_flag=0;
    key_mouse_inf.FB_flag=0;
		ws_flag=0;
		if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1)  //��Ť�� ������
		{
			ramp = CHASSIS_RAMP_FB;
			if(key_mouse_inf.FB_Press_flag==1)  //�ж��ϴ��Ƿ��� ֻ�ͷ�һ��б��
			{
				ResetSlope(ramp);               //���û�а����ͷ�б��
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
  else  //δ����
	{
		key_mouse_inf.rl_flag=0;
		key_mouse_inf.rr_flag=0;
		ad_flag=0;
		if(key_mouse_inf.no_twisted==1&&key_mouse_inf.rotate_to_middle==1)   //������״̬
		{
		  ramp = CHASSIS_RAMP_RL;
			if(key_mouse_inf.LR_Press_flag==1)
			{
			  ResetSlope(ramp);
				key_mouse_inf.LR_Press_flag=0;
			}
			Chassis_Speed_Ref.left_right_ref *=(1.0f-Slope(DECELERATE_TIME,ramp));  //��������ʱҲҪʹ��б�º���
		}
		else if(ws_flag==0)                         //����������������δ���� ��ֹ���ְ���ʧ�������Լ�����������
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
	
	if(key->v & KEY_F)      //F ������
	{
		  Magaine_ON();   //F���� �����
			key_mouse_inf.f_flag=1;
	}
	else
	{
		
		Magaine_OFF();   //F�ɿ� �رյ���
		key_mouse_inf.f_flag=0;
		ext_shoot_data.bullet_count=0;  //����ʱ ֮ǰ�ĵ����������
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
	if(key->v & 0x40)  //q  ��������
	{
			refree_change_inf.fricition_speed=LOW_FRICITION_ON_SPEED;  //����Ħ���ֵ�ת���ٶ�
			refree_change_inf.one_bullet_heat=SLOW_ONEBULLET_HEAT;  //�����ӵ�����������
			key_mouse_inf.con_shoot_flag=1;                   //�����־λ
	}
	else
	{//NORMAL_FRICITION_ON_SPEED
		if(key_mouse_inf.shoot_mode==1)  //����ģʽ
		{
			refree_change_inf.fricition_speed=NORMAL_FRICITION_ON_SPEED;  //Ħ���ֵ��ٶ�Ϊ����
			refree_change_inf.one_bullet_heat=FAST_ONEBULLET_HEAT;  //�����ӵ��������ϸ�
		}			
		else  //������
		{
			refree_change_inf.fricition_speed=MIDDLE_FRICITION_ON_SPEED;  //Ħ���ֵ��ٶ�Ϊ����
			refree_change_inf.three_bullet_heat=FAST_THREEBULLET_HEAT;
		}
		key_mouse_inf.con_shoot_flag=0;
	}
	
	if(key->v & 0x80)  //e
	{
		//Ť������
		e_count++;
		if(e_count>KEY_PRESS_DELAY)      //����һ��ʱ���
		{
				key_mouse_inf.save_flag = !key_mouse_inf.save_flag;
		    key_mouse_inf.no_twisted = !key_mouse_inf.no_twisted;
			  if(key_mouse_inf.rotate==1&&key_mouse_inf.no_twisted==0) //�����л�ΪŤ��
				{
					key_mouse_inf.last_rotate=0;
					key_mouse_inf.rotate=0;
					key_mouse_inf.rotate_to_middle=0;                      //����������������
				}
			  key_mouse_inf.last_no_twisted=key_mouse_inf.no_twisted;
			  e_count = -200;                 //��ֹ�ظ�����
		}
  }
	else
	{
		e_count=0;
	}
	#if INFANTRY!=3
	if(key->v & KEY_V)  //V С���ݰ���
	{
		v_count++;
		if(v_count>KEY_PRESS_DELAY)      //����һ��ʱ���
		{
			key_mouse_inf.rotate=!key_mouse_inf.rotate;  //����/�ر�����
			if(key_mouse_inf.rotate==1&&key_mouse_inf.last_rotate==0)   //�ɲ�������Ϊ����
			{
				key_mouse_inf.rotate_to_middle=0;  //ʼ���ǹ���δ��� ��chassis.c�иı���ֵ
				if(key_mouse_inf.no_twisted==0)  //Ť���л�Ϊ����
				{
					key_mouse_inf.no_twisted=1;
					key_mouse_inf.save_flag=0;
				}
			}
			key_mouse_inf.last_rotate=key_mouse_inf.rotate;
			v_count = -200;                 //��ֹ�ظ�����
		}
  }
	else
	{
		v_count=0;
	}
  if(key->v & KEY_Z)  //z �л�ͷ�ķ���
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
	if(key->v & KEY_G)                       //G������ ����λ ֻ���ڳ��������²���ʹ��
	{
		g_count++;
		if(g_count>KEY_PRESS_DELAY)      //����һ��ʱ���
		{
			for(i = 0; i<8; i++)
			{
				databuf[i] = 0;
				databuf_PTZ[i] = 0;
			}
			(*Chaais_Send)(databuf);
			CAN1_Send_Msg_PTZ(databuf_PTZ);
			__set_FAULTMASK(1);//�ر������ж�
			NVIC_SystemReset();//��λ����
		}
	}
	else
		g_count=0;
	#if INFANTRY!=5
	if(key->v & KEY_R)//����    ÿ����һ���л�һ��
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
	
		if(key->v & KEY_X)//Ѳ��
	{
		key_mouse_inf.x_flag=1;
	}
	else
	{
		key_mouse_inf.x_flag=0;
	}
	#endif
		//����޷�
		mouse_yaw=mouse->x;
		VAL_LIMIT(mouse->x,-43,43);
		VAL_LIMIT(mouse->y,-120,120);
	if(key_mouse_inf.autoaim_mode==0||(y_angle==0&&p_angle==0))//δ���������������״̬��δ��⵽Ŀ�� ���Ϊ�ֶ�������̨
	{
		GimbalRef.pitch_angle_dynamic_ref-=mouse->y*MOUSE_TO_PITCH_ANGLE_INC_FACT;
		GimbalRef.yaw_angle_dynamic_ref+=mouse->x*MOUSE_TO_YAW_ANGLE_INC_FACT;
	}
	VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,PITCH_DOWM_LIMIT,PITCH_UP_LIMIT);
		MouseShootControl(mouse);
}


//����ģʽ���� 
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

InputMode_e GetInputMode()   //������װ
{
	return inputmode;
}

/**
 *@brief ң�������ݴ���
 * const u8 *Msg(����ָ��) : ָ��ָ��ı�����ֵ����ͨ����ָ���޸ģ�����ָ��ָ���ֵ���Ըı䡣 ��ֹ���������
 * ��ָ�볣��ֻ��ָ��һ��ָ�룬����ָ������ָ�� u8 *const Msg
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
					//ң��������ģʽ
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
					//���� ����ģʽ
					//expect_speed=20;
					switch_flag=0;
					MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
				}break;
				case STOP:
				{
					//��ͣģʽ
					Friction_OFF();
					RemoteControlProcess(&(RC_CtrlData.rc));
					PID_Clear();
				}break;
				default:
					break;
		 }
}

//ң��������ģʽ����
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

//ң����Ħ����
void RemoteShootControl(uint8_t val) 
{
	static u8 key_flag=1;
	static u8 hit_flag=0;
	static u16 Friction_wheel=0;
	static u16 shoot_count=0;
	if(key_flag&&val==1)
	{
		shoot_count=0;
		Hit_speed.pid_out=0;                             //�������ֹͣת��
		key_flag=0;
		if(Friction_wheel%2==0)
		{
			friction_wheel_state=FRICTION_WHEEL_ON;
			#if FRICTION_420S_STATUS==0
			Friction_ON(NORMAL_FRICITION_ON_SPEED);
			LASER_ON();                                     //��������
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
		
	if(val==2&&hit_flag)                //��ʱ�䴦���µ� ������
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
			if(key_mouse_inf.shoot_mode==1&&__fabs(Dial_the_motor.ecd_angle-position_hit)<SONEBULLET_DISTANCE)//ǰһ���ӵ�δ���� ���ӵ����ܷ���
			{
			  position_hit+=DIAL_ONE_POSITION;   //����1620 
			}
			key_mouse_inf.shoot_flag=1;
			key_mouse_inf.shoot_flag_back_flag=1;
		}
		cartridge();
	}
	key_mouse_inf.last_val=val;
}

/*******w a s d control**************/
/*------����: ramp------------------*/
/*------�������--------------------*/
/*------���ܣ����������ٶ�----------*/
void WalkControl(ramp_t ramp)
{
	switch(ramp)
	{
		case CHASSIS_RAMP_FB:
		{
					if(key_mouse_inf.no_twisted==0||key_mouse_inf.rotate==1)   //��������
					{
						if(key_mouse_inf.w_flag)   //��ʱw������
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
					if(key_mouse_inf.no_twisted==0||key_mouse_inf.rotate==1)   //��������
					{
						if(key_mouse_inf.d_flag)   //��ʱd������  -/-  +/- -/+  +/+
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
/* ��������                  */
/* ����Ƶ�����뿨������       */
/* ------------------------ */
void cartridge()
{
	static u8 bullet_count=0;
	static float last_angle=0;
	if(key_mouse_inf.con_shoot_flag==1)  //����ʱ���ò�ͬ����������
	{
			if(Cartridge_Flag==0)  //���� ǰ���ǲ�����
			{
				if((Hit_position.pid_out>REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED)  //�ж��Ƿ񿨵�
				{
					Cartridge_Flag=1;   //��־���������п���
					last_angle=Dial_the_motor.ecd_angle;  //��¼����ʱ�ĽǶ�
				}
			}
			else if(Cartridge_Flag==1)  //�������� ����ʱ���� �ȵ�����ɺ��������
			{
				position_hit=Dial_the_motor.ecd_angle-DIAL_ONE_POSITION*0.5;  //��ת50��
				if((last_angle-Dial_the_motor.ecd_angle)>(DIAL_ONE_POSITION*0.5)*0.2 )  //�����Ƿ����
					Cartridge_Flag=0;
				if((Hit_position.pid_out<-REDIAL_BULLET_POSITION)&&__fabs(Dial_motor_speed_fdb)<REDIAL_BULLET_SPEED)  //�ж��Ƿ񿨵�
				{
					position_hit-=__fabs(last_angle-Dial_the_motor.ecd_angle);  //��ת50��
					Cartridge_Flag=0;   //��־����ʱ����
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
/* ��������                             */
/* ----------------------------------- */

void Shoot_Control(u8 shoot_mode)
{
	volatile static u16 bullet_heat=0;       
  volatile static u16 DIAL_POSITION=0;            //��������
  volatile static u16 bullet_dis=0;               //�������
  volatile static u16 BULLET_DISTANCE=0;          //�������ж� 
	volatile static int level_last=0;
	if(shoot_mode||key_mouse_inf.con_shoot_flag)  //����
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
	#if  SHOOT_FEEDBACK==1     //λ�û�+����ϵͳ 1220
				if(ext_power_heat_data.heat_count>ext_power_heat_data.heat_count_last)  //����Ӳ���ϵͳ���յ�����ֵ 
				{
					ext_power_heat_data.shoot_bullet_number=(max_heat-ext_power_heat_data.shooter_heat0)/bullet_heat;      //��ǰ�ɷ��䵯������
					if(ext_power_heat_data.shoot_bullet_number>1)  //���ʣ������ֵ���Է���һ�ŵ���
					{
						distance=__fabs(Dial_the_motor.ecd_angle-position_hit);
						if(key_mouse_inf.con_shoot_flag&&__fabs(Dial_the_motor.ecd_angle-position_hit)<BULLET_DISTANCE)
						{
							position_hit+=DIAL_POSITION;
							//position_hit+=ext_power_heat_data.shoot_bullet_number*DIAL_POSITION;   //ת��һ���ӵ��ľ���
						}
						else if(key_mouse_inf.con_shoot_flag==0&&__fabs(Dial_the_motor.ecd_angle-position_hit)<BULLET_DISTANCE)
							position_hit+=DIAL_POSITION;   //ת��һ���ӵ��ľ���
						key_mouse_inf.shoot_flag=1;      //�����־λ��1
						key_mouse_inf.shoot_flag_back_flag=1;
						ext_power_heat_data.heat_count_last=ext_power_heat_data.heat_count;
					}
				 }
#elif SHOOT_FEEDBACK==2||SHOOT_FEEDBACK==3   //λ�û�+������
				 if(level!=level_last)
				{
						switch(level)
						{
							case 1: ShootingFreCon.cooling_heat=4.0f;   //ÿ100ms������ȴֵ
											ShootingFreCon.limit_heat=240;       //�ȼ�Ϊ1ʱ����������
											refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
											level_last=level;
							break;
							case 2: ShootingFreCon.cooling_heat=6.0f;//ÿ100ms������ȴֵ
											ShootingFreCon.limit_heat=360;    //�ȼ�Ϊ2ʱ����������
											refree_change_inf.one_bullet_dis=MONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=MTHREEBULLET_DISTANCE;
											level_last=level;
							break;
							case 3: ShootingFreCon.cooling_heat=8.0f;//ÿ100ms������ȴֵ
											ShootingFreCon.limit_heat=480;    //�ȼ�Ϊ3ʱ����������
											refree_change_inf.one_bullet_dis=LONEBULLET_DISTANCE;
											refree_change_inf.three_bullet_dis=LTHREEBULLET_DISTANCE;
											level_last=level;
							break;
						}
				}
				if(Get_Time5_Micros()>=ShootingFreCon.LastcountTime+10000)//ϵͳÿ����100ms�ͻ�ˢ��һ������
				{
					ShootingFreCon.remain_heat+=(Get_Time5_Micros()-ShootingFreCon.LastcountTime)/10000.0*ShootingFreCon.cooling_heat;//������ȴ
					ShootingFreCon.LastcountTime=Get_Time5_Micros();   //���¼�ʱ
					if(ShootingFreCon.remain_heat>ShootingFreCon.limit_heat)  //�����޷�
						ShootingFreCon.remain_heat=ShootingFreCon.limit_heat;
				}
				if(ShootingFreCon.remain_heat>bullet_heat)  //�����ǰ������������һ���ӵ�
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
						ShootingFreCon.remain_heat-=refree_change_inf.one_bullet_heat;//�����ۼ�
						if(ShootingFreCon.remain_heat<0)//�����޷�
								ShootingFreCon.remain_heat=0;
			  }
				
#else //λ�û� �޲���ϵͳ
				if(distance<BULLET_DISTANCE)   
				{
					position_hit+=DIAL_POSITION;
					key_mouse_inf.shoot_flag=1;
					key_mouse_inf.shoot_flag_back_flag=1;
				}
#endif              
}



/**
  * @brief          ң�������߽������->�������ڵ�DMA����
  * @author         lgy
  * @param[in]      ������
  * @retval         ���ؿ�
  */
void Remote_Offline_Slove()
{
			HAL_UART_DMAStop(&huart1);    //ֹͣDMA����
			__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);  //�����ж�ʧ��
			__HAL_UART_DISABLE(&huart1);
			
			__HAL_UART_ENABLE(&huart1);
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //DBUSң��������  �����ж�
      HAL_UART_Receive_DMA(&huart1, usart_dma_bf, BUFLEN);
}






