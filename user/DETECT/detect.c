/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect.c/h
  * @brief      设备离线判断任务，通过freeRTOS滴答时间作为系统时间，设备获取数据后
  *             调用DetectHook记录对应设备的时间，在该任务会通过判断记录时间与系统
  *             时间之差来判断掉线，同时将最高的优先级的任务通过LED的方式改变，包括
  *             八个流水灯显示SBUB遥控器，三个云台上的电机，4个底盘电机 一个拨弹电机
	*             一个MinPC 还有裁判系统
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Fec-25-2019     lgy             1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "include.h"
error_t errorList[errorListLength + 1];  //记录设备信息
uint8_t rotate_revive_flag=0;  //自旋状态下复活

/**
  * @brief          重要设备状态信息初始化
  * @author         lgy
  * @param[in]      参数空
  * @retval         返回空
  */
void DetectInit(void)
{
	  uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    //设置离线时间，上线稳定工作时间 offlineTime onlinetime 
    uint32_t setItem[errorListLength][2] =
		{
			  {100, 40}, //Remote
				
				{100, 20}, //chassis 1
				{100, 20}, //chassis 2
				{100, 20}, //chassis 3
				{100, 20},  //chassis 4
				
				{100, 20},   //yaw
				{100, 20},   //pitch
				{100, 20},  //HIT
				
				{100, 10},  //MinPC
				
				{100, 10},  //Judge
     };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime = setItem[i][1];
        errorList[i].solveOfflineFun = NULL;

        errorList[i].enable = 1;
        errorList[i].isLost = 1;
        errorList[i].newTime = systemTime;
        errorList[i].lastTime = systemTime;
        errorList[i].Losttime = systemTime;
        errorList[i].worktime = systemTime;
    }
		errorList[RemoteTOE].solveOfflineFun=Remote_Offline_Slove;

}

/**
  * @brief          设备数据更新钩子函数
  * @author         lgy
  * @param[in]      设备号
  * @retval         返回空
  */
void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime = xTaskGetTickCount();

    if (errorList[toe].isLost)
    {
        errorList[toe].isLost = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
}


/**
  * @brief         指示灯显示状态信息
  * @author         lgy
  * @param[in]      参数空
  * @retval         返回空
  */
void DetectDisplay(void)
{
    uint8_t i = 0;
    for (i = 1 ;i <=8; i++)
    {
        if (errorList[i].isLost)    //底盘电机 云台 拨弹电机  H开始
        {
            FLOW_LED_ON(i);
        }
        else
        {
            FLOW_LED_OFF(i);
        }
				
    }
		
		if(errorList[0].isLost)  //DBUS
		{
			inputmode=OFFLINE;
			LED_Remote_On();
		}
		else                          //else语句中不用重新设置输入模式 如果遥控器未离线 在遥控器任务中会自动改变输入模式
		{
			LED_Remote_Off();
		}
		
		if(errorList[9].isLost)   //Judge
		{
			LED_Judge_On();
		}
		else
		{
			LED_Judge_Off();
		}
		
		//裁判系统未接受到数据 则采用自己的设备自检程序
		if(Init_End_Flag)
		{
			 #if REFEREE_SYSTEM
				if((errorList[5].isLost||errorList[6].isLost)&&ext_game_robot_survivors.survivor_flag)  //步兵存亡检测
				{
						vTaskSuspend(RemoteTaskHandle);
						vTaskSuspend(GimbalTaskHandle);
						vTaskSuspend(ChassisTaskHandle);
						Chassis_Speed_Ref_Init();
						
						ext_game_robot_survivors.survivor_flag=0;	
				}
				else if(!ext_game_robot_survivors.survivor_flag&&errorList[5].isLost==0&&errorList[6].isLost==0)
				{
						struct_clear();
					  Friction_Init();                        //摩擦轮初始化
						#if MPU_STATUS!=MPU_USART
							angle_offset=yaw_angle;
						#else
							angle_offset=yaw_usart_angle;
						#endif
							vTaskResume(ChassisTaskHandle);	
							vTaskResume(GimbalTaskHandle);
							vTaskResume(RemoteTaskHandle);
							ext_game_robot_survivors.survivor_flag=1;
				}
				#endif
		}
}

/**
  * @brief          蜂鸣器相关控制函数
  * @param[in]      预装载值 PWM
  * @retval         返回空
  */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    TIM12->PSC = psc;
    TIM_SetTIM12Compare1(pwm);
} 
void buzzer_off(void)
{
    TIM_SetTIM12Compare1(0);
}

void Init_Buzzer_Tip(void)
{
	static u32 buzzer_time=0;
  buzzer_time++;
	if (buzzer_time > (RCCALI_BUZZER_CYCLE_TIME-sqrt(soft_js_time*1600)) )
	{
			buzzer_time = 0;
	}
	else if (buzzer_time > (RC_CALI_BUZZER_PAUSE_TIME-sqrt(soft_js_time*400)) )
	{
			buzzer_off();
	}
	else
		buzzer_on(0,28000*sin(soft_js_time*0.00031415926));
}

//等待底盘 云台 拨弹电机上电
//全部上电后 返回 0 否则返回 1 
int Wait_Motor_Power_On(void)
{
	static u8 power_times=0;
	if(errorList[5].isLost==0&&errorList[6].isLost==0&&errorList[7].isLost==0)
  {
		Magaine_OFF();                          //关闭弹仓
		Friction_Init();                        //摩擦轮初始化
		power_times++;
		if(power_times>=5)  //确认上电成功
		{
			gimbal_power=1;
			power_times=0;
			return 0;
		}
		return 1;
	}
	else
	{
		power_times=0;
		return 1;
	}
}

void struct_clear(void)
{
	friction_wheel_state=FRICTION_WHEEL_OFF;  //关闭摩擦轮
	LASER_OFF();															//关闭激光
	key_mouse_inf.a_flag=0;
	key_mouse_inf.con_shoot_flag=0;
	key_mouse_inf.ctrl_flag=0;
	key_mouse_inf.c_flag=0;
	key_mouse_inf.d_flag=0;
	key_mouse_inf.e_flag=0;
	key_mouse_inf.e_times=0;
	key_mouse_inf.FB_flag=1;
	key_mouse_inf.FB_Press_flag=1;
	key_mouse_inf.f_flag=0;
	key_mouse_inf.last_val=0;
	key_mouse_inf.LR_Press_flag=1;
	key_mouse_inf.no_twisted=1;
	key_mouse_inf.play_flag=1;
	key_mouse_inf.q_flag=0;
	key_mouse_inf.rl_flag=1;
	key_mouse_inf.rotate=0;
	key_mouse_inf.rr_flag=1;
	key_mouse_inf.save_flag=0;
	key_mouse_inf.shift_flag=0;
	key_mouse_inf.shoot_flag=0;
	key_mouse_inf.s_flag=0;
	rotate_revive_flag=1;
	key_mouse_inf.no_twisted=1;
	key_mouse_inf.last_no_twisted=1;
	key_mouse_inf.last_rotate=0;
	key_mouse_inf.rotate=0;
	key_mouse_inf.rotate_to_middle=1;
	key_mouse_inf.w_flag=0;
	key_mouse_inf.z_flag=0;
	key_mouse_inf.shoot_mode=1;
	RC_CtrlData.mouse.last_press_r=0;
	RC_CtrlData.mouse.press_r=0;
/*	PID_Parment_Clear(&Chassis_Motor_PID_1);
	PID_Parment_Clear(&Chassis_Motor_PID_2);
	PID_Parment_Clear(&Chassis_Motor_PID_3);
	PID_Parment_Clear(&Chassis_Motor_PID_4);
	PID_Parment_Clear(&Hit_speed);
	PID_Parment_Clear(&Hit_position);
	PID_Parment_Clear(&Chassis_Motor_PID_rotate);
	PID_Parment_Clear(&PTZ_Motor_PID_Position_Y);
	PID_Parment_Clear(&PTZ_Motor_PID_Speed_Y);
	PID_Parment_Clear(&PTZ_Motor_PID_Position_P);
	PID_Parment_Clear(&PTZ_Motor_PID_Speed_P);
	PID_ADD_Parment_Clear(&Power_limit);*/
}






