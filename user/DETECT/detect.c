/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect.c/h
  * @brief      �豸�����ж�����ͨ��freeRTOS�δ�ʱ����Ϊϵͳʱ�䣬�豸��ȡ���ݺ�
  *             ����DetectHook��¼��Ӧ�豸��ʱ�䣬�ڸ������ͨ���жϼ�¼ʱ����ϵͳ
  *             ʱ��֮�����жϵ��ߣ�ͬʱ����ߵ����ȼ�������ͨ��LED�ķ�ʽ�ı䣬����
  *             �˸���ˮ����ʾSBUBң������������̨�ϵĵ����4�����̵�� һ���������
	*             һ��MinPC ���в���ϵͳ
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Fec-25-2019     lgy             1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "include.h"
error_t errorList[errorListLength + 1];  //��¼�豸��Ϣ
uint8_t rotate_revive_flag=0;  //����״̬�¸���

/**
  * @brief          ��Ҫ�豸״̬��Ϣ��ʼ��
  * @author         lgy
  * @param[in]      ������
  * @retval         ���ؿ�
  */
void DetectInit(void)
{
	  uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    //��������ʱ�䣬�����ȶ�����ʱ�� offlineTime onlinetime 
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
  * @brief          �豸���ݸ��¹��Ӻ���
  * @author         lgy
  * @param[in]      �豸��
  * @retval         ���ؿ�
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
  * @brief         ָʾ����ʾ״̬��Ϣ
  * @author         lgy
  * @param[in]      ������
  * @retval         ���ؿ�
  */
void DetectDisplay(void)
{
    uint8_t i = 0;
    for (i = 1 ;i <=8; i++)
    {
        if (errorList[i].isLost)    //���̵�� ��̨ �������  H��ʼ
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
		else                          //else����в���������������ģʽ ���ң����δ���� ��ң���������л��Զ��ı�����ģʽ
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
		
		//����ϵͳδ���ܵ����� ������Լ����豸�Լ����
		if(Init_End_Flag)
		{
			 #if REFEREE_SYSTEM
				if((errorList[5].isLost||errorList[6].isLost)&&ext_game_robot_survivors.survivor_flag)  //�����������
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
					  Friction_Init();                        //Ħ���ֳ�ʼ��
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
  * @brief          ��������ؿ��ƺ���
  * @param[in]      Ԥװ��ֵ PWM
  * @retval         ���ؿ�
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

//�ȴ����� ��̨ ��������ϵ�
//ȫ���ϵ�� ���� 0 ���򷵻� 1 
int Wait_Motor_Power_On(void)
{
	static u8 power_times=0;
	if(errorList[5].isLost==0&&errorList[6].isLost==0&&errorList[7].isLost==0)
  {
		Magaine_OFF();                          //�رյ���
		Friction_Init();                        //Ħ���ֳ�ʼ��
		power_times++;
		if(power_times>=5)  //ȷ���ϵ�ɹ�
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
	friction_wheel_state=FRICTION_WHEEL_OFF;  //�ر�Ħ����
	LASER_OFF();															//�رռ���
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






