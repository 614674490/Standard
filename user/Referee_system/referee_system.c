/*裁判系统的接受，用的是CPC校验  裁判系统发送给主控板的数据具有一定的格式*/
#include "stm32f4xx_hal.h"
#include "include.h"
//CRC-16:  G(X) = X16 + X15 + X2 + 1  0xc001

//from DJI manual V2.0
uint8_t Led[]={0,0,0,0,0,0};
u8 Led_Tip=0;
u16 max_heat=0;
u8 stdbuf[30] = {0};
u8 judge_buff[judgelen];  //裁判系统缓存BUFF   用DMA接收
ext_game_robot_survivors_t  ext_game_robot_survivors={0,1,0};   //默认存活
ext_game_robot_state_t ext_game_robot_state={0,0,0,0,0,0,0,0,0,0,0,0};
ext_power_heat_data_t ext_power_heat_data={0,0,0,0,0,0,0,0,0}; 
ext_robot_hurt_t ext_robot_hurt={0,0}; 
ext_shoot_data_t ext_shoot_data={0,0,0,0}; 
ext_student_interactive_header_data_t ext_student_interactive_header_data={0,0,0};
client_custom_data_t client_custom_data={0,0,0,0};
u8 gimbal_reset=0;
u32 gimbal_reset_jis=0;
/**
 * @brief 裁判系统数据处理
 * @param 裁判数据
 * @return 数据处理是否成功
 */
referee_system_StatusTypeDef referee_system_Rx(uint8_t * RxMsg, CRC_StatusTypeDef CRCSwitch)
{
	uint16_t CMD_code= (RxMsg[6] << 8) | RxMsg[5];
	uint8_t *Msg=&RxMsg[7];
	
	if (RxMsg[0] == 0xA5)
	{
		switch (CMD_code)
		{
#if Robot_state_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Robot_state_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_length_Robot_state_data))
				{
					Game_state_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Game_state_data_processing(Msg);
				return OK;
			}
#endif

#if Real_time_power_and_heat_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Real_time_power_and_heat_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_length_Real_time_power_and_heat_data))
				{
					Real_time_power_and_heat_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Real_time_power_and_heat_data_processing(Msg);
				return OK;
			}
#endif

#if Damage_status_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Damage_status_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_length_Damage_status_data))
				{
					Robot_hurt_status_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Robot_hurt_status_data_processing(Msg);
				return OK;
			}
#endif

#if Real_time_shooting_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Real_time_shooting_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_length_Real_time_shooting_data))
				{
					Real_time_Shoot_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Real_time_Shoot_data_processing(Msg);
				return OK;
			}
#endif
#if Match_robot_survival_data==REFEREE_SYSTEM_ENABLE
		case CMD_code_Match_robot_survival_data:
			if (CRCSwitch == CRC_ENABLE)
			{
				if (Verify_CRC16_Check_Sum(RxMsg, Data_length_Match_robot_survival_data))
				{
					Robot_survivors_data_processing(Msg);
					return OK;
				}
				else return CRC_ERROR;
			}
			else
			{
				Robot_survivors_data_processing(Msg);
				return OK;
			}
#endif
		default: return CMD_ERROR;
		}
	}
	return Error;
}
/**
 * @brief 实时功率热量数据处理函数 
 * @param 裁判数据
 * @other 50HZ
 */
void  Real_time_power_and_heat_data_processing(uint8_t *Msg)
{
	standardize std;

	ext_power_heat_data.chassis_volt=(((int16_t)Msg[1]<<8|Msg[0])/VOLTAGE_OFFSET);      //电压值 mV
	ext_power_heat_data.chassis_current=(((int16_t)Msg[3]<<8|Msg[2])/CURRENT_OFFSET);   //电流值 mA
	std.judge_buff[0]=Msg[4];
	std.judge_buff[1]=Msg[5];
	std.judge_buff[2]=Msg[6];
	std.judge_buff[3]=Msg[7];
	ext_power_heat_data.chassis_power=std.F;                //功率值   W
	ext_power_heat_data.chassis_power_buffer=((int16_t)Msg[9]<<8|Msg[8]);   //功率缓冲能量  J
	ext_power_heat_data.shooter_heat0=((int16_t)Msg[11]<<8|Msg[10]);    //17mm弹丸当前枪口热量
	ext_power_heat_data.heat_count++;
	
	CAP_Send.Data.chassis_power_buffer=ext_power_heat_data.chassis_power_buffer;//向超级电容端发送缓冲能量
	CAN2_Send_Msg_CAP(CAP_Send.buff);
	
}

/**
 * @brief 比赛机器人状态数据处理
 * @param 裁判数据
 * @other 10HZ
 */
void Game_state_data_processing(uint8_t *Msg)
{
	#if SHOOT_FEEDBACK==1 || SHOOT_FEEDBACK==3 
	ext_game_robot_state.robot_id = Msg[0];
	ext_game_robot_state.robot_level = Msg[1];
	ext_game_robot_state.remain_HP = ((int16_t)Msg[3] << 8 | Msg[2]);
	ext_game_robot_state.max_HP = ((int16_t)Msg[5] << 8 | Msg[4]);
	ext_game_robot_state.shooter_heat0_cooling_rate = ((int16_t)Msg[7] << 8 | Msg[6]);
	ext_game_robot_state.shooter_heat0_cooling_limit = ((int16_t)Msg[9]<<8|Msg[8]);
	max_heat=ext_game_robot_state.shooter_heat0_cooling_limit;
	ext_game_robot_state.state_count++;
	if(Msg[14]&0x01)
	{
		ext_game_robot_state.power_out=1;
	}
	else
	{
		ext_game_robot_state.power_out=0;
	}
	#if SHOOT_FEEDBACK==1
	switch(ext_game_robot_state.robot_level)   //根据当前等级选择距离敏感度
	{
		case Zero_Level:
										refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
										break;
		case One_Level:
										refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
										break;
		case Two_Level:
										refree_change_inf.one_bullet_dis=MONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=MTHREEBULLET_DISTANCE;
										break;
		case Three_Level:
										refree_change_inf.one_bullet_dis=LONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=LTHREEBULLET_DISTANCE;
										break;
	}
	#elif SHOOT_FEEDBACK==3			//裁判系统 只读取等级
	switch(ext_game_robot_state.robot_level)
	{
		case Zero_Level:ShootingFreCon.cooling_heat=HeatCollingSpeed_Lev1;
										ShootingFreCon.limit_heat=LimitHeat_Lev1;
										refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
										break;
		case One_Level: ShootingFreCon.cooling_heat=HeatCollingSpeed_Lev1;
										ShootingFreCon.limit_heat=LimitHeat_Lev1;
										refree_change_inf.one_bullet_dis=SONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=STHREEBULLET_DISTANCE;
										break;
		case Two_Level: ShootingFreCon.cooling_heat=HeatCollingSpeed_Lev2;
										ShootingFreCon.limit_heat=LimitHeat_Lev2;
										refree_change_inf.one_bullet_dis=MONEBULLET_DISTANCE;
										refree_change_inf.three_bullet_dis=MTHREEBULLET_DISTANCE;
										Hit_position.Kd=ONE_KD-0.04f;
										break;
		case Three_Level:ShootingFreCon.cooling_heat=HeatCollingSpeed_Lev3;
										 ShootingFreCon.limit_heat=LimitHeat_Lev3;
										 refree_change_inf.one_bullet_dis=LONEBULLET_DISTANCE;
										 refree_change_inf.three_bullet_dis=LTHREEBULLET_DISTANCE;
										 Hit_position.Kd=ONE_KD-0.08f;
										 break;
	}
	#endif
				
	#endif
}

/**
 * @brief 实时射击信息数据
 * @param 裁判数据
 * @other 射击完成后发送
 */
void Real_time_Shoot_data_processing(uint8_t *Msg)
{
	standardize std;
	
	ext_shoot_data.bullet_type = Msg[0];
	ext_shoot_data.bullet_freq = Msg[1];
	std.judge_buff[0]=Msg[2];
	std.judge_buff[1]=Msg[3];
	std.judge_buff[2]=Msg[4];
	std.judge_buff[3]=Msg[5];
	ext_shoot_data.bullet_speed = std.F;
	ext_shoot_data.bullet_count++;
}


/**
 * @brief 机器人伤害状态信息
 * @param 裁判数据
 * @other 受到伤害后发送
 * @tip   装甲伤害
 */
void Robot_hurt_status_data_processing(uint8_t *Msg)
{
	ext_robot_hurt.armor_id = Msg[0]&0x0f; //低四位为受伤装甲板
	ext_robot_hurt.hurt_type = Msg[0]>>4;  //高四位为伤害类型
}

/**
 * @brief 机器人存活状态信息
 * @param 裁判数据
 * @other 1HZ
 */
void Robot_survivors_data_processing(uint8_t *Msg)
{
	static u8 check_id=0;
	static u8 last_survivors=1;
	ext_game_robot_survivors.survivor_count++;
	
	ext_game_robot_survivors.robot_legion=Msg[1]<<8|Msg[0];
	if(ext_game_robot_state.robot_id<=7)   //红方
		check_id=1<<(ext_game_robot_state.robot_id-1);
	else if(ext_game_robot_state.robot_id>=11)  //蓝方
		check_id=1<<(ext_game_robot_state.robot_id-3);
	if(ext_game_robot_survivors.robot_legion&&check_id&&last_survivors==0)  //存活 且上次是死亡
	{
		ext_game_robot_survivors.survivor_flag=1;
		#if MPU_STATUS!=MPU_USART
				angle_offset=yaw_angle;
		#else
				angle_offset=yaw_usart_angle;
		#endif
		vTaskResume(ChassisTaskHandle);
		vTaskResume(GimbalTaskHandle);
		vTaskResume(RemoteTaskHandle);
		last_survivors=1;
	}
	else if(!(ext_game_robot_survivors.robot_legion&&check_id)&& last_survivors==1)    //死亡 且上次是存活
	{
		ext_game_robot_survivors.survivor_flag=0;
		last_survivors=0;
		vTaskSuspend(RemoteTaskHandle);
		vTaskSuspend(GimbalTaskHandle);
		vTaskSuspend(ChassisTaskHandle);
	}
}

void Open_Client_LED(int led)
{
		Led[led-1]=1<<(led-1);
	  Led_Tip=Led[0]|Led[1]|Led[2]|Led[3]|Led[4]|Led[5];
}

void Close_Client_LED(int led)
{
		Led[led-1]=111110<<(led-1);		
	  Led_Tip=Led[0]&Led[1]&Led[2]&Led[3]&Led[4]&Led[5];
}

/**
 * @brief 客户端 机器人交互信息发送
 * @param 无
 * @other 上限10HZ
 * @element 灯：单发/三连发 开超级电容/关超级电容 开舱门/关舱门 开扭腰/关扭腰 开摩擦轮/关摩擦轮  开启自瞄/关闭自瞄 
 *              绿 /红        绿    /    红      绿  / 红     绿  / 红     绿  /  红       绿  / 红
 *               0  1  2  3  4 5
 *         默认： 绿 红 红 红 红 红 0x01
 *               1  0  0  0  0  0
 * @selfdeinefloatdata  超级电容电量  当前可发射弹丸数量 已发射弹丸数量 
 */
void Client_custom_robot_data_Send(void)
{
	standardize std;
	int id_ge=0,id_shi=0;
  stdbuf[0] = 0xA5; stdbuf[1] = 0x13; stdbuf[2] = 0x00; stdbuf[3] = 0x01;//帧头 数据长度 包号 校验和
	Append_CRC8_Check_Sum(stdbuf, 5);                                      //CRC校验 0-4为包头
	stdbuf[5] = 0x01;  stdbuf[6] = 0x03;                              //ID 0x0301 低位在前 高位在后
	stdbuf[7]=0x80,stdbuf[8]=0xd1;                       //内容ID 0xD180
	stdbuf[9]=ext_game_robot_state.robot_id,stdbuf[10]=0x00;                      //发送者ID 3/4/5步兵红
	id_ge=ext_game_robot_state.robot_id%10;
	id_shi=ext_game_robot_state.robot_id/10;
	stdbuf[11]=id_shi<<4|id_ge,stdbuf[12]=0x01;                     //客户端ID 0x0103...红

	std.F = CAP_volt;                                           //超级电容电量
	stdbuf[13] = std.judge_buff[0];   stdbuf[14] = std.judge_buff[1];
	stdbuf[15] = std.judge_buff[2];   stdbuf[16] = std.judge_buff[3];  //数据 1
	std.F = ext_power_heat_data.shoot_bullet_number;   //当前可发射弹丸数量
	stdbuf[17] = std.judge_buff[0]; stdbuf[18] = std.judge_buff[1];
	stdbuf[19] = std.judge_buff[2]; stdbuf[20] = std.judge_buff[3];  //数据 2
	std.F = ext_shoot_data.bullet_count;              //当前已发射弹丸数量ht
	stdbuf[21] = std.judge_buff[0]; stdbuf[22] = std.judge_buff[1];
	stdbuf[23] = std.judge_buff[2]; stdbuf[24] = std.judge_buff[3];  //数据 3  
	Led_Tip=key_mouse_inf.shoot_mode<<0|cap_flag<<1|key_mouse_inf.f_flag<<2|(!key_mouse_inf.no_twisted)<<3|friction_flag<<4|key_mouse_inf.r_flag<<5;           //0011 1111	0x3F					
	stdbuf[25]=  Led_Tip;			                       //自定义8位数据 00 00 0000  
	Append_CRC16_Check_Sum(stdbuf, 28);
	HAL_UART_Transmit(&huart6, stdbuf, 28, 0xff);
}

/**
 * CRC校验代码
 */
//crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] =
{
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int
dwLength,unsigned char ucCRC8)
{
unsigned char ucIndex;
while (dwLength--)
{
ucIndex = ucCRC8^(*pchMessage++);
ucCRC8 = CRC8_TAB[ucIndex];
}
return(ucCRC8);
}
/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
unsigned char ucExpected = 0;
if ((pchMessage == 0) || (dwLength <= 2)) return 0;
ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
return ( ucExpected == pchMessage[dwLength-1] );
}
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
unsigned char ucCRC = 0;
if ((pchMessage == 0) || (dwLength <= 2)) return;
ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;
//运用了查表的方法  CRC16校验
const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
** Descriptions: CRC16 checksum function   描述：CRC16校验和功能
** Input: Data to check,Stream length, initialized checksum  输入：要检查的数据，流长度，已初始化的校验和
** Output: CRC checksum   输出：CRC校验和
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC)
{
    uint8_t chData;
    if (pchMessage == NULL)
    {
        return 0xFFFF;
    }
    while(dwLength--)
    {
        chData = *pchMessage++;  //高位                  CRC高位              接受信息
        (wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) &
                 0x00ff];
    }
    return wCRC;
}

/*
** Descriptions: CRC16 Verify function   描述：CRC16验证功能
** Input: Data to Verify,Stream length = Data + checksum  输入：要验证的数据，流数据=数据+校验和
** Output: True or False (CRC Verify Result)  输出：真或假（CRC校验结果）
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))   //帧头帧尾
    {
        return 0;
    }
    wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) ==
            pchMessage[dwLength - 1]);     //CRC校验和/生成码 判断余数是否为零 为零则数据可靠，可以接收
}

/*
** Descriptions: append CRC16 to the end of data   描述：将CRC16附加到数据结尾   产生信息码+CRC校验码
** Input: Data to CRC and append,Stream length = Data + checksum   输入：数据到CRC并追加，流长度=数据+校验和
** Output: True or False (CRC Verify Result)   在数据末尾添加CRC16校验码
*/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
    pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}

