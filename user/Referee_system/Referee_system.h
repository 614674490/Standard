#ifndef __REFEREE_SYSTEM_H
#define __REFEREE_SYSTEM_H
/*裁判系统的一些相关数据*/
#include "stm32f4xx_hal.h"
#include "sys.h"

#define judgelen                                                   30            //裁判系统接收缓存数组的最大长度


#define referee_system_QueueLength                                 5   //队列项目个数
#define referee_system_ItemSize		                                 judgelen
#define threshold                                                  (referee_system_QueueLength-2)

#define REFEREE_SYSTEM_ENABLE                           			  	 1
#define REFEREE_SYSTEM_DISABLE                          				   0

#define Zero_Level            																  	 0     //没有等级
#define One_Level             																  	 1    //一级
#define Two_Level             																  	 2    //二级
#define Three_Level           																  	 3    //三级

#define LimitHeat_Lev1       																	  	 240    //一级热量上限
#define LimitHeat_Lev2       																	  	 360     //二级热量上限
#define LimitHeat_Lev3      																	  	 480     //三级热量上限

#define HeatCollingSpeed_Lev1                                      4.0f
#define HeatCollingSpeed_Lev2                                      6.0f
#define HeatCollingSpeed_Lev3                                      8.0f

#define JudgeFrameHeader                                           0xA5   //裁判系统数据的帧头

#define Match_status_data																       	   REFEREE_SYSTEM_ENABLE
#define Result_data			    														    	     REFEREE_SYSTEM_ENABLE
#define Match_robot_survival_data 											           REFEREE_SYSTEM_DISABLE
#define Site_event_data 																			  	 REFEREE_SYSTEM_ENABLE
#define Site_supply_station_action_identification_data 				  	 REFEREE_SYSTEM_DISABLE
#define Robot_state_data																			  	 REFEREE_SYSTEM_ENABLE
#define Real_time_power_and_heat_data													   	 REFEREE_SYSTEM_ENABLE
#define Robot_position_data																			   REFEREE_SYSTEM_ENABLE
#define Robot_gain_data																				  	 REFEREE_SYSTEM_ENABLE
#define Air_robot_energy_status_data													  	 REFEREE_SYSTEM_ENABLE
#define Damage_status_data																			   REFEREE_SYSTEM_ENABLE
#define Real_time_shooting_data																	   REFEREE_SYSTEM_ENABLE

#define CMD_code_Match_status_data															   ((uint16_t)0x0001)
#define CMD_code_Result_data			    													   ((uint16_t)0x0002)
#define CMD_code_Match_robot_survival_data 										 	   ((uint16_t)0x0003)
#define CMD_code_Site_event_data 																   ((uint16_t)0x0101)
#define CMD_code_Site_supply_station_action_identification_data	   ((uint16_t)0x0102)
#define CMD_code_Robot_state_data														 		   ((uint16_t)0x0201)
#define CMD_code_Real_time_power_and_heat_data								 	   ((uint16_t)0x0202)
#define CMD_code_Robot_position_data													 	   ((uint16_t)0x0203)
#define CMD_code_Robot_gain_data															 	   ((uint16_t)0x0204)
#define CMD_code_Air_robot_energy_status_data										   ((uint16_t)0x0205)
#define CMD_code_Damage_status_data													 		   ((uint16_t)0x0206)
#define CMD_code_Real_time_shooting_data											 	   ((uint16_t)0x0207)
#define CMD_code_Robots_interact_with_data											 	 ((uint16_t)0x0301)


                                                                   //包头+命令字+数据帧长度+包尾
#define Data_length_Match_status_data														   ((uint8_t)12)  //5+2+3+2
#define Data_length_Result_data			    													 ((uint8_t)10)  //5+2+1+2
#define Data_length_Match_robot_survival_data 									   ((uint8_t)11)  //5+2+2+2
#define Data_length_Site_event_data 															 ((uint8_t)13)  //5+2+4+2
#define Data_length_Site_supply_station_action_identification_data ((uint8_t)12)  //5+2+3+2
#define Data_length_Robot_state_data															 ((uint8_t)24)  //5+2+15+2 *
#define Data_length_Real_time_power_and_heat_data		    					 ((uint8_t)23)  //5+2+14+2 *
#define Data_length_Robot_position_data													   ((uint8_t)25)  //5+2+16+2 *
#define Data_length_Robot_gain_data															   ((uint8_t)10)  //5+2+1+2
#define Data_length_Air_robot_energy_status_data									 ((uint8_t)12)  //5+2+3+2
#define Data_length_Damage_status_data													   ((uint8_t)10)  //5+2+1+2
#define Data_length_Real_time_shooting_data												 ((uint8_t)15)  //5+2+6+2
#define Data_length_Client_custom_robot_data								       ((uint8_t)28)  //5+2+19+2


#define Client_LED1																								 1
#define Client_LED2																								 2
#define Client_LED3																								 3
#define Client_LED4																								 4
#define Client_LED5																								 5
#define Client_LED6																								 6

typedef enum
{
	CRC_DISABLE = 0,
	CRC_ENABLE,
}CRC_StatusTypeDef;

typedef enum
{
	OK = 0,
	CRC_ERROR,
	CMD_ERROR,
	Error,
}referee_system_StatusTypeDef;

typedef union                         //共用体
{
	uint8_t judge_buff[4];
	float F;
	int I;
}standardize;          //用于临时存储judge_buff的信息 一次性存储四个，利用共用体的性质通过F或I来计算数据，控制步兵车


//注意：不添加__packed系统默认4字节对齐，不足以4字节补齐 加了之后按类型长度对齐
//比赛状态数据 ：0x0001    发送频率：1HZ
typedef __packed struct
{
	uint8_t game_type : 4;    //比赛类型 0-3bit 1:RM对抗赛 2:单项赛 3:RMICRA
	uint8_t game_progress : 4;//比赛阶段 4-7bit 0:未开始比赛 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中 5:比赛结束
  uint16_t stage_remain_time; //当前阶段剩余时间(S)
} ext_game_state_t;

//比赛结果数据 ：0x0002   发送频率：比赛结束后发送
typedef __packed struct
{
uint8_t winner;        //0：平局 1：红方胜利 2：蓝方胜利
} ext_game_result_t;

//机器人存活数据：0x0003  发送频率：1HZ
typedef __packed struct
{
uint16_t robot_legion;    //共16bit 相应位置表示存活
uint8_t survivor_flag;    //1代表存活 0代表死亡
uint32_t survivor_count;
uint32_t last_survivor_count;
} ext_game_robot_survivors_t;

//场地事件数据：0x0101 发送频率：事件改变后发送  
typedef __packed struct
{
	uint32_t event_type;//己方 停机坪 补给站 能量机关 关口 碉堡 资源岛 基地防御相关事件
} ext_event_data_t;

//补给站动作标识：0x0102 发送频率：动作改变后发送  有用
typedef __packed struct
{
  uint8_t supply_projectile_id;    //补给站口ID 
	uint8_t supply_robot_id;         //预约机器人ID
  uint8_t supply_projectile_step;  //子弹口开闭状态 0：关闭 1：准备 2：子弹下落
} ext_supply_projectile_action_t;

//补给站预约子弹：0x0103  发送频率：上限10HZ   RM对抗赛未开放
typedef __packed struct
{
uint8_t supply_projectile_id;        //预约补给站ID
uint8_t supply_num;                  //预约子弹数目 0-50（50）   51-100（100） 101-150（150） 151-255（200）
} ext_supply_projectile_booking_t;

//比赛机器人状态：0x0201  发送频率：10HZ  有用
typedef __packed struct
{
uint8_t robot_id;                   //机器人ID
uint8_t robot_level;                //机器人等级
uint16_t remain_HP;                 //剩余血量
uint16_t max_HP;                    //满血量
uint16_t shooter_heat0_cooling_rate;//17mm 冷却速度 J/s
uint16_t shooter_heat0_cooling_limit;//17mm 子弹热量上限
uint16_t shooter_heat1_cooling_rate; //42mm
uint16_t shooter_heat1_cooling_limit;
uint8_t power_out;
uint32_t state_count;
uint32_t last_state_count;
uint8_t mains_power_gimbal_output  : 1;  //gimbal电源输出 1->24V开启
uint8_t mains_power_chassis_output : 1;   //chassis
uint8_t mains_power_shooter_output : 1;   //shooter
} ext_game_robot_state_t;

//实时热量数据 0x0202 发送频率：50HZ 有用
typedef __packed struct
{
float chassis_volt;           //底盘输出电压 mV  发送的数据是uint16_t
float chassis_current;        //底盘输出电流 mA    发送的数据是uint16_t
float chassis_power;             //底盘输出功率 W
uint16_t chassis_power_buffer;   //底盘功率缓冲 J
uint16_t shooter_heat0;          //17mm 枪口热量
uint16_t shooter_heat1;          //40mm 枪口热量
uint32_t heat_count;             //自定义计数变量 用来控制弹丸的发射
uint32_t heat_count_last;
uint32_t shoot_bullet_number;   //当前可发射弹丸数
} ext_power_heat_data_t;

//机器人位置：0x0203 发送频率：10HZ
typedef __packed struct
{
float x;             //m
float y;
float z;
float yaw;           //位置枪口 度
} ext_game_robot_pos_t;

//机器人增益：0x0204 相关状态  发送频率：状态改变后发送
typedef __packed struct
{
uint8_t power_rune_buff; //0 1 2 3：血量补血 热量冷却加速  防御加成 攻击加成
}ext_buff_musk_t;

//空中机器人能量状态 ：0x0205  发送频率：10HZ
typedef __packed struct
{
uint8_t energy_point;    //积累的能量点
uint8_t attack_time;     //可攻击的时间 s 50s递减至零
} aerial_robot_energy_t;

//伤害状态：0x0206 发送频率：伤害发生后发送  有用
typedef __packed struct
{
uint8_t armor_id : 4;   //0-3受伤装甲ID 其他血量变化类型 该变量数值为0
uint8_t hurt_type : 4;  //0 1 2 3：装甲伤害扣血 模块掉线扣血 超热量扣血 超功率扣血
} ext_robot_hurt_t;

//实时射击信息：0x0207 发送频率:射击后发送  有用
typedef __packed struct
{
uint8_t bullet_type;    //子弹类型
uint8_t bullet_freq;    //子弹射频 HZ
float bullet_speed;     //子弹射速 m/s
u32 bullet_count;       //射击计数
u16 remain_bullet_num;  //剩余弹丸数
} ext_shoot_data_t;

//交互数据接受信息：0x0301 发送频率：上限10HZ
typedef __packed struct
{
	uint16_t data_cmd_id;      //内容ID 0xD180：客户端自定义数据   0x0200-0x02ff：己方机器人间通信
uint16_t send_ID;          //发送者ID
uint16_t receiver_ID;      //接受者ID
}ext_student_interactive_header_data_t;

//客户端自定义数据  发送频率：上限10HZ
typedef __packed struct
{
float data1;     //浮点型数据
float data2;     //浮点型数据
float data3;     //浮点型数据
uint8_t masks;   //自定义8位数据
}client_custom_data_t;

extern u8 stdbuf[30];            //配合Append_CRC16_Check_Sum()使用 用于生成信息码+校验码 从而向裁判系统发送数据
extern u8 judge_buff[judgelen];     //裁判系统接收缓存数组
extern u16 max_heat;              //最大热量

extern ext_game_robot_survivors_t  ext_game_robot_survivors; //机器人存活状态信息
extern ext_game_robot_state_t ext_game_robot_state;       //机器人状态
extern ext_power_heat_data_t ext_power_heat_data;         //实时功率 热量
extern ext_robot_hurt_t ext_robot_hurt;                   //伤害情况
extern ext_shoot_data_t ext_shoot_data;                   //射击信息
extern ext_student_interactive_header_data_t ext_student_interactive_header_data;
extern client_custom_data_t client_custom_data;
extern u8 gimbal_reset;
void  Real_time_power_and_heat_data_processing(uint8_t *Msg);
void Game_state_data_processing(uint8_t *Msg);
void Real_time_Shoot_data_processing(uint8_t *Msg);
void Robot_hurt_status_data_processing(uint8_t *Msg);
void Client_custom_robot_data_Send(void);
void Robot_survivors_data_processing(uint8_t *Msg);
referee_system_StatusTypeDef referee_system_Rx(uint8_t * RxMsg, CRC_StatusTypeDef CRCSwitch);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
