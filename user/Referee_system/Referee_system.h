#ifndef __REFEREE_SYSTEM_H
#define __REFEREE_SYSTEM_H
/*����ϵͳ��һЩ�������*/
#include "stm32f4xx_hal.h"
#include "sys.h"

#define judgelen                                                   30            //����ϵͳ���ջ����������󳤶�


#define referee_system_QueueLength                                 5   //������Ŀ����
#define referee_system_ItemSize		                                 judgelen
#define threshold                                                  (referee_system_QueueLength-2)

#define REFEREE_SYSTEM_ENABLE                           			  	 1
#define REFEREE_SYSTEM_DISABLE                          				   0

#define Zero_Level            																  	 0     //û�еȼ�
#define One_Level             																  	 1    //һ��
#define Two_Level             																  	 2    //����
#define Three_Level           																  	 3    //����

#define LimitHeat_Lev1       																	  	 240    //һ����������
#define LimitHeat_Lev2       																	  	 360     //������������
#define LimitHeat_Lev3      																	  	 480     //������������

#define HeatCollingSpeed_Lev1                                      4.0f
#define HeatCollingSpeed_Lev2                                      6.0f
#define HeatCollingSpeed_Lev3                                      8.0f

#define JudgeFrameHeader                                           0xA5   //����ϵͳ���ݵ�֡ͷ

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


                                                                   //��ͷ+������+����֡����+��β
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

typedef union                         //������
{
	uint8_t judge_buff[4];
	float F;
	int I;
}standardize;          //������ʱ�洢judge_buff����Ϣ һ���Դ洢�ĸ������ù����������ͨ��F��I���������ݣ����Ʋ�����


//ע�⣺�����__packedϵͳĬ��4�ֽڶ��룬������4�ֽڲ��� ����֮�����ͳ��ȶ���
//����״̬���� ��0x0001    ����Ƶ�ʣ�1HZ
typedef __packed struct
{
	uint8_t game_type : 4;    //�������� 0-3bit 1:RM�Կ��� 2:������ 3:RMICRA
	uint8_t game_progress : 4;//�����׶� 4-7bit 0:δ��ʼ���� 1:׼���׶� 2:�Լ�׶� 3:5s����ʱ 4:��ս�� 5:��������
  uint16_t stage_remain_time; //��ǰ�׶�ʣ��ʱ��(S)
} ext_game_state_t;

//����������� ��0x0002   ����Ƶ�ʣ�������������
typedef __packed struct
{
uint8_t winner;        //0��ƽ�� 1���췽ʤ�� 2������ʤ��
} ext_game_result_t;

//�����˴�����ݣ�0x0003  ����Ƶ�ʣ�1HZ
typedef __packed struct
{
uint16_t robot_legion;    //��16bit ��Ӧλ�ñ�ʾ���
uint8_t survivor_flag;    //1������ 0��������
uint32_t survivor_count;
uint32_t last_survivor_count;
} ext_game_robot_survivors_t;

//�����¼����ݣ�0x0101 ����Ƶ�ʣ��¼��ı����  
typedef __packed struct
{
	uint32_t event_type;//���� ͣ��ƺ ����վ �������� �ؿ� �ﱤ ��Դ�� ���ط�������¼�
} ext_event_data_t;

//����վ������ʶ��0x0102 ����Ƶ�ʣ������ı����  ����
typedef __packed struct
{
  uint8_t supply_projectile_id;    //����վ��ID 
	uint8_t supply_robot_id;         //ԤԼ������ID
  uint8_t supply_projectile_step;  //�ӵ��ڿ���״̬ 0���ر� 1��׼�� 2���ӵ�����
} ext_supply_projectile_action_t;

//����վԤԼ�ӵ���0x0103  ����Ƶ�ʣ�����10HZ   RM�Կ���δ����
typedef __packed struct
{
uint8_t supply_projectile_id;        //ԤԼ����վID
uint8_t supply_num;                  //ԤԼ�ӵ���Ŀ 0-50��50��   51-100��100�� 101-150��150�� 151-255��200��
} ext_supply_projectile_booking_t;

//����������״̬��0x0201  ����Ƶ�ʣ�10HZ  ����
typedef __packed struct
{
uint8_t robot_id;                   //������ID
uint8_t robot_level;                //�����˵ȼ�
uint16_t remain_HP;                 //ʣ��Ѫ��
uint16_t max_HP;                    //��Ѫ��
uint16_t shooter_heat0_cooling_rate;//17mm ��ȴ�ٶ� J/s
uint16_t shooter_heat0_cooling_limit;//17mm �ӵ���������
uint16_t shooter_heat1_cooling_rate; //42mm
uint16_t shooter_heat1_cooling_limit;
uint8_t power_out;
uint32_t state_count;
uint32_t last_state_count;
uint8_t mains_power_gimbal_output  : 1;  //gimbal��Դ��� 1->24V����
uint8_t mains_power_chassis_output : 1;   //chassis
uint8_t mains_power_shooter_output : 1;   //shooter
} ext_game_robot_state_t;

//ʵʱ�������� 0x0202 ����Ƶ�ʣ�50HZ ����
typedef __packed struct
{
float chassis_volt;           //���������ѹ mV  ���͵�������uint16_t
float chassis_current;        //����������� mA    ���͵�������uint16_t
float chassis_power;             //����������� W
uint16_t chassis_power_buffer;   //���̹��ʻ��� J
uint16_t shooter_heat0;          //17mm ǹ������
uint16_t shooter_heat1;          //40mm ǹ������
uint32_t heat_count;             //�Զ���������� �������Ƶ���ķ���
uint32_t heat_count_last;
uint32_t shoot_bullet_number;   //��ǰ�ɷ��䵯����
} ext_power_heat_data_t;

//������λ�ã�0x0203 ����Ƶ�ʣ�10HZ
typedef __packed struct
{
float x;             //m
float y;
float z;
float yaw;           //λ��ǹ�� ��
} ext_game_robot_pos_t;

//���������棺0x0204 ���״̬  ����Ƶ�ʣ�״̬�ı����
typedef __packed struct
{
uint8_t power_rune_buff; //0 1 2 3��Ѫ����Ѫ ������ȴ����  �����ӳ� �����ӳ�
}ext_buff_musk_t;

//���л���������״̬ ��0x0205  ����Ƶ�ʣ�10HZ
typedef __packed struct
{
uint8_t energy_point;    //���۵�������
uint8_t attack_time;     //�ɹ�����ʱ�� s 50s�ݼ�����
} aerial_robot_energy_t;

//�˺�״̬��0x0206 ����Ƶ�ʣ��˺���������  ����
typedef __packed struct
{
uint8_t armor_id : 4;   //0-3����װ��ID ����Ѫ���仯���� �ñ�����ֵΪ0
uint8_t hurt_type : 4;  //0 1 2 3��װ���˺���Ѫ ģ����߿�Ѫ ��������Ѫ �����ʿ�Ѫ
} ext_robot_hurt_t;

//ʵʱ�����Ϣ��0x0207 ����Ƶ��:�������  ����
typedef __packed struct
{
uint8_t bullet_type;    //�ӵ�����
uint8_t bullet_freq;    //�ӵ���Ƶ HZ
float bullet_speed;     //�ӵ����� m/s
u32 bullet_count;       //�������
u16 remain_bullet_num;  //ʣ�൯����
} ext_shoot_data_t;

//�������ݽ�����Ϣ��0x0301 ����Ƶ�ʣ�����10HZ
typedef __packed struct
{
	uint16_t data_cmd_id;      //����ID 0xD180���ͻ����Զ�������   0x0200-0x02ff�����������˼�ͨ��
uint16_t send_ID;          //������ID
uint16_t receiver_ID;      //������ID
}ext_student_interactive_header_data_t;

//�ͻ����Զ�������  ����Ƶ�ʣ�����10HZ
typedef __packed struct
{
float data1;     //����������
float data2;     //����������
float data3;     //����������
uint8_t masks;   //�Զ���8λ����
}client_custom_data_t;

extern u8 stdbuf[30];            //���Append_CRC16_Check_Sum()ʹ�� ����������Ϣ��+У���� �Ӷ������ϵͳ��������
extern u8 judge_buff[judgelen];     //����ϵͳ���ջ�������
extern u16 max_heat;              //�������

extern ext_game_robot_survivors_t  ext_game_robot_survivors; //�����˴��״̬��Ϣ
extern ext_game_robot_state_t ext_game_robot_state;       //������״̬
extern ext_power_heat_data_t ext_power_heat_data;         //ʵʱ���� ����
extern ext_robot_hurt_t ext_robot_hurt;                   //�˺����
extern ext_shoot_data_t ext_shoot_data;                   //�����Ϣ
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
