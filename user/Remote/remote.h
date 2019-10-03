#ifndef _remote_h
#define _remote_h
#ifdef _cplusplus



extern 'C'
{
#endif
#include "stm32f4xx_hal.h"
#include "remote.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "string.h"
#include "sys.h"	
#include "ramp.h"	
	
//���̺궨��
#define KEY_V		0x4000//����
#define KEY_C		0x2000//���������л�
#define KEY_X		0x1000//Ѳ��
#define KEY_Z		0x0800//ת����С���ݲ����ã�
#define KEY_G		0x0400//
#define KEY_F		0x0200//����
#define KEY_R		0x0100//����
#define KEY_E		0x0080//Ť��
#define KEY_Q		0x0040//����
#define KEY_CTRL	0x0020//����
#define KEY_SHIFT	0x0010//����
#define KEY_D		0x0008//��
#define KEY_A		0x0004//��
#define KEY_S		0x0002//��
#define KEY_W		0x0001//ǰ
#define KEY_B   0x8000//
//����޷�����
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

//ң��������
#define RC_deadband 10

/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
		
#define REMOTE_CONTROLLER_STICK_OFFSET      1024u 

#define STICK_TO_CHASSIS_SPEED_REF_FACT     10.0f
#define STICK_TO_PITCH_ANGLE_INC_FACT       0.0010f
#define STICK_TO_YAW_ANGLE_INC_FACT         0.0040f  //0.0045
#define FRICTION_WHEEL_MAX_DUTY             1600     //�������ĺ궨��ʹ��

//mouse control parameters  ���������װ��ΪY P��Ƕȵ�ϵ��
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.06f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 	   	0.08f

typedef __packed struct
{
	int16_t ch0;        //ǰ����
	int16_t ch1;        //������
	int16_t ch2;        //Y��
	int16_t ch3;        //P��
	int8_t s1;          //Ħ����
	int8_t s2;           //ģʽ
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
	u16 press_l_times;   //���������¼���
}Mouse;

//e q ctrl shift d a s w
typedef	__packed struct
{
	uint16_t e;
	uint16_t q;
	uint16_t ctrl;
	uint16_t shift;
	uint16_t d;
	uint16_t a;
	uint16_t s;
	uint16_t w;
	uint16_t v;
	
}Key;

typedef	__packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;             //ң��������

//����ģʽѡ��  ö������  �Ƕ��� ���Ƿֺ�
typedef enum
{
	REMOTE_INPUT=1,
	KEY_MOUSE_INPUT=3,
	STOP=2,
	OFFLINE=4,
}InputMode_e;

//Ħ����״̬ ö������
typedef enum
{
	FRICTION_WHEEL_OFF=0,    //�ر�
	FRICTION_WHEEL_START_TURNNING=1,    //Ħ��������
	FRICTION_WHEEL_ON=2,    //Ħ������������
}FrictionWheelState_e;

typedef struct
{
	volatile float pitch_angle_dynamic_ref;   //ʵʱ�Ƕ�
	volatile float yaw_angle_dynamic_ref;
	float pitch_speed_ref;
	float yaw_speed_ref;
	float yaw_position_ref;
}Gimbal_Ref_t;           //��̨����


typedef struct
{
 u16 fricition_speed;
 float one_bullet_heat;
 float three_bullet_heat;
 u16 one_bullet_dis;
 u16 three_bullet_dis;

}REFEREE_LIMIT_MESSAGE;

typedef struct
{
	u8 play_flag;
  u8 FB_flag;
	u8 rl_flag;
	u8 rr_flag;
	u8 shoot_mode;   //Ϊ�浥�� Ϊ��������  2һֱ��
	u8 no_twisted;
	u8 last_no_twisted;
	
	u8 last_val;
	u8 shoot_flag;
	u8 q_flag;
	volatile u8 save_flag;
	u8 shift_flag;
	u8 ctrl_flag;
  u8 e_times;          //����e���´���
	u8 e_flag;       //e�����±�־λ
	u8 w_flag;
	u8 s_flag;
	u8 a_flag;
	u8 d_flag;
	u8 f_flag;
	u8 c_flag;
	u8 z_flag;
	u8 r_flag;
	u8 x_flag;
	u8 rotate_to_middle; //0Ť��  1����
	u8 last_rotate;
	u8 rotate;
	u8 shoot_flag_back_flag;
	u8 con_shoot_flag;  //0������ 1����
	u8 autoaim_mode;    //�����־λ
	u8 FB_Press_flag;    //�ϴ��Ƿ��±�־λ   w s
	u8 LR_Press_flag;    //�ϴ��Ƿ��±�־λ   a d
	
	
}OPERATE_MESSAGE;
typedef struct
{
	u8 Shooting_Flag;    //1:������ӵ�
		u32 LastcountTime;  //�ϴ�����ͳ��ʱ��
	  u8 Shooting_delay;   //�����ʱ
   float remain_heat;   //ʵʱ����
   float limit_heat; 	  //��������
	 float cooling_heat;	//������ȴֵ
} ShootingFreCon_t;   //�����ӵ���������������ṹ��

void Remote_Rx(u8 *RxMsg);
void RemoteControlProcess(Remote *rc);
void RemoteShootControl(uint8_t val);

extern RC_Ctl_t RC_CtrlData;
extern Gimbal_Ref_t GimbalRef;
extern FrictionWheelState_e friction_wheel_state;
extern REFEREE_LIMIT_MESSAGE refree_change_inf;    //

extern ShootingFreCon_t ShootingFreCon;
extern OPERATE_MESSAGE  key_mouse_inf;
extern volatile float angle_offset; 
extern volatile float pitch_angle_offset; 

extern u8 level;

extern int q_count , e_count ;   //���� Ť�������̶� ��̨������ �ļ�������
extern u8 ws_flag,ad_flag;      //w/s���±�־λ a/d���±�־λ

extern InputMode_e inputmode;
extern u8 friction_flag;

extern float changehead_rorate,changehead_y;

void SetInputMode(Remote *rc);   //����ģʽ����
InputMode_e GetInputMode(void);   //������װ
void RemoteControlProcess(Remote *rc);//ң��������ģʽ����
void RemoteShootControl(uint8_t val);   //ң����Ħ����
void MouseKeyControlProcess(Mouse *mouse, Key *key);   //���̿��ƺ���
void MouseShootControl(Mouse *mouse);   //������
void WalkControl(ramp_t ramp);    //���߿���
void cartridge(void);
void Shoot_Control(u8 shoot_mode);
void Remote_Offline_Slove(void);
void Remote_Data_Process(const u8  *Msg);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */


