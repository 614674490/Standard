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
	
//键盘宏定义
#define KEY_V		0x4000//自旋
#define KEY_C		0x2000//单三连发切换
#define KEY_X		0x1000//巡逻
#define KEY_Z		0x0800//转向（无小陀螺不可用）
#define KEY_G		0x0400//
#define KEY_F		0x0200//开舱
#define KEY_R		0x0100//自瞄
#define KEY_E		0x0080//扭腰
#define KEY_Q		0x0040//连发
#define KEY_CTRL	0x0020//减速
#define KEY_SHIFT	0x0010//加速
#define KEY_D		0x0008//右
#define KEY_A		0x0004//左
#define KEY_S		0x0002//后
#define KEY_W		0x0001//前
#define KEY_B   0x8000//
//鼠标限幅处理
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

//遥控器死区
#define RC_deadband 10

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
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
#define FRICTION_WHEEL_MAX_DUTY             1600     //配合上面的宏定义使用

//mouse control parameters  由鼠标坐标装换为Y P轴角度的系数
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.06f
#define MOUSE_TO_YAW_ANGLE_INC_FACT 	   	0.08f

typedef __packed struct
{
	int16_t ch0;        //前后轮
	int16_t ch1;        //左右轮
	int16_t ch2;        //Y轴
	int16_t ch3;        //P轴
	int8_t s1;          //摩擦轮
	int8_t s2;           //模式
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
	u16 press_l_times;   //鼠标左键按下计数
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
}RC_Ctl_t;             //遥控器参数

//输入模式选择  枚举类型  是逗号 不是分号
typedef enum
{
	REMOTE_INPUT=1,
	KEY_MOUSE_INPUT=3,
	STOP=2,
	OFFLINE=4,
}InputMode_e;

//摩擦轮状态 枚举类型
typedef enum
{
	FRICTION_WHEEL_OFF=0,    //关闭
	FRICTION_WHEEL_START_TURNNING=1,    //摩擦轮启动
	FRICTION_WHEEL_ON=2,    //摩擦轮正在运行
}FrictionWheelState_e;

typedef struct
{
	volatile float pitch_angle_dynamic_ref;   //实时角度
	volatile float yaw_angle_dynamic_ref;
	float pitch_speed_ref;
	float yaw_speed_ref;
	float yaw_position_ref;
}Gimbal_Ref_t;           //云台参数


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
	u8 shoot_mode;   //为真单发 为假三连发  2一直发
	u8 no_twisted;
	u8 last_no_twisted;
	
	u8 last_val;
	u8 shoot_flag;
	u8 q_flag;
	volatile u8 save_flag;
	u8 shift_flag;
	u8 ctrl_flag;
  u8 e_times;          //按键e按下次数
	u8 e_flag;       //e键按下标志位
	u8 w_flag;
	u8 s_flag;
	u8 a_flag;
	u8 d_flag;
	u8 f_flag;
	u8 c_flag;
	u8 z_flag;
	u8 r_flag;
	u8 x_flag;
	u8 rotate_to_middle; //0扭腰  1自旋
	u8 last_rotate;
	u8 rotate;
	u8 shoot_flag_back_flag;
	u8 con_shoot_flag;  //0不连发 1连发
	u8 autoaim_mode;    //自瞄标志位
	u8 FB_Press_flag;    //上次是否按下标志位   w s
	u8 LR_Press_flag;    //上次是否按下标志位   a d
	
	
}OPERATE_MESSAGE;
typedef struct
{
	u8 Shooting_Flag;    //1:可射击子弹
		u32 LastcountTime;  //上次热量统计时间
	  u8 Shooting_delay;   //射击延时
   float remain_heat;   //实时热量
   float limit_heat; 	  //热量限制
	 float cooling_heat;	//热量冷却值
} ShootingFreCon_t;   //发射子弹的热量计算参数结构体

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

extern int q_count , e_count ;   //连发 扭腰（底盘动 云台不动） 的计数变量
extern u8 ws_flag,ad_flag;      //w/s按下标志位 a/d按下标志位

extern InputMode_e inputmode;
extern u8 friction_flag;

extern float changehead_rorate,changehead_y;

void SetInputMode(Remote *rc);   //输入模式设置
InputMode_e GetInputMode(void);   //函数封装
void RemoteControlProcess(Remote *rc);//遥控器控制模式处理
void RemoteShootControl(uint8_t val);   //遥控器摩擦轮
void MouseKeyControlProcess(Mouse *mouse, Key *key);   //键盘控制函数
void MouseShootControl(Mouse *mouse);   //鼠标射击
void WalkControl(ramp_t ramp);    //行走控制
void cartridge(void);
void Shoot_Control(u8 shoot_mode);
void Remote_Offline_Slove(void);
void Remote_Data_Process(const u8  *Msg);

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */


