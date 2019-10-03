#ifndef _fuzzy_pid_h
#define _fuzzy_pid_h
#include "sys.h"
/* ---------------------------  模糊PID参数及函数 --------------------------------- */
#define ID_dKp 1
#define ID_dKi 2
#define ID_dKd 3
//Ge Gec:量化因子 Gkp GKi GKd:比例因子
static const float levelInterval = 5;// 用于等价扩大论域的范围 论域的分级间隔必须相同，此处设置为5
static const float Ge =  1.0f;        
static const float Gec = 1.0f;           //限制Kp Ki Kd 的输出范围
static const float Gkp = 10.0f*levelInterval; //3->3/10=0.3                  0.3    
static const float Gki = 100.0f*levelInterval;//3->3/100=0.03            0.03
static const float Gkd = 10.0f*levelInterval;//3->3/10=0.3        比例因子         0.3

//语言变量的论域划分  量化等级：7 论域：[-3 3]*levelInterval
static const float NB= -3 * levelInterval;
static const float NM= -2 * levelInterval;
static const float NS= -1 * levelInterval;
static const float ZE= 0;
static const float PS= 1 * levelInterval;
static const float PM= 2 * levelInterval;
static const float PB= 3 * levelInterval;

static const float fuzzyRuleKp[7][7]={
 
PB,PB,PM,PM,PS,PS,ZE,
PB,PB,PM,PM,PS,ZE,ZE,
PM,PM,PM,PS,ZE,NS,NM,
PM,PS,PS,ZE,NS,NM,NM,
PS,PS,ZE,NS,NS,NM,NM,
ZE,ZE,NS,NM,NM,NM,NB,
ZE,NS,NS,NM,NM,NB,NB
 
};//dKp模糊规则表
 
static const float fuzzyRuleKi[7][7]={
 
NB,NB,NB,NM,NM,ZE,ZE,
NB,NB,NM,NM,NS,ZE,ZE,
NM,NM,NS,NS,ZE,PS,PS,
NM,NS,NS,ZE,PS,PS,PM,
NS,NS,ZE,PS,PS,PM,PM,
ZE,ZE,PS,PM,PM,PB,PB,
ZE,ZE,PS,PM,PB,PB,PB
 
};//dKi模糊规则表
 
static const float fuzzyRuleKd[7][7]={
 
PS,PS,ZE,ZE,ZE,PB,PB,
NS,NS,NS,NS,ZE,NS,PM,
NB,NB,NM,NS,ZE,PS,PM,
NB,NM,NM,NS,ZE,PS,PM,
NB,NM,NS,NS,ZE,PS,PS,
NM,NS,NS,NS,ZE,PS,PS,
PS,ZE,ZE,ZE,ZE,PB,PB
 
};//dKd模糊规则表

struct Factor
{
	float Ke;            //确定 E EC Kp Ki Kd的论域
	float Kec;
	float Ku_Kp;
	float Ku_Ki;
	float Ku_Kd;
};

typedef struct 
{
 
		float Kp;
		float Ki;
		float Kd;
		float Fuzzy_Kp;
		float Fuzzy_Ki;
		float Fuzzy_Kd;
	  float sum_e;
	  float last_e;
	  float e;
	  float ec;
	  struct Factor factor;
	  int pid_out;
 
}fPID;               //模糊PID Kp Ki Kd参数保存 位置式


typedef struct 
{
 
		float Kp;
		float Ki;
		float Kd;
		float Fuzzy_Kp;
		float Fuzzy_Ki;
		float Fuzzy_Kd;
	  float per_e;     //前次
	  float last_e;    //上次
	  float e;
	  float ec;
		struct Factor factor;
	  int pid_out;
 
}fPID_Inc;               //模糊PID Kp Ki Kd参数保存 增量式
void FPID_Init(void);
void Fuzzy(fPID *motor);
void Fuzzy_Inc(fPID_Inc *motor);
void Fuzzy_PidControler(float tar, float cur,fPID *motor);
float DeFuzzy(int eLevel,int ecLevel,u8 ID_item,struct Factor *factor);
int FuzzyPID_Inc(float tar,float cur,fPID_Inc *motor);
//extern fPID Chassis_Motor_FPID_1,Chassis_Motor_FPID_2,Chassis_Motor_FPID_3,Chassis_Motor_FPID_4,Chassis_Motor_FPID_rotate;
extern fPID PTZ_Motor_FPID_Position_Y,PTZ_Motor_FPID_Speed_Y,PTZ_Motor_FPID_Position_P,PTZ_Motor_FPID_Speed_P;
//extern fPID_Inc FPower_limit;

/* ------------------------------------------------------------------------------ */

#endif
