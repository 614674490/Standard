#ifndef _detect_h
#define _detect_h
#include "sys.h"

//#define ONLINE    0
//#define OFFLINE   1
#define RCCALI_BUZZER_CYCLE_TIME 4000  //校准选择时间20s，蜂鸣器断续发声周期时间
#define RC_CALI_BUZZER_PAUSE_TIME 2000 //校准选择时间20s，蜂鸣器断续发声停声时间

//错误码以及对应设备顺序
enum errorList
{
	  RemoteTOE=0 ,      //RED
	
    ChassisMotor1TOE,    //A
	  ChassisMotor2TOE,    //B
	  ChassisMotor3TOE,    //C
	  ChassisMotor4TOE,    //D
	 
	  YawGimbalMotorTOE,   //E
	  PitchGimbalMotorTOE, //F
	  HITTOE,              //H
	
	  
	 	MinPCTOE,           //G
	
    JudgeTOE,            //GREEN
	
    errorListLength,
};

typedef __packed struct
{
    u32 newTime;
    u32 lastTime;
    u32 Losttime;
    u32 worktime;
    u32 setOfflineTime : 12;
    u32 setOnlineTime : 12;
    u8 enable : 1;
    u8 isLost : 1;
	
    void (*solveOfflineFun)(void);

} error_t;


extern  error_t errorList[errorListLength + 1];
extern uint8_t rotate_revive_flag;
void DetectHook(uint8_t toe);
void DetectInit(void);
void DetectDisplay(void);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void Init_Buzzer_Tip(void);
int Wait_Motor_Power_On(void);
void struct_clear(void);
#endif

