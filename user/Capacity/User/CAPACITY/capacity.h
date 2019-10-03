#ifndef _capacity_h
#define _capacity_h
#include "rtos.h"

#define CHARGE_START()	 HAL_GPIO_WritePin(CHARGE_GPIO_Port,CHARGE_Pin,GPIO_PIN_RESET)   //开始充电
#define CHARGE_STOP()    HAL_GPIO_WritePin(CHARGE_GPIO_Port,CHARGE_Pin,GPIO_PIN_SET)
//#define KEY_VALUE      HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)
//#define LED_ON()	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)  //PC13
//#define LED_OFF()	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
//#define LED_Toggle() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define CAPACITY_START() HAL_GPIO_WritePin(GPIOC,SWITCH_Pin,GPIO_PIN_SET)	//电容给底盘供电 
#define POWER_START()    HAL_GPIO_WritePin(GPIOC,SWITCH_Pin,GPIO_PIN_RESET)//电源给底盘供电


extern float CAP_Power;          //经过PID得出的电容充电功率
extern float charge_ma;          //充电电流
extern uint16_t charge_PWM;           //PWM->current
extern uint8_t charge_flag;           //充电标志位
extern int16_t Chassis_Power;
extern int16_t Chassis_Power_Last; //
extern float CAP_Tar_Power;        //电容期望充电功率
extern float CAP_Now_Power;        //当前电容充电功率
extern float vel;
int fputc(int ch ,FILE *f);  //串口1打印数据

void Capacity_Charge_Power(void);
void Charge_Current(void);
void Power_Switch(void);
#endif

