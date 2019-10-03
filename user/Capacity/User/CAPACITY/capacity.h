#ifndef _capacity_h
#define _capacity_h
#include "rtos.h"

#define CHARGE_START()	 HAL_GPIO_WritePin(CHARGE_GPIO_Port,CHARGE_Pin,GPIO_PIN_RESET)   //��ʼ���
#define CHARGE_STOP()    HAL_GPIO_WritePin(CHARGE_GPIO_Port,CHARGE_Pin,GPIO_PIN_SET)
//#define KEY_VALUE      HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)
//#define LED_ON()	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)  //PC13
//#define LED_OFF()	   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
//#define LED_Toggle() HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)
#define CAPACITY_START() HAL_GPIO_WritePin(GPIOC,SWITCH_Pin,GPIO_PIN_SET)	//���ݸ����̹��� 
#define POWER_START()    HAL_GPIO_WritePin(GPIOC,SWITCH_Pin,GPIO_PIN_RESET)//��Դ�����̹���


extern float CAP_Power;          //����PID�ó��ĵ��ݳ�繦��
extern float charge_ma;          //������
extern uint16_t charge_PWM;           //PWM->current
extern uint8_t charge_flag;           //����־λ
extern int16_t Chassis_Power;
extern int16_t Chassis_Power_Last; //
extern float CAP_Tar_Power;        //����������繦��
extern float CAP_Now_Power;        //��ǰ���ݳ�繦��
extern float vel;
int fputc(int ch ,FILE *f);  //����1��ӡ����

void Capacity_Charge_Power(void);
void Charge_Current(void);
void Power_Switch(void);
#endif

