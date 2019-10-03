#ifndef _app_h
#define _app_h

/**
 * GPIO ��˵��
 *USART 1 -> DBUS        ң����
 *USART 2 -> DEBUG       ��ӡ�ӿ�
 *USART 3 -> NULL        ��
 *USART 6 -> JUDGE       ����ϵͳ
 *USART 7 -> MinPc       PC
 *USART 8 -> Capacity    ����
 *ADC1    -> PF11        �����ɼ�

 *TIM4 1 2-> PD12 PD13   Ħ����
 *TIM8 1  -> PI5         ���
 *TIM12 1 -> BUZZY       ������
 */
#include "stm32f4xx_HAL.h"

#include "gpio.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "detect.h"
#define LED_Judge_On()        HAL_GPIO_WritePin(Judge_Led_GPIO_Port, Judge_Led_Pin, GPIO_PIN_RESET)
#define LED_Judge_Off()       HAL_GPIO_WritePin(Judge_Led_GPIO_Port, Judge_Led_Pin, GPIO_PIN_SET)
#define LED_Judge_Toggle()   if(soft_js_time%1000==0) HAL_GPIO_TogglePin(Judge_Led_GPIO_Port, Judge_Led_Pin)

#define LED_Remote_On()      HAL_GPIO_WritePin(STOP_Led_GPIO_Port, STOP_Led_Pin, GPIO_PIN_RESET)
#define LED_Remote_Off()     HAL_GPIO_WritePin(STOP_Led_GPIO_Port, STOP_Led_Pin, GPIO_PIN_SET)
#define LED_Remote_Toggle() if(soft_js_time%1000==0) HAL_GPIO_TogglePin(STOP_Led_GPIO_Port, STOP_Led_Pin)




#define FLOW_LED_ON(num)       HAL_GPIO_WritePin(GPIOG, ((uint16_t)0x0001) << num,GPIO_PIN_RESET);
#define FLOW_LED_OFF(num)      HAL_GPIO_WritePin(GPIOG, ((uint16_t)0x0001) << num,GPIO_PIN_SET);
#define FLOW_LED_TOGGLE( num)  HAL_GPIO_TogglePin(GPIOG, ((uint16_t)0x0001) << num)
	
#define LASER_ON()          HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET)  //���� PG13
#define LASER_OFF()         HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET)

#define POWER_ON()          HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET)  //�����ĸ�24V��Դ
#define POWER_OFF()          HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET)  //�ز��ĸ�24V��Դ

#define Butten_Trig_Pin     HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)   //΢�����ؼ��

#define Init_buzzer()       buzzer_on(5, 25000) //��ʼ����������Ƶ���Լ�ǿ��

#define Wait_Power()        while(Wait_Motor_Power_On()){ vTaskDelay(1);}

//�ź��� ������ ����
extern osThreadId GimbalTaskHandle;
extern osThreadId defaultTaskHandle;
extern osThreadId IMUTaskHandle;
extern osThreadId RemoteTaskHandle;
extern osThreadId ChassisTaskHandle;
extern osThreadId JudgeTaskHandle;
extern osThreadId MiniPCTaskHandle;
extern osThreadId CapacityTaskHandle;
extern osThreadId CanTaskHandle;
extern osThreadId BuzzyTaskHandle;
extern osMessageQId JudgeQueueHandle;
extern osMessageQId IMUQueueHandle;
extern int Init_End_Flag;   
void BSP_Init(void);
void Task_Suspend(void);
void Task_Resume(void);

#endif

