#ifndef _RTOS_H_
#define _RTOS_H_

#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "cmsis_os.h"
#include "tim.h"
#include "ARM_math.h"
#include "pid.h"
#include "capacity.h"

extern uint8_t SupperCapacitorSwitch;


void start(void);

extern int16_t Chassis_Motor_Actual_Speed_1;
extern int16_t Chassis_Motor_Actual_Speed_2;
extern int16_t Chassis_Motor_Actual_Speed_3;
extern int16_t Chassis_Motor_Actual_Speed_4;
extern TaskHandle_t control_Handler;
void control_task(void *pvParameters);



extern float Capacitor_voltage;
extern float Charging_current;
extern float Total_current;
extern float Charging_power;
extern float Total_power;
extern uint32_t Speed;

extern TaskHandle_t ADC_Handler;
void ADC_task(void *pvParameters);

#endif
