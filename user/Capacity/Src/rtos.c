#include "rtos.h"

uint8_t SupperCapacitorSwitch=0;
uint32_t Speed;
	BaseType_t ss;
void start(void)
{
	
	HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,usart3_dma_buff,30);
	
//	CanFilter_Init(&hcan);
//	HAL_CAN_Receive_IT(&hcan,CAN_FIFO0);
//	CHARGE_STOP();
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	
	
	PID_Init();
	
	xTaskCreate((TaskFunction_t)control_task,
		(const char *)"control_task",
		(uint16_t)128,
		(void *)NULL,
		(UBaseType_t)2,
		(TaskHandle_t *)&control_Handler);

	ss = xTaskCreate((TaskFunction_t)ADC_task,
		(const char *)"ADC_task",
		(uint16_t)256,
		(void *)NULL,
		(UBaseType_t)2,
		(TaskHandle_t *)&ADC_Handler);
}

/*control进程*/
uint16_t charspeed=10;
TaskHandle_t control_Handler;
void control_task(void *pvParameters)
{
	portTickType xLastWakeTime;

	for(;;)
	{
		Capacity_Charge_Power();
		Charge_Current();
		Power_Switch();
		vTaskDelayUntil(&xLastWakeTime, 1);
	}
}

/*ADC进程*/
#define galvanometer_base 2.5f

float Capacitor_voltage = 0;
float Charging_current = 0;
float Total_current = 0;
float Charging_power = 0;
float Total_power = 0;
uint8_t voltage[] = {0x5a,0x00,0x00};
TaskHandle_t ADC_Handler;
void ADC_task(void *pvParameters)
{
	__IO uint16_t Capacitor_voltage_u16 = 0;
	
	uint16_t Capacitor_voltage_buff[10];
//	uint16_t Charging_current_buff[10];
	uint16_t Total_current_buff[10];
	for(;;)
	{
		Capacitor_voltage = Average(Capacitor_voltage_buff,Get_Adc(ADC_CHANNEL_0));
		Capacitor_voltage = _fabsf(Capacitor_voltage/1241.212f)*7.67f;
		vTaskDelay(2);
//		Charging_current = Get_Adc2(ADC_CHANNEL_4);
//		Charging_current = _fabsf(Charging_current*0.8056640625f);//-2500.0f)/100.0f;
//		Charging_power = Charging_current*24.0f;
//		vTaskDelay(2);
		Total_current = Average(Total_current_buff,Get_Adc(ADC_CHANNEL_4));
		Total_current = _fabsf(Total_current/1241.212f-2.35f)/0.1f;
		Total_power = Total_current*24.0f;
		vTaskDelay(2);
		
		if(Capacitor_voltage<6)
		{
			LED1(GPIO_PIN_RESET);LED2(GPIO_PIN_RESET);LED3(GPIO_PIN_RESET);
		}
		else if(Capacitor_voltage<12&&Capacitor_voltage>6)
		{
			LED1(GPIO_PIN_SET);LED2(GPIO_PIN_RESET);LED3(GPIO_PIN_RESET);
		}
		else if(Capacitor_voltage<18&&Capacitor_voltage>12)
		{
			LED1(GPIO_PIN_SET);LED2(GPIO_PIN_SET);LED3(GPIO_PIN_RESET);
		}
		else
		{
			LED1(GPIO_PIN_SET);LED2(GPIO_PIN_SET);LED3(GPIO_PIN_SET);
		}
		
		Capacitor_voltage_u16 = Capacitor_voltage * 1000;
		voltage[1] = (uint8_t)(Capacitor_voltage_u16 >> 8);
		voltage[2] = (uint8_t)(Capacitor_voltage_u16 & 0xff);
		while(HAL_UART_Transmit(&huart3,voltage,3,10)!=HAL_OK);
		
	}
}


