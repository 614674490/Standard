#include "usart_imu.h"
#include "string.h"
#include "include.h"

int yaw_usart_count = 0;
volatile  float pitch_usart_angle=0;
volatile  float yaw_usart_angle=0;
volatile  float yaw_usart_temp=0;
volatile  float last_yaw_usart_temp=0;

static  float gyro_y=0;
static  float gyro_z=0;

float gyro_usart_y=0;
float gyro_usart_z=0;

int data_index=0;

const u8 TEXT_Buffer[]={"AT+RST"};
#define SIZE sizeof(TEXT_Buffer);
//����������ʱ����˴��ڲ������Ƿ��Ϊ460800,�Լ���main.c���HAL_NVIC_EnableIRQ(UART8_IRQn)������
void Usart_Imu_Data_Process(const u8 *ucRxBuffer)
{
	/*****************������ͷ*******************/
			if(ucRxBuffer[0]==0x5A)
			{
					data_index=0;
			}
			else if(ucRxBuffer[data_index]!=0x5A)
			{		
					data_index++;
					if(data_index>usart_mpu_buflen)data_index=0;
			}
	/*********************���ݴ���***************/
			if(ucRxBuffer[data_index+6]==0xB0)  //���ٶ�
			{
				gyro_y=-((int16_t)(ucRxBuffer[data_index+8]<<8|ucRxBuffer[data_index+7]))/10.0f;//P��
				gyro_z=((int16_t)(ucRxBuffer[data_index+12]<<8|ucRxBuffer[data_index+11]))/10.0f;//Y��
				gyro_usart_y=gyro_y;
				gyro_usart_z=gyro_z;		
			}
			if(ucRxBuffer[data_index+13]==0xD0)  //�Ƕ�
			{
				pitch_usart_angle=((int16_t)(ucRxBuffer[data_index+17]<<8|ucRxBuffer[data_index+16]))/100.0f;
				yaw_usart_temp=((int16_t)(ucRxBuffer[data_index+19]<<8|ucRxBuffer[data_index+18]))/10.0f;
				if(yaw_usart_temp-last_yaw_usart_temp>=300)  //yaw��ǶȾ����������������
				{
						yaw_usart_count--;
				}
				else if (yaw_usart_temp-last_yaw_usart_temp<=-300)
				{
						yaw_usart_count++;
				}
				yaw_usart_angle = -(yaw_usart_temp + yaw_usart_count*360);  //roll��Ƕ�
				last_yaw_usart_temp = yaw_usart_temp;
			}
}


//����һ���ֽ�
void USART8_SendByte(u8 Data)
{
   //�ȴ��������ݼĴ�����
   while (!(huart8.Instance->SR & 0X0080));
   //��������
   huart8.Instance->DR = (Data & 0x00FF);
   //�ȴ��������
   while (!(huart8.Instance->SR & 0X0040));   
}

void USART_IMU_Init(u8 *s)
{
	 while (*s)
	 {
		 USART8_SendByte(*s);
		 (*s)++;
	 }
}
