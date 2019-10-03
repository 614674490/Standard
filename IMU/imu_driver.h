/**
  *@file test_imu.h
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__IMU_H
#define _TEST__IMU_H

#include "stm32f4xx_hal.h"
#include "main.h"

#define MPU6500_NSS_Low() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU6500_NSS_High() HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

/***********************   �����ǲ����޸ġ�   ************************/		 //
//    #define      AXSET                  -39.5f    //���ٶȼ�x�� -39.5 36.5 
//		#define      AYSET                   78   //���ٶȼ�y��     77
//		#define      AZSET                   200    //���ٶȼ�z��  4296-200=4096---�������ٶ�
//		#define      GXSET                   0.5f  //������x��       1
//		#define      GYSET                  -1.5f    //������y��   -1.5f
//		#define      GZSET                   0.5f    //������z��     0


typedef __packed struct
{
	int16_t MaxMagX;
	int16_t MaxMagY;
	int16_t MaxMagZ;
	int16_t MinMagX;
	int16_t MinMagY;
	int16_t MinMagZ;
}MagMaxMinData_t;

typedef struct
{
  int16_t ax;
  int16_t ay;
  int16_t az;
  
  int16_t temp;
  
  int16_t gx;
  int16_t gy;
  int16_t gz;
  
  int16_t mx;
  int16_t my;
  int16_t mz;
}IMUDataTypedef;

typedef struct __MPU6500_RAW_Data__
{
    short Accel_X;  //�Ĵ���ԭ��X����ٶȱ�ʾֵ
    short Accel_Y;  //�Ĵ���ԭ��Y����ٶȱ�ʾֵ
    short Accel_Z;  //�Ĵ���ԭ��Z����ٶȱ�ʾֵ
    short Temp;     //�Ĵ���ԭ���¶ȱ�ʾֵ
    short Gyro_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Gyro_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Gyro_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	  short Mag_X;   //�Ĵ���ԭ��X�������Ǳ�ʾֵ
    short Mag_Y;   //�Ĵ���ԭ��Y�������Ǳ�ʾֵ
    short Mag_Z;   //�Ĵ���ԭ��Z�������Ǳ�ʾֵ
	
}MPU6500_RAW_DATA;

typedef struct __MPU6500_REAL_Data__
{
    float Accel_X;  //ת����ʵ�ʵ�X����ٶȣ�
    float Accel_Y;  //ת����ʵ�ʵ�Y����ٶȣ�
    float Accel_Z;  //ת����ʵ�ʵ�Z����ٶȣ�
    float Temp;     //ת����ʵ�ʵ��¶ȣ���λΪ���϶�
    float Gyro_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Gyro_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Gyro_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	  float Mag_X;   //ת����ʵ�ʵ�X��Ǽ��ٶȣ�
    float Mag_Y;   //ת����ʵ�ʵ�Y��Ǽ��ٶȣ�
    float Mag_Z;   //ת����ʵ�ʵ�Z��Ǽ��ٶ�
	
}MPU6500_REAL_DATA;

//define the eluer angle


extern uint8_t MPU_id;
extern volatile MPU6500_RAW_DATA    MPU6500_Raw_Data; 
extern volatile MPU6500_REAL_DATA   MPU6500_Real_Data;

extern MagMaxMinData_t MagMaxMinData;

uint8_t MPU6500_Init(void);
uint8_t MPU6500_EnableInt(void);
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);

void MPU_Get_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6500_getlastMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);
void error_0(void);   //���������

extern uint8_t isMPU6500_is_DRY;   //INT���� �жϱ�־λ PB8
#endif

