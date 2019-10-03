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

/***********************   陀螺仪参数修改↓   ************************/		 //
//    #define      AXSET                  -39.5f    //加速度计x轴 -39.5 36.5 
//		#define      AYSET                   78   //加速度计y轴     77
//		#define      AZSET                   200    //加速度计z轴  4296-200=4096---重力加速度
//		#define      GXSET                   0.5f  //陀螺仪x轴       1
//		#define      GYSET                  -1.5f    //陀螺仪y轴   -1.5f
//		#define      GZSET                   0.5f    //陀螺仪z轴     0


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
    short Accel_X;  //寄存器原生X轴加速度表示值
    short Accel_Y;  //寄存器原生Y轴加速度表示值
    short Accel_Z;  //寄存器原生Z轴加速度表示值
    short Temp;     //寄存器原生温度表示值
    short Gyro_X;   //寄存器原生X轴陀螺仪表示值
    short Gyro_Y;   //寄存器原生Y轴陀螺仪表示值
    short Gyro_Z;   //寄存器原生Z轴陀螺仪表示值
	  short Mag_X;   //寄存器原生X轴陀螺仪表示值
    short Mag_Y;   //寄存器原生Y轴陀螺仪表示值
    short Mag_Z;   //寄存器原生Z轴陀螺仪表示值
	
}MPU6500_RAW_DATA;

typedef struct __MPU6500_REAL_Data__
{
    float Accel_X;  //转换成实际的X轴加速度，
    float Accel_Y;  //转换成实际的Y轴加速度，
    float Accel_Z;  //转换成实际的Z轴加速度，
    float Temp;     //转换成实际的温度，单位为摄氏度
    float Gyro_X;   //转换成实际的X轴角加速度，
    float Gyro_Y;   //转换成实际的Y轴角加速度，
    float Gyro_Z;   //转换成实际的Z轴角加速度
	  float Mag_X;   //转换成实际的X轴角加速度，
    float Mag_Y;   //转换成实际的Y轴角加速度，
    float Mag_Z;   //转换成实际的Z轴角加速度
	
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
void error_0(void);   //零点误差处理函数

extern uint8_t isMPU6500_is_DRY;   //INT引脚 中断标志位 PB8
#endif

