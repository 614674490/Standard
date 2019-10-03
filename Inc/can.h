/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
typedef struct
{
   uint8_t CAN1_Data[8];
	 uint8_t CAN2_Data[8];
}CAN_Msg;

typedef struct{
	float chassis_power;
	uint16_t chassis_power_buffer;
}Pow_Msg_t;

typedef union{
	uint8_t buff[8];
	Pow_Msg_t Data;
}Rx_t;//给电容端发送

typedef struct{
	float VCAP;
	float Power_Set;
}CAP_Msg_t;

typedef union{
	uint8_t buff[8];
	CAP_Msg_t Data;
}Tx_t;//接收

extern CAN_Msg CAN_Message;
extern CAP_Msg_t CAP_Message;
extern Pow_Msg_t POWER_Message;
extern Tx_t CAP_Receive;
extern Rx_t CAP_Send;
extern uint8_t databuf[8];
extern uint8_t databuf_PTZ[8];
extern uint32_t can_count_y;
extern uint32_t count;
extern float CAP_volt;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CanFilter_Init(CAN_HandleTypeDef* hcan);
uint8_t CAN1_Send_Msg_Chassis(uint8_t* msg);
uint8_t CAN1_Receive_Msg_Chassis(uint8_t *buf);
uint8_t CAN2_Send_Msg_Chassis(uint8_t* msg);
uint8_t CAN2_Receive_Msg_Chassis(uint8_t *buf);
uint8_t CAN1_Send_Msg_PTZ(uint8_t* msg);
uint8_t CAN1_Receive_Msg_PTZ(uint8_t *buf);
uint8_t CAN2_Send_Msg_CAP(uint8_t* msg);
uint8_t CAN2_Receive_Msg_CAP(uint8_t *buf);
void CAN2_Data_Proess(void);
void CAN1_Data_Proess(void);
float Check_Speed(void);

//函数指针
extern uint8_t (*Chaais_Send) (uint8_t*);
extern uint8_t (*Chaais_Receive) (uint8_t*);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
