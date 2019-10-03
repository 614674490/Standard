/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "config.h"
#include "arm_math.h"
#include "chassis.h"
#include "motorhead.h"
#include "detect.h"
#include "app.h"
#include "gimbal.h"

float Speed1=0,Speed2=0,Speed3=0,Speed4=0;
float CAP_volt=0;
uint8_t databuf[8]={0};
uint8_t databuf_PTZ[8]={0,0,0,0,0,0,0,0};
CAN_Msg CAN_Message;
Tx_t CAP_Receive;
Rx_t CAP_Send;

//三个电机的CAN计数变量独立 否则可能会有某些电机跳过零点检测
uint32_t can_count_y=0;  //Y轴云台
uint32_t count_p=0;      //P轴云台
uint32_t count_h=0;      //拨弹电机
static CAN_TxHeaderTypeDef  Tx1Message;
static CAN_RxHeaderTypeDef  Rx1Message;
static CAN_TxHeaderTypeDef  Tx2Message;
static CAN_RxHeaderTypeDef  Rx2Message;
uint8_t (*Chaais_Send) (uint8_t*);
uint8_t (*Chaais_Receive) (uint8_t*);
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef canfilter;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
 
	/*can1和CAN2使用不同的滤波器*/
	
	canfilter.FilterBank = 0;              //主can的过滤器编号
  canfilter.SlaveStartFilterBank=14;     //从can的过滤器起始编号 只有当设置两个can时 该参数才有意义
	
	if(hcan == &hcan1)
  {
    canfilter.FilterBank = 0;
    canfilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  }
	
	else if(hcan == &hcan2)   //若初始化CAN2的过滤器 需要首先在cube中开启CAN2
  {
    canfilter.FilterBank = 14;
		canfilter.FilterFIFOAssignment = CAN_RX_FIFO1;
  }
	
	canfilter.FilterActivation = ENABLE;              //激活过滤器
  HAL_CAN_ConfigFilter(hcan, &canfilter);//开启CAN的过滤器配置
	
	HAL_CAN_Start(hcan);                            //开启can   启动can的两个函数
	
	if(hcan == &hcan1)
	{
			HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);  //开启挂起中断允许
			__HAL_CAN_ENABLE_IT(hcan,CAN_IER_FMPIE0);    //此句可解决进入CAN接收中断失败的问题
	}
  else if(hcan == &hcan2)
	{
			HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);  
			__HAL_CAN_ENABLE_IT(hcan,CAN_IER_FMPIE1);  
	}
	#if CHASSIS_CAN_STATUS==CAN1_Chassis
	Chaais_Send=CAN1_Send_Msg_Chassis;
	Chaais_Receive=CAN1_Receive_Msg_Chassis;
	#elif CHASSIS_CAN_STATUS==CAN2_Chassis
	Chaais_Send=CAN2_Send_Msg_Chassis;
	Chaais_Receive=CAN2_Receive_Msg_Chassis;
	#endif

}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
uint8_t CAN2_Send_Msg_Chassis(uint8_t* msg)
{	
    Tx2Message.StdId=0x200;       //标准标识符
    Tx2Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx2Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx2Message.DLC=0x08;                
    if(HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,msg,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK) return 1;
    return 0;		
}

uint8_t CAN2_Receive_Msg_Chassis(uint8_t *buf)
{		   		   
	if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&Rx2Message,buf)!=HAL_OK) return 0;
	return Rx2Message.DLC;	
}
/************************************************/
uint8_t CAN2_Send_Msg_CAP(uint8_t* msg)
{	
    Tx2Message.StdId=0x210;       //标准标识符
    Tx2Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx2Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx2Message.DLC=0x08;                
    if(HAL_CAN_AddTxMessage(&hcan2,&Tx2Message,msg,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK) return 1;
    return 0;		
}

uint8_t CAN2_Receive_Msg_CAP(uint8_t *buf)
{		   		   
	if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&Rx2Message,buf)!=HAL_OK) return 0;
	return Rx2Message.DLC;	
}
/***********************************************/

uint8_t CAN1_Send_Msg_PTZ(uint8_t* msg)
{	
    Tx1Message.StdId=0x1FF;        //标准标识符
//	  Tx1Message.StdId=0x3F0;        //标准标识符
    Tx1Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx1Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx1Message.DLC=0x08;                
		if(HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,msg,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK) return 1;
    return 0;		
}

uint8_t CAN1_Receive_Msg_PTZ(uint8_t *buf)
{		   		   
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,buf)!=HAL_OK) return 0;
	return Rx1Message.DLC;	
}


uint8_t CAN1_Send_Msg_Chassis(uint8_t* msg)
{	
    Tx1Message.StdId=0x200;       //标准标识符
    Tx1Message.IDE=CAN_ID_STD;    //使用标准帧
    Tx1Message.RTR=CAN_RTR_DATA;  //数据帧
    Tx1Message.DLC=0x08;                
    if(HAL_CAN_AddTxMessage(&hcan1,&Tx1Message,msg,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK) return 1;
    return 0;		
}

uint8_t CAN1_Receive_Msg_Chassis(uint8_t *buf)
{		   		   

	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&Rx1Message,buf)!=HAL_OK) return 0;
	return Rx1Message.DLC;	
}

void CAN1_Data_Proess(void)
{
	CAN1_Receive_Msg_PTZ(CAN_Message.CAN1_Data);
	if(gimbal_power)
	{
		count_p++;
		can_count_y++;
		count_h++;
	}
	#if CHASSIS_CAN_STATUS==CAN1_Chassis
	if(Rx1Message.StdId==0x201)   //底盘电机
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_1=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor1TOE);
	}
	
	if(Rx1Message.StdId==0x202)   
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_2=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor2TOE);
	}
	
	if(Rx1Message.StdId==0x203)  
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_3=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3]);
		DetectHook(ChassisMotor3TOE);
	}
	
	if(Rx1Message.StdId==0x204)  
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3]);
	  Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_4=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor4TOE);
	}
	#endif
	if(Rx1Message.StdId==0x205)  //6623电机 闪一下表示205---Y轴
	{
		(can_count_y<=50) ? GetEncoderBias_Y(&mechanical_angle_PTZ_Y):EncoderProcess(&mechanical_angle_PTZ_Y);
		GetY_sin_cos(&mechanical_angle_PTZ_Y);   //计算正弦 余弦值
		yaw_motor_speed_ref=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3])*6;
		if(yaw_motor_speed_ref>32768*6)
				yaw_motor_speed_ref-=65536*6;   //防止过调
		DetectHook(YawGimbalMotorTOE);
	}
	
	else if(Rx1Message.StdId==0x206) 
	{
		(count_p<=50) ? GetEncoderBias_P(&mechanical_angle_PTZ_P):EncoderProcess(&mechanical_angle_PTZ_P);
		 DetectHook(PitchGimbalMotorTOE);
	}
	
	else if(Rx1Message.StdId==0x207) 
	{  
		//此处被断点占有 切勿写程序！！
		(count_h<=50) ? GetEncoderBias(&Dial_the_motor):EncoderProcessHIT(&Dial_the_motor);
		Dial_motor_speed_fdb=(CAN_Message.CAN1_Data[2]<<8|CAN_Message.CAN1_Data[3]);
		if(Dial_motor_speed_fdb>32768)
				Dial_motor_speed_fdb-=65536;   //防止过调
		DetectHook(HITTOE);
	}
}

void CAN2_Data_Proess(void)
{
	CAN2_Receive_Msg_Chassis(CAN_Message.CAN2_Data);
	if(Rx2Message.StdId==0x201)   //底盘电机
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1=(CAN_Message.CAN2_Data[2]<<8|CAN_Message.CAN2_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_1=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor1TOE);
		return;
	}
	
	if(Rx2Message.StdId==0x202)   
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2=(CAN_Message.CAN2_Data[2]<<8|CAN_Message.CAN2_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_2=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor2TOE);
		return;
	}
	
	if(Rx2Message.StdId==0x203)  
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3=(CAN_Message.CAN2_Data[2]<<8|CAN_Message.CAN2_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_3=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor3TOE);
		return;
	}
	
	if(Rx2Message.StdId==0x204)  
	{
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4=(CAN_Message.CAN2_Data[2]<<8|CAN_Message.CAN2_Data[3]);
		Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Torque_Current_4=(CAN_Message.CAN2_Data[4]<<8|CAN_Message.CAN2_Data[5]);
		DetectHook(ChassisMotor4TOE);
		return;
	}
	if(Rx2Message.StdId==0x211)  
	{
		memcpy(CAP_Receive.buff,CAN_Message.CAN2_Data,8);
		CAP_volt=CAP_Receive.Data.VCAP;
		return;
	}
}

float Check_Speed(void)
{
	float Speed_Actual=0;
	//原始数值 转速 RPM
	Speed1=((float)Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1/19.0f)*(158.0f*PI)/60000.0f;  // m/s
	Speed2=((float)Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2/19.0f)*(158.0f*PI)/600.0f; 
	Speed3=((float)Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3/19.0f)*(158.0f*PI)/600.0f;  
	Speed4=((float)Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4/19.0f)*(158.0f*PI)/600.0f;  

	float a=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1+Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2
	        +Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3+Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4;
	float s1=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_1/(a+50);   //计算权重
	float s2=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_2/(a+50);
	float s3=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_3/(a+50);
	float s4=Chassis_Motor_Actual_Speed_ref.Chassis_Motor_Actual_Speed_4/(a+50);
	
	Speed_Actual=(Speed1*s1+Speed2*s2+Speed3*s3+Speed4*s4)/4.0f;
	
	return Speed_Actual;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
