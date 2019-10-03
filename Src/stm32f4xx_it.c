/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "include.h"
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */
//u32 gimbal_reset_jis=0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream3 global interrupt.
  */
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	
  CAN1_Data_Proess();
  #if 0
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
	#endif
   __HAL_CAN_ENABLE_IT(&hcan1,CAN_IER_FMPIE0); //此句可解决进入CAN接收中断失败的问题
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
   __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);//陀螺仪触发定时中断 清除中断
   GetPitchYawGxGyGz();        //读取姿态数据,数据已经处理成连续方式		
   isMPU6500_is_DRY = 1;   //mpu6500中断标志
	#if 0
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
  #endif
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	if(gimbal_power)  //等待云台上电后再进行计数
		soft_js_time++;
	#if 0
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	#endif
	if(Init_End_Flag==0)    //初始化进程
	{
		#if CHASSIS_MOTOR_TYPE==M3510
			Current_Init();                         //电流计初始化
		#endif
		if(auto_counter>=180)
			auto_counter=0;
		else
			auto_counter++;
	}
  else
	{
		DATA_delay(&Yawangle_delay,yaw_usart_angle,28);//28
		DATA_delay(&Pitchangle_delay,pitch_usart_angle,1);//30

		#if FRICTION_420S_STATUS
		if(friction_wheel_state==FRICTION_WHEEL_ON)
			Friction_420S_ON(2000);
		else if(friction_wheel_state==FRICTION_WHEEL_OFF)
			Friction_420S_OFF();
		#endif
		#if Client_Robot_Iteractive  //进行机器人 客户端交互
	
		if(soft_js_time%200==0)  //发送频率 10HZ
		{
			Client_custom_robot_data_Send();
		}
			
		#endif
		#if PTZ_MODE==PTZ_DEBUG	
		if(soft_js_time%2==0)
			sin_data=System_identification(Period,Omg,40,30);
		Gimbal_Debug();
		#endif
		
		
	}
	 __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	 BaseType_t xHigherPriorityTaskWoken_Remote;
	 if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) && __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET)//是否触发IDLE中断
	 {
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);  //每次都要清除空闲中断
				(void)huart1.Instance->SR;
				(void)huart1.Instance->DR;
			HAL_UART_DMAStop(&huart1);    //停止DMA接收
		 
		 if(RemoteTaskHandle!=NULL)                  //接收到数据，并且接收任务通知的任务有效
	   {
			  HAL_UART_Receive_DMA(&huart1, usart_dma_bf, BUFLEN);
		    vTaskNotifyGiveFromISR(RemoteTaskHandle,&xHigherPriorityTaskWoken_Remote);//发送任务通知 使通知置+1
		    portYIELD_FROM_ISR(xHigherPriorityTaskWoken_Remote);//如果需要的话进行一次任务切换
	   }
		 else
		 {
			 HAL_UART_Receive_DMA(&huart1, usart_dma_bf, BUFLEN);
		 }
	 }
	 #if 0
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	#endif
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	BaseType_t xHigherPriorityTaskWoken_MiniPC;
	if((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) && __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE) != RESET)//是否触发IDLE中断
	 {
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);  //每次都要清除空闲中断
				(void)huart2.Instance->SR;
				(void)huart2.Instance->DR;
			HAL_UART_DMAStop(&huart2);    //停止DMA接收
		 
		 if(MiniPCTaskHandle!=NULL&&camera_buff[0]==0xFF&&camera_buff[9]==0xFE)            
	   {
			 HAL_UART_Receive_DMA(&huart2, camera_buff, MINIPC_BUFFLEN);
		   vTaskNotifyGiveFromISR(MiniPCTaskHandle,&xHigherPriorityTaskWoken_MiniPC);	//发送任务通知 使通知置+1
	     portYIELD_FROM_ISR(xHigherPriorityTaskWoken_MiniPC);//如果需要的话进行一次任务切换
	   }
		 else
		 {
			 HAL_UART_Receive_DMA(&huart2, camera_buff, MINIPC_BUFFLEN);
		 }
	 }
	 #if 0
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
  #endif
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX1 interrupt.
  */
void CAN2_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX1_IRQn 0 */
  CAN2_Data_Proess();
	#if 0
  /* USER CODE END CAN2_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX1_IRQn 1 */
	#endif
   __HAL_CAN_ENABLE_IT(&hcan2,CAN_IER_FMPIE1); //此句可解决进入CAN接收中断失败的问题
  /* USER CODE END CAN2_RX1_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	BaseType_t xHigherPriorityTaskWoken;
  /* USER CODE END USART6_IRQn 0 */
  if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET) && __HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_IDLE) != RESET)//是否触发IDLE中断
 {
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);  //每次都要清除空闲中断
			(void)huart6.Instance->SR;
			(void)huart6.Instance->DR;
		HAL_UART_DMAStop(&huart6);    //停止DMA接收
	 if(JudgeQueueHandle!=NULL)   //每接收完30个数据 进行一次处理
	 {
			HAL_UART_Receive_DMA(&huart6, judge_buff, judgelen);
		  xQueueSendToBackFromISR(JudgeQueueHandle,judge_buff,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	 }
	 else if(JudgeQueueHandle==NULL)
	{
		HAL_UART_Receive_DMA(&huart6, judge_buff, judgelen);
	}
	 
 }
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}

/**
  * @brief This function handles UART7 global interrupt.
  */
void UART7_IRQHandler(void)
{
  /* USER CODE BEGIN UART7_IRQn 0 */
	BaseType_t xHigherPriorityTaskWoken_MiniPC;
 if((__HAL_UART_GET_FLAG(&huart7, UART_FLAG_IDLE) != RESET) && __HAL_UART_GET_IT_SOURCE(&huart7, UART_IT_IDLE) != RESET)//是否触发IDLE中断
 {
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);  //每次都要清除空闲中断
			(void)huart7.Instance->SR;
			(void)huart7.Instance->DR;
		HAL_UART_DMAStop(&huart7);    //停止DMA接收
	 if(MiniPCTaskHandle!=NULL&&camera_buff[0]==0xFF&&camera_buff[9]==0xFE)            
	 {
		 HAL_UART_Receive_DMA(&huart2, camera_buff, MINIPC_BUFFLEN);
		 vTaskNotifyGiveFromISR(MiniPCTaskHandle,&xHigherPriorityTaskWoken_MiniPC);	//发送任务通知 使通知置+1
		 portYIELD_FROM_ISR(xHigherPriorityTaskWoken_MiniPC);//如果需要的话进行一次任务切换
	 }
	 else
	 {
		 HAL_UART_Receive_DMA(&huart2, camera_buff, MINIPC_BUFFLEN);
	 }
 }
 #if 0
  /* USER CODE END UART7_IRQn 0 */
  HAL_UART_IRQHandler(&huart7);
  /* USER CODE BEGIN UART7_IRQn 1 */
#endif
  /* USER CODE END UART7_IRQn 1 */
}

/**
  * @brief This function handles UART8 global interrupt.
  */
void UART8_IRQHandler(void)
{
  /* USER CODE BEGIN UART8_IRQn 0 */
	 if((__HAL_UART_GET_FLAG(&huart8, UART_FLAG_IDLE) != RESET) && __HAL_UART_GET_IT_SOURCE(&huart8, UART_IT_IDLE) != RESET)//是否触发IDLE中断
	 {
		 
			__HAL_UART_CLEAR_IDLEFLAG(&huart8);  //每次都要清除空闲中断
				(void)huart8.Instance->SR;
				(void)huart8.Instance->DR;
			HAL_UART_DMAStop(&huart8);    //停止DMA接收
		  HAL_UART_Receive_DMA(&huart8, usart_mpu_bf, usart_mpu_buflen);
	 }
	#if 0	      
  /* USER CODE END UART8_IRQn 0 */
  HAL_UART_IRQHandler(&huart8);
  /* USER CODE BEGIN UART8_IRQn 1 */
  #endif
  /* USER CODE END UART8_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t xHigherPriorityTaskWoken;
	if(JudgeQueueHandle!=NULL)
	{
		if(huart->Instance == USART6)
		{
			HAL_UART_Receive_DMA(&huart6, judge_buff, judgelen);
			xQueueSendToBackFromISR(JudgeQueueHandle,judge_buff,&xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
	else if(huart->Instance == USART6)
	{
		HAL_UART_Receive_DMA(&huart6, judge_buff, judgelen);
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
