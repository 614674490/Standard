/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "include.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */
osThreadId GimbalTaskHandle;
osThreadId IMUTaskHandle;
osThreadId RemoteTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId JudgeTaskHandle;
osThreadId MiniPCTaskHandle;
osThreadId CapacityTaskHandle;
osThreadId StartTaskHandle;
osThreadId BuzzyTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//裁判系统队列
xQueueHandle JudgeQueueHandle;


/* USER CODE END FunctionPrototypes */

void Gimbal_Task(void const * argument);
void IMU_Task(void const * argument);
void Remote_Task(void const * argument);
void Chassis_Task(void const * argument);
void Judge_Task(void const * argument);
void MinPC_Task(void const * argument);
void Capacity_Task(void const * argument);
void Start_Task(void const * argument);
void Buzzy_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
    static u8 flag=0;
	static u8 offline_count=0;
	  static uint32_t systemTime;
		if(flag==0) 
		{
			DetectInit();
			flag=1;
		}
		taskENTER_CRITICAL();
		systemTime = xTaskGetTickCount();
		errorList[errorListLength].isLost = 0;
		for (int i = 0; i < errorListLength; i++)
		{
				//未使能，跳过
				if (errorList[i].enable == 0)
				{
						continue;
				}

				//判断掉线
				if ((systemTime - errorList[i].newTime) > errorList[i].setOfflineTime)
				{
					  offline_count++;
					  if(offline_count>8)
						{
							if (errorList[i].isLost == 0)
							{
									//记录错误以及掉线时间
									errorList[i].isLost = 1;
									errorList[i].Losttime = systemTime;
							}
							if (errorList[i].solveOfflineFun != NULL)
							{
									errorList[i].solveOfflineFun();
							}
							offline_count=0;
						}
						
				}
		}
		DetectDisplay();
		taskEXIT_CRITICAL();
}
/* USER CODE END 2 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */ 

	
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, Gimbal_Task, osPriorityNormal, 0, 128);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, IMU_Task, osPriorityRealtime, 0, 256);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of RemoteTask */
  osThreadDef(RemoteTask, Remote_Task, osPriorityAboveNormal, 0, 256);
  RemoteTaskHandle = osThreadCreate(osThread(RemoteTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, Chassis_Task, osPriorityBelowNormal, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of JudgeTask */
  osThreadDef(JudgeTask, Judge_Task, osPriorityHigh, 0, 256);
  JudgeTaskHandle = osThreadCreate(osThread(JudgeTask), NULL);

  /* definition and creation of MiniPCTask */
  osThreadDef(MiniPCTask, MinPC_Task, osPriorityNormal, 0, 256);
  MiniPCTaskHandle = osThreadCreate(osThread(MiniPCTask), NULL);

  /* definition and creation of CapacityTask */
  osThreadDef(CapacityTask, Capacity_Task, osPriorityBelowNormal, 0, 128);
  CapacityTaskHandle = osThreadCreate(osThread(CapacityTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, Start_Task, osPriorityIdle, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* definition and creation of BuzzyTask */
  osThreadDef(BuzzyTask, Buzzy_Task, osPriorityIdle, 0, 128);
  BuzzyTaskHandle = osThreadCreate(osThread(BuzzyTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	Task_Suspend();                    //挂起工作任务
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	JudgeQueueHandle=xQueueCreate(referee_system_QueueLength,referee_system_ItemSize);
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
  * @brief  Function implementing the GimbalTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Gimbal_Task */
void Gimbal_Task(void const * argument)
{

  /* USER CODE BEGIN Gimbal_Task */
	
  /* Infinite loop */
  while(1)
  {	
   #if PTZ_MODE!=PTZ_DEBUG	
		
			Gimbal_Control();
			Gimbal_Data_Send();
   #endif
    vTaskDelay(2);  //延时时间会有影响
  }
  /* USER CODE END Gimbal_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  
  /* Infinite loop */
  while(1)
  {
	
		#if MPU_STATUS!=MPU_USART
		if(isMPU6500_is_DRY==1)  //数值更新
		{
			IMU_getYawPitchRoll(angle);
			MPU_Get_Temperature();
		
			//陀螺仪温度控制
			mpu_pwm+=PID_Increment(mpu_temp,MPU_temp.expect,&MPU_temp);   //保持陀螺仪恒温
			if(mpu_pwm>MPU6500_TEMP_PWM_MAX)   //输出限幅
				mpu_pwm=MPU6500_TEMP_PWM_MAX;
			if(mpu_pwm<0)
				mpu_pwm=0;
			TIM_SetTIM3Compare2(mpu_pwm);
		}
		#else
		Usart_Imu_Data_Process(usart_mpu_bf);
		#endif
    vTaskDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
	u8 NotifyValue=0;
  /* Infinite loop */
  while(1)
  {
		
		NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	//获取任务通知 获取后将通知值清零
		if(NotifyValue==1)
		{
		  taskENTER_CRITICAL();   //解析数据时进入临界区
			Remote_Data_Process(usart_dma_bf);
			taskEXIT_CRITICAL();
		}
		vTaskDelay(1);
		DetectHook(RemoteTOE);
	 }
		
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */

  /* Infinite loop */
  while(1)
  {
	  Chassis_Rotate_Control();
		Chassis_Motor_Control();
		#if CHASSIS_FEEDBACK!=0            //开启功率控制
		Chassis_Power_Control();
		#endif
		Chassis_Data_Send();
		#if CHASSIS_CAN_STATUS==CAN1_Chassis
			vTaskDelay(2);
		#elif CHASSIS_CAN_STATUS==CAN2_Chassis
		  vTaskDelay(1);
		#endif
	}
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_Judge_Task */
/**
* @brief Function implementing the JudgeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Judge_Task */
void Judge_Task(void const * argument)
{
  /* USER CODE BEGIN Judge_Task */
	uint8_t referee_system_buff[judgelen*2]={0};
	uint8_t referee_system_halfbuff[judgelen]={0};
	UBaseType_t buffer_size;
	CRC_StatusTypeDef CRC_structure;
	int i=0;
	portBASE_TYPE err;
  /* Infinite loop */
  while(1)
  {
		err=xQueueReceive(JudgeQueueHandle,referee_system_halfbuff,portMAX_DELAY);
		if(err==pdTRUE)
		{
			buffer_size = uxQueueMessagesWaiting(JudgeQueueHandle);
			if (buffer_size < threshold)CRC_structure = CRC_ENABLE;
				else CRC_structure=CRC_DISABLE;
      taskENTER_CRITICAL();
			for(i=judgelen;i<judgelen*2;i++)
			{
				referee_system_buff[i-judgelen] = referee_system_buff[i];
				referee_system_buff[i] = referee_system_halfbuff[i-judgelen];
			}
			for(i=0;i<judgelen;i++)
			{
				if(referee_system_buff[i] == 0xA5)
				{
					if(referee_system_Rx(referee_system_buff+i, CRC_structure)==OK)
						i += (uint8_t)referee_system_buff[i+2]<<8|referee_system_buff[i+1]-1;
				}
			}
			taskEXIT_CRITICAL();
		}
    DetectHook(JudgeTOE);			
		vTaskDelay(1);
  }
  /* USER CODE END Judge_Task */
}

/* USER CODE BEGIN Header_MinPC_Task */
/**
* @brief Function implementing the MinPCTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MinPC_Task */
void MinPC_Task(void const * argument)
{
  /* USER CODE BEGIN MinPC_Task */
	u8 NotifyValue=0;
  /* Infinite loop */
  while(1)
  {
		NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	
		if(NotifyValue==1)
		{
			  //MinPC数据处理代码
				taskENTER_CRITICAL();
				aimauto_control();
				taskEXIT_CRITICAL();
			  DetectHook(MinPCTOE);
		}
		vTaskDelay(1);
  }
	
  /* USER CODE END MinPC_Task */
}

/* USER CODE BEGIN Header_Capacity_Task */
/**
* @brief Function implementing the CapacityTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Capacity_Task */
void Capacity_Task(void const * argument)
{
  /* USER CODE BEGIN Capacity_Task */
  /* Infinite loop */
  while(1)
  {
		taskENTER_CRITICAL();
		Capacity_Data_Send();
#if CHASSIS_FEEDBACK==2&&CHASSIS_MOTOR_TYPE==M3510   //当底盘电机为3510时 采用电流计
		Get_Current();
#endif
		taskEXIT_CRITICAL();
    vTaskDelay(1);
  }
  /* USER CODE END Capacity_Task */
}

/* USER CODE BEGIN Header_Start_Task */
/**
* @brief Function implementing the StartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Task */
void Start_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Task */
  /* Infinite loop */
	static u16 Init_time=0;
	
	#if MPU_STATUS!=MPU_USART
		Init_time=11000;
	#elif  MPU_STATUS==MPU_USART&&CHASSIS_MOTOR_TYPE==M3510                                      //开启串口陀螺仪后 减少初始化时间
	  Init_time=8000;
	#elif  MPU_STATUS==MPU_USART&&CHASSIS_MOTOR_TYPE==M3508                                      //开启串口陀螺仪后 减少初始化时间
	  Init_time=5000;
	#endif
	BSP_Init();                               //外设初始化
	Wait_Power();                             //等待云台上电 不然云台有时初始化是会疯掉
	gimbal_power=1;
  while(soft_js_time<Init_time)
  {
		Magaine_OFF();                          //关闭弹仓
		Friction_Init();                        //摩擦轮初始化
	  if(soft_js_time>(Init_time-2000))                  //先初始化陀螺仪 后初始化云台
			Gimbal_Init();                          //云台初始化
		#if MPU_STATUS!=MPU_USART    //不采用独立串口陀螺仪
		IMU_Init();                   //自带陀螺仪初始化
		#endif
    vTaskDelay(1);
  }
	
	taskENTER_CRITICAL();
	power_offset=POWER_OFFSET;   //加入超级电容之后 不能读取静止时裁判系统的功率 因为此时也包括了超级电容的充电功率 否则步兵的速度会变慢
	pitch_angle_offset=pitch_usart_angle;  //P轴补偿角度
	#if MPU_STATUS!=MPU_USART 
	angle_offset=yaw_angle;                //Y轴补偿角度
	#else
	angle_offset=yaw_usart_angle;
	#endif
	Task_Resume();
	LASER_ON();
	Init_End_Flag=1;
	vTaskDelete(BuzzyTaskHandle);
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);
	taskEXIT_CRITICAL();
	vTaskDelete(StartTaskHandle);
  /* USER CODE END Start_Task */
}

/* USER CODE BEGIN Header_Buzzy_Task */
/**
* @brief Function implementing the BuzzyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzy_Task */
void Buzzy_Task(void const * argument)
{
  /* USER CODE BEGIN Buzzy_Task */
  /* Infinite loop */
  for(;;)
  {
	  random_play();
  }
  /* USER CODE END Buzzy_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
