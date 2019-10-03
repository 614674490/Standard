#include "include.h"
int Init_End_Flag=0;   //初始化完成标志位
void BSP_Init()
{
	POWER_OFF();                                                    //关闭主控板上的24V电源口   POWER_ON() 为开启24V电源
	set_random_music();                                             //生成初始化随机音乐
	HAL_TIM_Base_Start_IT(&htim3);                                  //定时器3中断初始化
	All_kalman_init();                                              //卡尔曼初始化
	ADRC_Init();                                                    //ADRC初始化 一种可代替PID的控制器 未使用
	LASER_OFF();                                                    //关闭激光笔 由于现在激光笔直接连到云台的IMU的5V电源上 一直开启 故此句无用
	PID_Init();                                                     //初始化PID参数
  FPID_Init();                                                    //初始化模糊PID参数 未用模糊PID
	Chassis_Speed_Ref_Init();                                       //底盘相关参数置零 不然初始化完成后 一旦拨动遥控器 步兵底盘就会自动行驶	
	#if MPU_STATUS!=MPU_USART                                       //使用板载陀螺仪
	MPU6500_Init();            																			//	mpu寄存器初始化
	HAL_Delay(1000);                                                //延时1000ms 保证imu初始化完成
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);  											//加热电阻 使MPU保持恒温 解决温飘
	TIM_SetTIM3Compare2(100);                                       //开启定时器3的PWM通道二 用于加热IMU的加热电阻
	#else                                                           //使用外置陀螺仪
	 __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE); 									//串口陀螺仪数据接收
  HAL_UART_Receive_DMA(&huart8, usart_mpu_bf, usart_mpu_buflen);  //开启串口8的DMA接收
	#endif
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 									  //DBUS数据接收
  HAL_UART_Receive_DMA(&huart1, usart_dma_bf, BUFLEN);            //开启串口1的DMA接收
	
	#if INFANTRY==3
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE); 									  //摄像头数据接收
  HAL_UART_Receive_DMA(&huart2, camera_buff, MINIPC_BUFFLEN);     //开启串口2的DMA接收
	#else
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE); 									//超级电容数据接收
  HAL_UART_Receive_DMA(&huart7, camera_buff, MINIPC_BUFFLEN);
	#endif
	HAL_UART_Receive_DMA(&huart6, judge_buff, judgelen);           //裁判系统串口接收 此处不能开启DMA接收 而是使用串口回调函数接收数据 不然会存在一些BUG
	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);                                         //初始化CAN滤波器 
	HAL_TIM_Base_Start(&htim2);        															//开启定时器2
	HAL_TIM_Base_Start(&htim5);       															//开启定时器5
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1); 											  //新版摩擦轮PWM通道初始化
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);   											//弹仓舵机PWM
  
#if CHASSIS_FEEDBACK==2    																				//开启电流计检测
    HAL_ADC_Start(&hadc1); 																			  //开启ADC中断
#endif
 
}

//任务挂起函数
void Task_Suspend(void)
{
	vTaskSuspend(GimbalTaskHandle);
//	vTaskSuspend(IMUTaskHandle);
	vTaskSuspend(RemoteTaskHandle);
	vTaskSuspend(ChassisTaskHandle);
	vTaskSuspend(MiniPCTaskHandle);
	vTaskSuspend(CapacityTaskHandle);
}

//任务恢复函数
void Task_Resume(void)
{
	vTaskResume(GimbalTaskHandle);
	#if MPU_STATUS!=MPU_USART
	vTaskResume(IMUTaskHandle);
	#endif
	vTaskResume(RemoteTaskHandle);
	vTaskResume(ChassisTaskHandle);
	vTaskResume(MiniPCTaskHandle);
	//vTaskResume(CapacityTaskHandle);
}


