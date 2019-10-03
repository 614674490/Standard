                                                    /*用于自瞄*/
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "filter.h"
#include "pid.h"
#include "imu.h"

float y_anglesave=0,p_anglesave=0;
float yaw_speed_raw=0,dist_speed_raw=0,pitch_speed_raw=0;
float cameraopen_yaw=0,cameraopen_pitch=0,dist_raw=0;
extern float y_angle,kd_filter_raw;


kalman_filter_init_t yaw_kalman_filter_data={
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）            
  .R_data = {200,0,0,7000},      // 传感器测量方差（采集数据方差）          
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t dist_kalman_filter_data={
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {0.1, 0, 0, 0.1},           // 外部的不确定性（过程噪声协方差）            
  .R_data = {7000,0,0,2000},      // 传感器测量方差（采集数据方差）          
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t pitch_kalman_filter_data={
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 0.1},           // 外部的不确定性（过程噪声协方差）            
  .R_data = {2000,0,0,10000},      // 传感器测量方差（采集数据方差）          
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t pitch_kp_pid_kalman_filter_data={
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.01, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 0.1},           // 外部的不确定性（过程噪声协方差）            
  .R_data = {500,0,0,5000},      // 传感器测量方差（采集数据方差）          
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t yaw_pid_kalman_filter_data={										//用于抑制PID微分引起的噪声
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.0001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {500,0,0,500},      // 传感器测量方差（采集数据方差）      500 500    
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t pitch_pid_kalman_filter_data={										//用于抑制PID微分引起的噪声
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.002, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {10,0,0,500},      // 传感器测量方差（采集数据方差）      500 500    
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t current_kalman_filter_data={										//用于抑制电流计的噪声
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.0003, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {2000,0,0,500},      // 传感器测量方差（采集数据方差）      500 500    
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t usart_imu_gyro_filter_data={										
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.0001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {100,0,0,100},      // 传感器测量方差（采集数据方差）      500 500    
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t power_filter_data={										//用于抑制PID微分引起的噪声
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {800,0,0,800},      // 传感器测量方差（采集数据方差）      500 500    
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t chassis_rotate_filter_data={										//底盘转向PID滤波
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.0001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {500,0,0,5000},      // 传感器测量方差（采集数据方差）     越小实时性越好 滤波效果越差
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_init_t hit_motor_filter_data={										//底盘转向PID滤波
	.P_data = {2,0,0,2},           // 协方差矩阵           
  .A_data = {1, 0.0001, 0, 1},        // 预测矩阵 （采样时间）
  .H_data = {1, 0, 0, 1}, 					// 传感器测量数据矩阵																					
  .Q_data = {1, 0, 0, 1},           // 外部的不确定性（过程噪声协方差）1   1            
  .R_data = {500,0,0,500},      // 传感器测量方差（采集数据方差）     越小实时性越好 滤波效果越差
	.E_data = {1,0,0,1}            // 单位矩阵
};

kalman_filter_t usart_imu_gyro_filter;
kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_t dist_kalman_filter;
kalman_filter_t yaw_pid_kalman_filter;
kalman_filter_t pitch_pid_kalman_filter;
kalman_filter_t pitch_kp_pid_kalman_filter;
kalman_filter_t current_kalman_filter;
kalman_filter_t chassis_rotate_filter;

kalman_filter_t power_filter;
kalman_filter_t hit_motor_filter;

speed_calc_data_t yaw_speed_struct;
speed_calc_data_t pitch_speed_struct;
speed_calc_data_t dist_speed_struct;

speed_calc_data_t pitch_motor_speed_struct;

Data_delay_t Yawangle_delay;
Data_delay_t Pitchangle_delay;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
  mat_init(&F->xhat,2,1,(float*)I->xhat_data);     //  前行后列
  mat_init(&F->xhatminus,2,1,(float*)I->xhatminus_data);     
	mat_init(&F->z,2,1,(float*)I->z_data);
	mat_init(&F->A,2,2,(float*)I->A_data);
	mat_init(&F->H,2,2,(float*)I->H_data);
	mat_init(&F->AT,2,2,(float*)I->AT_data);
	mat_trans(&F->A, &F->AT);
	mat_init(&F->Q,2,2,(float*)I->Q_data);
	mat_init(&F->R,2,2,(float*)I->R_data);
	mat_init(&F->P,2,2,(float*)I->P_data);
	mat_init(&F->Pminus,2,2,(float*)I->Pminus_data);
	mat_init(&F->K,2,2,(float*)I->K_data);
  mat_init(&F->HT,2,2,(float*)I->HT_data);
	mat_trans(&F->H, &F->HT);
	mat_init(&F->E,2,2,(float*)I->E_data);
}

void All_kalman_init(void)      //所有卡尔曼初始化
{
		kalman_filter_init(&yaw_kalman_filter,&yaw_kalman_filter_data); 
		kalman_filter_init(&dist_kalman_filter,&dist_kalman_filter_data); 
		kalman_filter_init(&pitch_kalman_filter,&pitch_kalman_filter_data); 
	  kalman_filter_init(&yaw_pid_kalman_filter,&yaw_pid_kalman_filter_data); 
	  kalman_filter_init(&pitch_pid_kalman_filter,&yaw_pid_kalman_filter_data); 
	  kalman_filter_init(&pitch_kp_pid_kalman_filter,&pitch_kp_pid_kalman_filter_data); 
	  kalman_filter_init(&current_kalman_filter,&current_kalman_filter_data); 
	kalman_filter_init(&chassis_rotate_filter,&chassis_rotate_filter_data);
	kalman_filter_init(&power_filter,&power_filter_data);
	
	kalman_filter_init(&usart_imu_gyro_filter,&usart_imu_gyro_filter_data);

}

void All_kalman_cal(void)       //所有卡尔曼计算
{
	
		kalman_filter_calc(&yaw_kalman_filter,y_anglesave,yaw_speed_raw);
    kalman_filter_calc(&dist_kalman_filter,dist_raw,dist_speed_raw);
    kalman_filter_calc(&pitch_kalman_filter,p_anglesave,pitch_speed_raw);
	  kalman_filter_calc(&yaw_pid_kalman_filter,PTZ_Motor_PID_Position_Y.ec,PTZ_Motor_PID_Speed_Y.ec);//两环微分滤波
}

void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)         //卡尔曼解算
{ 
  float TEMP_data[4] = {0, 0, 0, 0};
  float TEMP_data21[2] = {0, 0};
  mat TEMP,TEMP21;

  mat_init(&TEMP,2,2,(float *)TEMP_data);
  mat_init(&TEMP21,2,1,(float *)TEMP_data21);	

  F->z.pData[0] = signal1;       // 传感器读数
  F->z.pData[1] = signal2;      

  //1. xhat'(k)= A xhat(k-1)
  mat_mult(&F->A, &F->xhat, &F->xhatminus);   //根据k-1时刻的值预测k时刻的值

  //2. P'(k) = A P(k-1) AT + Q
  mat_mult(&F->A, &F->P, &F->Pminus);        //根据k-1时刻的方差预测k时刻的方差
  mat_mult(&F->Pminus, &F->AT, &TEMP);
  mat_add(&TEMP, &F->Q, &F->Pminus);

  //3. K(k) = P'(k) HT / (H P'(k) HT + R)     //计算卡尔曼增益
  mat_mult(&F->H, &F->Pminus, &F->K);         //计算除数
  mat_mult(&F->K, &F->HT, &TEMP);
  mat_add(&TEMP, &F->R, &F->K);

  mat_inv(&F->K, &F->P);                     //除数转置
  mat_mult(&F->Pminus, &F->HT, &TEMP);
  mat_mult(&TEMP, &F->P, &F->K);             //更新卡尔曼增益

  //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
  mat_mult(&F->H, &F->xhatminus, &TEMP21);  //预测的均值
  mat_sub(&F->z, &TEMP21, &F->xhat);          
  mat_mult(&F->K, &F->xhat, &TEMP21);
  mat_add(&F->xhatminus, &TEMP21, &F->xhat);   //获得最佳估计值

  //5. P(k) = (1-K(k)H)P'(k)
  mat_mult(&F->K, &F->H, &F->P);               //更新总体协方差
  mat_sub(&F->E, &F->P, &TEMP);
  mat_mult(&TEMP, &F->Pminus, &F->P);

  F->filtered_value[0] = F->xhat.pData[0];     
  F->filtered_value[1] = F->xhat.pData[1];

}

float speed_threshold = 10.0f;// 速度阈值（防止数据大幅跳变）
float target_speed_calc(speed_calc_data_t *S,float time,float position)     //解算目标速度值,即对绝对角度做差分
{
   if(y_angle==0)S->delay_cnt++;

  if (time != S->last_time)
  {
    S->speed = (position - S->last_position) / (time - S->last_time) * 1000;
#if 1
    if ((S->speed - S->processed_speed) < -speed_threshold)
    {
        S->processed_speed = S->processed_speed - speed_threshold;
    }
    else if ((S->speed - S->processed_speed) > speed_threshold)
    {
        S->processed_speed = S->processed_speed + speed_threshold;
    }
    else 
#endif
    S->processed_speed = S->speed;
		
    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }
  
  if(S->delay_cnt > 200) // 200ms speed=0  防止目标脱离时因数据未及时更新而保持当前值
  {
    S->processed_speed = 0;
  }

  return S->processed_speed;
}

void DATA_delay(Data_delay_t *D,float delay_object,float delay_index)  //对数据做延时处理
{  
		D->archive_index++;       //存档标志
  if (D->archive_index > 199) //系统延迟时间   
    D->archive_index = 0;
    D->gimbal_attitude_archive[D->archive_index] = delay_object;
	if(D->archive_index >= delay_index)
	{	
    D->get_index = D->archive_index - delay_index;
	}
	else
	{
	  D->get_index = D->archive_index + 200-delay_index;             //delay_index所需延时的时间
	}	
}


