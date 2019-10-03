#ifndef __FILTER_H
#define __FILTER_H

#include "sys.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"


#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32

extern float y_anglesave,p_anglesave;
extern float yaw_speed_raw,dist_speed_raw,pitch_speed_raw;
extern float cameraopen_yaw,cameraopen_pitch,dist_raw,pitch_camera_filter;

typedef struct                //卡尔曼矩阵
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K, E;
} kalman_filter_t;

typedef struct               //卡尔曼初始化
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[2],Pminus_data[4], K_data[4];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[4];
  float Q_data[4];
  float R_data[4];
	float E_data[4];
} kalman_filter_init_t;

typedef struct                //数据延迟结构体
{
	float gimbal_attitude_archive[200];
	uint8_t archive_index;
  uint8_t get_index;
} Data_delay_t;

typedef struct         //速度解算结构体
{
  int delay_cnt;
  int freq;
  int last_time;
  float last_position;
  float speed;
  float last_speed;
  float processed_speed;
} speed_calc_data_t;

extern kalman_filter_t yaw_kalman_filter;
extern kalman_filter_t pitch_kalman_filter;
extern kalman_filter_t dist_kalman_filter;
extern kalman_filter_t yaw_pid_kalman_filter;
extern kalman_filter_t current_kalman_filter;
extern kalman_filter_t pitch_pid_kalman_filter;
extern kalman_filter_t pitch_kp_pid_kalman_filter;
extern kalman_filter_t usart_imu_gyro_filter;
extern kalman_filter_t power_filter;
extern kalman_filter_t chassis_rotate_filter;

extern kalman_filter_init_t yaw_kalman_filter_data;
extern kalman_filter_init_t pitch_kalman_filter_data;
extern kalman_filter_init_t dist_kalman_filter_data;
extern kalman_filter_init_t yaw_pid_kalman_filter_data;
extern kalman_filter_init_t pitch_pid_kalman_filter_data;
extern kalman_filter_init_t current_kalman_filter_data;
extern kalman_filter_init_t pitch_kp_pid_kalman_filter_data;
extern kalman_filter_init_t power_filter_data;
extern kalman_filter_init_t usart_imu_gyro_filter_data;
extern kalman_filter_init_t chassis_rotate_filter_data;
extern speed_calc_data_t yaw_speed_struct;
extern speed_calc_data_t pitch_speed_struct;
extern speed_calc_data_t dist_speed_struct;
extern speed_calc_data_t pitch_motor_speed_struct;;
extern Data_delay_t Yawangle_delay;
extern Data_delay_t Pitchangle_delay;


extern float speed_threshold;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void All_kalman_init(void);
void All_kalman_cal(void);
void kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2);
float target_speed_calc(speed_calc_data_t *S, float time, float position);
void DATA_delay(Data_delay_t *D,float delay_object,float delay_index);
#endif

