#include "include.h"

int opencam_flag=0,patrol_flag=1;
AIMAUTO_DATA aimauto_yaw/*={0,0,0,0,0,0,0,0,0,0}*/;
AIMAUTO_DATA aimauto_pitch={0,0,0,0,0,0,0,0,0,0};
AIMAUTO_DATA aimauto_dist={0,0,0,0,0,0,0,0,0,0};
int16_t yaw_rec,pitch_rec,dist_rec,thelta_raw;
float y_angle,p_angle,yaw_delay,y_angle_offset,yaw_speed_filter,dist_filter,yaw_offset;
float pitch_delay,p_angle_offset,pitch_speed_filter,pitch_camera_filter=0;
float prediction_testy;
#if INFANTRY!=3   //自瞄

void aimauto_control(void)
{
				 yaw_rec=((int16_t)camera_buff[2]<<8)|((int16_t)camera_buff[1]);//yaw
				 pitch_rec=((int16_t)camera_buff[4]<<8)|((int16_t)camera_buff[3]);//pitch	
				 dist_rec=((int16_t)camera_buff[6]<<8)|((int16_t)camera_buff[5]);//帧率
				 thelta_raw=((int16_t)camera_buff[8]<<8)|((int16_t)camera_buff[7]);
				 y_angle=(float)yaw_rec/10.0f;
				 p_angle=-(float)pitch_rec/10.0f;
				 dist_raw=(float)dist_rec/10.0f;

			if(y_angle!=0&&key_mouse_inf.autoaim_mode==1)      //脱离视野时保持当前值
			{		   
					yaw_speed_raw=target_speed_calc(&yaw_speed_struct,soft_js_time,y_anglesave);
					dist_speed_raw=target_speed_calc(&dist_speed_struct,soft_js_time,dist_raw);
					yaw_delay=Yawangle_delay.gimbal_attitude_archive[Yawangle_delay.get_index];
					
					y_angle_offset=y_angle/2.45f;
					y_anglesave=yaw_delay+y_angle_offset;						  
					yaw_speed_filter=yaw_kalman_filter.filtered_value[1];
					yaw_filter=yaw_kalman_filter.filtered_value[0];
					dist_filter=dist_kalman_filter.filtered_value[0];
					cameraopen_yaw = yaw_filter+yaw_speed_filter*(0.14f+dist_filter/450.0f)-angle_offset-GimbalRef.yaw_angle_dynamic_ref;		
			}
					 
			if(p_angle!=0&&key_mouse_inf.autoaim_mode==1)
			{	   
					pitch_speed_raw=target_speed_calc(&pitch_speed_struct,soft_js_time,cameraopen_pitch);
					pitch_delay=Pitchangle_delay.gimbal_attitude_archive[Pitchangle_delay.get_index];
				
					p_angle_offset=p_angle/2.60f;						  
					p_anglesave=pitch_delay+p_angle_offset;
					pitch_speed_filter=pitch_kalman_filter.filtered_value[1];	
					pitch_camera_filter=pitch_kalman_filter.filtered_value[0]+dist_filter*0.1f-GimbalRef.pitch_angle_dynamic_ref;						
			}	 
}
#else     //大符

void aimauto_control(void)
{
				 yaw_rec=((int16_t)camera_buff[2]<<8)|((int16_t)camera_buff[1]);//yaw
				 pitch_rec=((int16_t)camera_buff[4]<<8)|((int16_t)camera_buff[3]);//pitch	大符取-
				 dist_rec=((int16_t)camera_buff[6]<<8)|((int16_t)camera_buff[5]);//帧率
				 thelta_raw=((int16_t)camera_buff[8]<<8)|((int16_t)camera_buff[7]);
				 y_angle=(float)yaw_rec/1000.0f;
				 p_angle=-(float)pitch_rec/1000.0f;
				 dist_raw=(float)dist_rec/1000.0f;

			if(y_angle!=0&&key_mouse_inf.autoaim_mode==1)      //脱离视野时保持当前值
			{		  
					yaw_speed_raw=target_speed_calc(&yaw_speed_struct,soft_js_time,y_anglesave);
					dist_speed_raw=target_speed_calc(&dist_speed_struct,soft_js_time,dist_raw);
					yaw_delay=Yawangle_delay.gimbal_attitude_archive[Yawangle_delay.get_index];
					
					y_angle_offset=y_angle/0.80f;
					y_anglesave=yaw_delay+y_angle_offset;						  
					yaw_speed_filter=yaw_kalman_filter.filtered_value[1];
					yaw_filter=yaw_kalman_filter.filtered_value[0];
					dist_filter=dist_kalman_filter.filtered_value[0];
					cameraopen_yaw = yaw_filter/*+yaw_speed_filter*(0.11f+dist_filter/450.0f)*/-angle_offset-GimbalRef.yaw_angle_dynamic_ref;		
			}
					 
			if(p_angle!=0&&key_mouse_inf.autoaim_mode==1)
			{	   
					pitch_speed_raw=target_speed_calc(&pitch_speed_struct,soft_js_time,cameraopen_pitch);
					pitch_delay=Pitchangle_delay.gimbal_attitude_archive[Pitchangle_delay.get_index];
				
					p_angle_offset=p_angle/0.95f;						  
					p_anglesave=pitch_delay+p_angle_offset;
					pitch_speed_filter=pitch_kalman_filter.filtered_value[1];	
					pitch_camera_filter=pitch_kalman_filter.filtered_value[0]/*+dist_filter*0.1f*/-GimbalRef.pitch_angle_dynamic_ref;						
			}	 
}
#endif




