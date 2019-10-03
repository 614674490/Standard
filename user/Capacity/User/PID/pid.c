#include "pid.h"
PID_ADD Power_limit;

/**************************ʵ�ֺ���********************************************
*����ԭ��:	  float PID_Increment(float current,float expect)
*��������:	 ���ʿ���PID
���������  current����ǰֵ
           expected��Ŀ��ֵ
��������� �趨ֵ
*******************************************************************************/
float PID_Increment(float current,float expect)//���ʻ����Ƶ�PID����
{
	
     Power_limit.error_now=expect-current;
	   Power_limit.increament=Power_limit.Kp*(Power_limit.error_now-Power_limit.error_next)+Power_limit.Ki*(Power_limit. error_now)+
	                          Power_limit.Kd*(Power_limit.error_now-2*Power_limit.error_next+Power_limit.error_last);

     Power_limit.error_last=Power_limit.error_next;
	   Power_limit.error_next=Power_limit.error_now;
	   
	  return Power_limit.increament;
}



void PID_Init()
{
		Power_limit.Kp=0.8;   // 0.8
		Power_limit.Ki=1.2;  
		Power_limit.Kd=0;
		Power_limit.error_last=0;
		Power_limit.error_next=0;
		Power_limit.error_now=0;

}

