#include "pid.h"
PID_ADD Power_limit;

/**************************实现函数********************************************
*函数原型:	  float PID_Increment(float current,float expect)
*功　　能:	 功率控制PID
输入参数：  current：当前值
           expected：目标值
输出参数： 设定值
*******************************************************************************/
float PID_Increment(float current,float expect)//功率环控制的PID代码
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

