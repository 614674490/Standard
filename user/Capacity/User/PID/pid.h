#ifndef _pid_h
#define _pid_h
#include "rtos.h"

typedef struct PID_INCREASE
{
	float Kp;
	float Ki;
	float Kd;
	float error_now;
	float error_next;
	float error_last;
	float increament;
}PID_ADD;

void PID_Init(void);

extern float PID_Increment(float current,float expect);
extern PID_ADD Power_limit;



#endif

