#ifndef _ADRC_H_
#define _ADRC_H_

#include "sys.h"
#include "math.h"

typedef enum  //枚举，变量值从0开始依次加1
{
    Yaw_ADRC_CONTROL=0,
    PITCH_ADRC_CONTROL,
}ADRC_ID_e;

typedef struct
{
/*****安排过度过程*******/
float x1;//跟踪微分期状态量
float x2;//跟踪微分期状态量微分项
float r;//时间尺度
float h;//ADRC系统积分时间
uint16_t N0;//跟踪微分器解决速度超调h0=N*h

float h0;
float fh;//最速微分加速度跟踪量
/*****扩张状态观测器*******/
/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
float z1;
float z2;
float z3;//根据控制对象输入与输出，提取的扰动信息
float e;//系统状态误差
float y;//系统输出量
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b;


/**********系统状态误差反馈率*********/
float e0;//状态误差积分项
float e1;//状态偏差
float e2;//状态量微分项
float u0;//非线性组合系统输出
int u;//带扰动补偿后的输出
int filter_u;
float b0;//扰动补偿

/*********第一种组合形式*********/
float beta_0;//线性
float beta_1;//非线性组合参数
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
/*********第二种组合形式*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
float alpha2;//0<alpha1<1<alpha2
float zeta;//线性段的区间长度
/*********第三种组合形式*********/
float h1;//u0=-fhan(e1,e2,r,h1);
uint16_t N1;//跟踪微分器解决速度超调h0=N*h
/*********第四种组合形式*********/
float c;//u0=-fhan(e1,c*e2*e2,r,h1);


}Fhan_Data;

void ADRC_Init(void);
//void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2);
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);
void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback);

extern Fhan_Data ADRC_Yaw_Controller,ADRC_Yaw_Speed_Controller,ADRC_Pitch_Controller;
#endif

