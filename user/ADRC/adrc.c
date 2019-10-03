#include "ADRC.h"
#include "math.h"

Fhan_Data ADRC_Yaw_Controller;
Fhan_Data ADRC_Yaw_Speed_Controller;
Fhan_Data ADRC_Pitch_Controller;

#define ABS(X)  (((X)>0)?(X):-(X))

const float ADRC_Unit[3][16]=
{
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      50000,     0.001,    0.002,   200,     1.5,        5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0},
};

/*
*r越小，跟踪速度和跟踪加速度越平滑，但滞后性越大；越大，相反
*h越大，跟踪加速度波动越小，幅值越小
*N越大，跟踪速度和加速度都变平滑，幅值也相应变小
*
*beta_01越小，三个输出的噪声越大，越大越平滑，但会造成滞后
*beta_02越大，z2,z3的噪声(高频)越大；越小，z1,z2,z3都有正弦波动，z2,z3尤为剧烈
*beta_03越大，z3的噪声越大，越小越平滑
*b0
*
*beta_0，暂未使用
*bata_1越大，增益越大
*beta_2越大，响应越快，实时性越好，但稳定性变差
*
*alpha1越小，实时性越好，但稳定性和增益都变差，越大，相反
*alpha2越小，越平滑；越大，噪声越大，但实时性和增益都变好
*zeta越小，越平滑，但实时性不好；越大，实时性越好，但有噪声
*b
*/

     
float Constrain_Float(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int16_t Sign_ADRC(float Input)
{
    int16_t output=0;
    if((double)Input>1E-6) output=1;
    else if((double)Input<-1E-6) output=-1;
    else output=0;
    return output;
}

int16_t Fsg_ADRC(float x,float d)
{
  int16_t output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

void ADRC_Init(void)
{
/*****位置环*****/
	ADRC_Yaw_Controller.r=300000;//TD跟踪微分器
  ADRC_Yaw_Controller.h=0.002;
  ADRC_Yaw_Controller.N0=3;	
	
  ADRC_Yaw_Controller.beta_01=300;//扩张状态观测器ESO
  ADRC_Yaw_Controller.beta_02=4000;
  ADRC_Yaw_Controller.beta_03=50000;	
	
  ADRC_Yaw_Controller.b0=0.001;//扰动补偿
	
  ADRC_Yaw_Controller.beta_0=0.002;//非线性组合
  ADRC_Yaw_Controller.beta_1=250;
  ADRC_Yaw_Controller.beta_2=20;
  ADRC_Yaw_Controller.N1=5;
  ADRC_Yaw_Controller.c=5;
	
  ADRC_Yaw_Controller.alpha1=0.40;/*****/
  ADRC_Yaw_Controller.alpha2=0.60;
  ADRC_Yaw_Controller.zeta=8;
  ADRC_Yaw_Controller.b=0;
	
/*********速度环**********/	
	ADRC_Yaw_Speed_Controller.r=300000;//TD跟踪微分器
  ADRC_Yaw_Speed_Controller.h=0.005;
  ADRC_Yaw_Speed_Controller.N0=3;	
	
  ADRC_Yaw_Speed_Controller.beta_01=300;//扩张状态观测器ESO
  ADRC_Yaw_Speed_Controller.beta_02=4000;
  ADRC_Yaw_Speed_Controller.beta_03=50000;	
	
  ADRC_Yaw_Speed_Controller.b0=0.001;//扰动补偿
	
  ADRC_Yaw_Speed_Controller.beta_0=0;//非线性组合
  ADRC_Yaw_Speed_Controller.beta_1=15;
  ADRC_Yaw_Speed_Controller.beta_2=0.8;
  ADRC_Yaw_Speed_Controller.N1=5;
  ADRC_Yaw_Speed_Controller.c=5;
	
  ADRC_Yaw_Speed_Controller.alpha1=0.95;
  ADRC_Yaw_Speed_Controller.alpha2=1.3;
  ADRC_Yaw_Speed_Controller.zeta=10;
  ADRC_Yaw_Speed_Controller.b=0;
}

//ADRC最速跟????分器TD，改进的算法fhan
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC状态跟????????
  x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新????
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟????分器速度超调????
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪??
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//跟新最速跟??状态量x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//跟新最速跟??状态量??分x2
}


//原点附近有连线性???的连续幂???函??
float Fal_ADRC(float e,float alpha,float zeta)
{
    int16_t s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}

/************扩张状态???测??********************/
//状态???测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;//状态?????

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//非线性函数，提取跟踪状态与当前状态?????
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

  /*************扩展状态量更新**********/
  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b*fhan_Input->u);
 //ESO估???状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}


/************非线性组??****************/
/*
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float Sy=0,Sa=0;//ADRC状态跟????????

  fhan_Input->h1=fhan_Input->N1*fhan_Input->h;

  d=fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
  a0=fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
  y=fhan_Input->e1+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;

  Sy=Fsg_ADRC(y,d);
  a=(a0+y-a2)*Sy+a2;
  Sa=Fsg_ADRC(a,d);
  fhan_Input->u0=-fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);

  //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

  //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
  //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪??
}
*/
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float temp_e2=0;
  temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}


void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC)
{
    /*??抗扰控制器???1??*/
    /********
        **
        **
        **
        **
        **
     ********/
      /*****
      安排过度过程，输入为期望给定??
      由TD跟踪??分器得到??
      过度期望信号x1，过度期望微分信号x2
      ******/
      Fhan_ADRC(fhan_Input,expect_ADRC);

    /*??抗扰控制器???2??*/
    /********
            *
            *
       ****
     *
     *
     ********/
      /************系统输出值为反???量，状态反馈，ESO扩张状态???测器的输入*********/
      fhan_Input->y=feedback_ADRC;
      /*****
      扩张状态???测??，得到反馈信号的扩张状态：
      1、状态信号z1??
      2、状态速度信号z2??
      3、状态加速度信号z3??
      其中z1、z2用于作为状态反馈与TD??分跟??器得到的x1,x2做差后，
      经过非线性函数映射，乘以beta系数后，
      组合得到??加入状态加速度估???扰动补偿的原???控制量u
      *********/
      ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，??前暂时未想到办法解决，未用到z3
    /*??抗扰控制器???3??*/
    /********
           **
         **
       **
         **
           **
     ********/
      /********状态?????反???率***/
      fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//状态积分项
      fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//状态偏????
      fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//状态微分项??
      /********线性组??*******/
     /*
      fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                    +fhan_Input->beta_1*fhan_Input->e1
                    +fhan_Input->beta_2*fhan_Input->e2;
     */
      Nolinear_Conbination_ADRC(fhan_Input);
      /**********扰动补偿*******/
      //fhan_Input->u=fhan_Input->u0
      //             -fhan_Input->z3/fhan_Input->b0;
      //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，??前不加入扰动补偿控制??
      fhan_Input->u=Constrain_Float(fhan_Input->u0,-29000,29000);
}

