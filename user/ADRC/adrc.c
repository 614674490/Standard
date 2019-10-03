#include "ADRC.h"
#include "math.h"

Fhan_Data ADRC_Yaw_Controller;
Fhan_Data ADRC_Yaw_Speed_Controller;
Fhan_Data ADRC_Pitch_Controller;

#define ABS(X)  (((X)>0)?(X):-(X))

const float ADRC_Unit[3][16]=
{
/*TD����΢����   �Ľ�����TD,h0=N*h      ����״̬�۲���ESO           �Ŷ�����     ���������*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      50000,     0.001,    0.002,   200,     1.5,        5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0},
};

/*
*rԽС�������ٶȺ͸��ټ��ٶ�Խƽ�������ͺ���Խ��Խ���෴
*hԽ�󣬸��ټ��ٶȲ���ԽС����ֵԽС
*NԽ�󣬸����ٶȺͼ��ٶȶ���ƽ������ֵҲ��Ӧ��С
*
*beta_01ԽС���������������Խ��Խ��Խƽ������������ͺ�
*beta_02Խ��z2,z3������(��Ƶ)Խ��ԽС��z1,z2,z3�������Ҳ�����z2,z3��Ϊ����
*beta_03Խ��z3������Խ��ԽСԽƽ��
*b0
*
*beta_0����δʹ��
*bata_1Խ������Խ��
*beta_2Խ����ӦԽ�죬ʵʱ��Խ�ã����ȶ��Ա��
*
*alpha1ԽС��ʵʱ��Խ�ã����ȶ��Ժ����涼��Խ���෴
*alpha2ԽС��Խƽ����Խ������Խ�󣬵�ʵʱ�Ժ����涼���
*zetaԽС��Խƽ������ʵʱ�Բ��ã�Խ��ʵʱ��Խ�ã���������
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
/*****λ�û�*****/
	ADRC_Yaw_Controller.r=300000;//TD����΢����
  ADRC_Yaw_Controller.h=0.002;
  ADRC_Yaw_Controller.N0=3;	
	
  ADRC_Yaw_Controller.beta_01=300;//����״̬�۲���ESO
  ADRC_Yaw_Controller.beta_02=4000;
  ADRC_Yaw_Controller.beta_03=50000;	
	
  ADRC_Yaw_Controller.b0=0.001;//�Ŷ�����
	
  ADRC_Yaw_Controller.beta_0=0.002;//���������
  ADRC_Yaw_Controller.beta_1=250;
  ADRC_Yaw_Controller.beta_2=20;
  ADRC_Yaw_Controller.N1=5;
  ADRC_Yaw_Controller.c=5;
	
  ADRC_Yaw_Controller.alpha1=0.40;/*****/
  ADRC_Yaw_Controller.alpha2=0.60;
  ADRC_Yaw_Controller.zeta=8;
  ADRC_Yaw_Controller.b=0;
	
/*********�ٶȻ�**********/	
	ADRC_Yaw_Speed_Controller.r=300000;//TD����΢����
  ADRC_Yaw_Speed_Controller.h=0.005;
  ADRC_Yaw_Speed_Controller.N0=3;	
	
  ADRC_Yaw_Speed_Controller.beta_01=300;//����״̬�۲���ESO
  ADRC_Yaw_Speed_Controller.beta_02=4000;
  ADRC_Yaw_Speed_Controller.beta_03=50000;	
	
  ADRC_Yaw_Speed_Controller.b0=0.001;//�Ŷ�����
	
  ADRC_Yaw_Speed_Controller.beta_0=0;//���������
  ADRC_Yaw_Speed_Controller.beta_1=15;
  ADRC_Yaw_Speed_Controller.beta_2=0.8;
  ADRC_Yaw_Speed_Controller.N1=5;
  ADRC_Yaw_Speed_Controller.c=5;
	
  ADRC_Yaw_Speed_Controller.alpha1=0.95;
  ADRC_Yaw_Speed_Controller.alpha2=1.3;
  ADRC_Yaw_Speed_Controller.zeta=10;
  ADRC_Yaw_Speed_Controller.b=0;
}

//ADRC���ٸ�????����TD���Ľ����㷨fhan
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//����ADRC���ȹ���
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC״̬��????????
  x1_delta=fhan_Input->x1-expect_ADRC;//��x1-v(k)���x1�õ���ɢ����????
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//��h0���h��������ٸ�????�����ٶȳ���????
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//�õ�����΢�ּ��ٶȸ���??
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//�������ٸ�??״̬��x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//�������ٸ�??״̬��??��x2
}


//ԭ�㸽����������???��������???��??
float Fal_ADRC(float e,float alpha,float zeta)
{
    int16_t s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}

/************����״̬???��??********************/
//״̬???��������beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;//״̬?????

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//�����Ժ�������ȡ����״̬�뵱ǰ״̬?????
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

  /*************��չ״̬������**********/
  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b*fhan_Input->u);
 //ESO��???״̬���ٶ��źţ������Ŷ���������ͳMEMS������Ư�ƽϴ󣬹��ƻ����Ư��
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}


/************��������??****************/
/*
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float Sy=0,Sa=0;//ADRC״̬��????????

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
  //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//�õ�����΢�ּ��ٶȸ���??
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
    /*??���ſ�����???1??*/
    /********
        **
        **
        **
        **
        **
     ********/
      /*****
      ���Ź��ȹ��̣�����Ϊ��������??
      ��TD����??�����õ�??
      ���������ź�x1����������΢���ź�x2
      ******/
      Fhan_ADRC(fhan_Input,expect_ADRC);

    /*??���ſ�����???2??*/
    /********
            *
            *
       ****
     *
     *
     ********/
      /************ϵͳ���ֵΪ��???����״̬������ESO����״̬???����������*********/
      fhan_Input->y=feedback_ADRC;
      /*****
      ����״̬???��??���õ������źŵ�����״̬��
      1��״̬�ź�z1??
      2��״̬�ٶ��ź�z2??
      3��״̬���ٶ��ź�z3??
      ����z1��z2������Ϊ״̬������TD??�ָ�??���õ���x1,x2�����
      ���������Ժ���ӳ�䣬����betaϵ����
      ��ϵõ�??����״̬���ٶȹ�???�Ŷ�������ԭ???������u
      *********/
      ESO_ADRC(fhan_Input);//�ͳɱ�MEMS�����Ư�ƣ���չ������z3�����Ư�ƣ�??ǰ��ʱδ�뵽�취�����δ�õ�z3
    /*??���ſ�����???3??*/
    /********
           **
         **
       **
         **
           **
     ********/
      /********״̬?????��???��***/
      fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//״̬������
      fhan_Input->e1=fhan_Input->x1-fhan_Input->z1;//״̬ƫ????
      fhan_Input->e2=fhan_Input->x2-fhan_Input->z2;//״̬΢����??
      /********������??*******/
     /*
      fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                    +fhan_Input->beta_1*fhan_Input->e1
                    +fhan_Input->beta_2*fhan_Input->e2;
     */
      Nolinear_Conbination_ADRC(fhan_Input);
      /**********�Ŷ�����*******/
      //fhan_Input->u=fhan_Input->u0
      //             -fhan_Input->z3/fhan_Input->b0;
      //����MEMS������Ư�ƱȽ����أ���beta_03ȡֵ�Ƚϴ�ʱ����ʱ��z3Ư�ƱȽϴ�??ǰ�������Ŷ���������??
      fhan_Input->u=Constrain_Float(fhan_Input->u0,-29000,29000);
}

