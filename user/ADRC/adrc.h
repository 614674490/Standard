#ifndef _ADRC_H_
#define _ADRC_H_

#include "sys.h"
#include "math.h"

typedef enum  //ö�٣�����ֵ��0��ʼ���μ�1
{
    Yaw_ADRC_CONTROL=0,
    PITCH_ADRC_CONTROL,
}ADRC_ID_e;

typedef struct
{
/*****���Ź��ȹ���*******/
float x1;//����΢����״̬��
float x2;//����΢����״̬��΢����
float r;//ʱ��߶�
float h;//ADRCϵͳ����ʱ��
uint16_t N0;//����΢��������ٶȳ���h0=N*h

float h0;
float fh;//����΢�ּ��ٶȸ�����
/*****����״̬�۲���*******/
/******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
float z1;
float z2;
float z3;//���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
float e;//ϵͳ״̬���
float y;//ϵͳ�����
float fe;
float fe1;
float beta_01;
float beta_02;
float beta_03;
float b;


/**********ϵͳ״̬������*********/
float e0;//״̬��������
float e1;//״̬ƫ��
float e2;//״̬��΢����
float u0;//���������ϵͳ���
int u;//���Ŷ�����������
int filter_u;
float b0;//�Ŷ�����

/*********��һ�������ʽ*********/
float beta_0;//����
float beta_1;//��������ϲ���
float beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
/*********�ڶ��������ʽ*********/
float alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
float alpha2;//0<alpha1<1<alpha2
float zeta;//���Զε����䳤��
/*********�����������ʽ*********/
float h1;//u0=-fhan(e1,e2,r,h1);
uint16_t N1;//����΢��������ٶȳ���h0=N*h
/*********�����������ʽ*********/
float c;//u0=-fhan(e1,c*e2*e2,r,h1);


}Fhan_Data;

void ADRC_Init(void);
//void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2);
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);
void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback);

extern Fhan_Data ADRC_Yaw_Controller,ADRC_Yaw_Speed_Controller,ADRC_Pitch_Controller;
#endif

