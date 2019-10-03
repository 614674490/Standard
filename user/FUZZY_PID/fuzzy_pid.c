#include "fuzzy_pid.h"
#include "remote.h"
//fPID Chassis_Motor_FPID_1,Chassis_Motor_FPID_2,Chassis_Motor_FPID_3,Chassis_Motor_FPID_4,Chassis_Motor_FPID_rotate;
fPID PTZ_Motor_FPID_Position_Y,PTZ_Motor_FPID_Speed_Y,PTZ_Motor_FPID_Position_P,PTZ_Motor_FPID_Speed_P;
//fPID_Inc FPower_limit;
/**
  * @brief          ģ��PID������ʼ��
  * @param[in]      ��
  */
void FPID_Init(void)
{
//	Chassis_Motor_FPID_1.Kd=3.5;
//	Chassis_Motor_FPID_1.Ki=0.02;
//	Chassis_Motor_FPID_1.Kp=2;
//	Chassis_Motor_FPID_1.Fuzzy_Kd=0;
//	Chassis_Motor_FPID_1.Fuzzy_Ki=0;
//	Chassis_Motor_FPID_1.Fuzzy_Kp=0;
//	Chassis_Motor_FPID_1.e=0;             //���
//	Chassis_Motor_FPID_1.ec=0;            //���仯��
//	Chassis_Motor_FPID_1.sum_e=0;         //�ۼ����
//	Chassis_Motor_FPID_1.last_e=0;        //�ϴ����
//	Chassis_Motor_FPID_1.factor.Ke=1;
//	Chassis_Motor_FPID_1.factor.Kec=1;
//	Chassis_Motor_FPID_1.factor.Ku_Kp=1.9f*levelInterval;  // 3/1.8=5/3
//	Chassis_Motor_FPID_1.factor.Ku_Ki=180*levelInterval;  // 3/150=0.02
//	Chassis_Motor_FPID_1.factor.Ku_Kd=1.5f*levelInterval;  // 3/1.8=5/3
//	
//	Chassis_Motor_FPID_2.Kd=3.2;
//	Chassis_Motor_FPID_2.Ki=0.02;
//	Chassis_Motor_FPID_2.Kp=2.2;
//	Chassis_Motor_FPID_2.Fuzzy_Kd=0;
//	Chassis_Motor_FPID_2.Fuzzy_Ki=0;
//	Chassis_Motor_FPID_2.Fuzzy_Kp=0;
//	Chassis_Motor_FPID_2.e=0;
//	Chassis_Motor_FPID_2.ec=0;
//	Chassis_Motor_FPID_2.sum_e=0;
//	Chassis_Motor_FPID_2.last_e=0;
//	Chassis_Motor_FPID_2.factor.Ke=1;
//	Chassis_Motor_FPID_2.factor.Kec=1;
//	Chassis_Motor_FPID_2.factor.Ku_Kp=2.0f*levelInterval;
//	Chassis_Motor_FPID_2.factor.Ku_Ki=180*levelInterval;
//	Chassis_Motor_FPID_2.factor.Ku_Kd=1.0f*levelInterval;
//	
//	Chassis_Motor_FPID_3.Kd=3.5;
//	Chassis_Motor_FPID_3.Ki=0.02;
//	Chassis_Motor_FPID_3.Kp=2;
//	Chassis_Motor_FPID_3.Fuzzy_Kd=0;
//	Chassis_Motor_FPID_3.Fuzzy_Ki=0;
//	Chassis_Motor_FPID_3.Fuzzy_Kp=0;
//	Chassis_Motor_FPID_3.e=0;
//	Chassis_Motor_FPID_3.ec=0;
//	Chassis_Motor_FPID_3.sum_e=0;
//	Chassis_Motor_FPID_3.last_e=0;
//	Chassis_Motor_FPID_3.factor.Ke=1;
//	Chassis_Motor_FPID_3.factor.Kec=1;
//	Chassis_Motor_FPID_3.factor.Ku_Kp=1.85f*levelInterval;
//	Chassis_Motor_FPID_3.factor.Ku_Ki=180*levelInterval;
//	Chassis_Motor_FPID_3.factor.Ku_Kd=1.8f*levelInterval;
//	
//	Chassis_Motor_FPID_4.Kd=3.3;
//	Chassis_Motor_FPID_4.Ki=0.02;
//	Chassis_Motor_FPID_4.Kp=2.1;
//	Chassis_Motor_FPID_4.Fuzzy_Kd=0;
//	Chassis_Motor_FPID_4.Fuzzy_Ki=0;
//	Chassis_Motor_FPID_4.Fuzzy_Kp=0;
//	Chassis_Motor_FPID_4.e=0;
//	Chassis_Motor_FPID_4.ec=0;
//	Chassis_Motor_FPID_4.sum_e=0;
//	Chassis_Motor_FPID_4.last_e=0;
//	Chassis_Motor_FPID_4.factor.Ke=1;
//	Chassis_Motor_FPID_4.factor.Kec=1;
//	Chassis_Motor_FPID_4.factor.Ku_Kp=2.0f*levelInterval;
//	Chassis_Motor_FPID_4.factor.Ku_Ki=180*levelInterval;
//	Chassis_Motor_FPID_4.factor.Ku_Kd=1.8f*levelInterval;
//	
//	Chassis_Motor_FPID_rotate.Kd=1;
//	Chassis_Motor_FPID_rotate.Ki=0.0135;
//	Chassis_Motor_FPID_rotate.Kp=25;
//	Chassis_Motor_FPID_rotate.Fuzzy_Kd=0;
//	Chassis_Motor_FPID_rotate.Fuzzy_Ki=0;
//	Chassis_Motor_FPID_rotate.Fuzzy_Kp=0;
//	Chassis_Motor_FPID_rotate.e=0;
//	Chassis_Motor_FPID_rotate.ec=0;
//	Chassis_Motor_FPID_rotate.sum_e=0;
//	Chassis_Motor_FPID_rotate.last_e=0;
//	Chassis_Motor_FPID_rotate.factor.Ke=1;
//	Chassis_Motor_FPID_rotate.factor.Kec=1;
//	Chassis_Motor_FPID_rotate.factor.Ku_Kp=1.5f*levelInterval;
//	Chassis_Motor_FPID_rotate.factor.Ku_Ki=300*levelInterval;
//	Chassis_Motor_FPID_rotate.factor.Ku_Kd=6.0f*levelInterval;
	
	//8 0 25
	PTZ_Motor_FPID_Position_Y.Kd=5;
	PTZ_Motor_FPID_Position_Y.Ki=0;
	PTZ_Motor_FPID_Position_Y.Kp=25;
	PTZ_Motor_FPID_Position_Y.Fuzzy_Kd=0;
	PTZ_Motor_FPID_Position_Y.Fuzzy_Ki=0;
	PTZ_Motor_FPID_Position_Y.Fuzzy_Kp=0;
	PTZ_Motor_FPID_Position_Y.e=0;             //���
	PTZ_Motor_FPID_Position_Y.ec=0;            //���仯��
	PTZ_Motor_FPID_Position_Y.sum_e=0;         //�ۼ����
	PTZ_Motor_FPID_Position_Y.last_e=0;        //�ϴ����
	PTZ_Motor_FPID_Position_Y.factor.Ke=1;
	PTZ_Motor_FPID_Position_Y.factor.Kec=1;
	PTZ_Motor_FPID_Position_Y.factor.Ku_Kp=0.6f*levelInterval;                 //[-5 5]
	PTZ_Motor_FPID_Position_Y.factor.Ku_Ki=100*levelInterval;   //Y����̨ȥ������
	PTZ_Motor_FPID_Position_Y.factor.Ku_Kd=1.5f*levelInterval;
	//1.25 0 20
	PTZ_Motor_FPID_Speed_Y.Kd=5;
	PTZ_Motor_FPID_Speed_Y.Ki=0;
	PTZ_Motor_FPID_Speed_Y.Kp=20;
	PTZ_Motor_FPID_Speed_Y.Fuzzy_Kd=0;
	PTZ_Motor_FPID_Speed_Y.Fuzzy_Ki=0;
	PTZ_Motor_FPID_Speed_Y.Fuzzy_Kp=0;
	PTZ_Motor_FPID_Speed_Y.e=0;             //���
	PTZ_Motor_FPID_Speed_Y.ec=0;            //���仯��
	PTZ_Motor_FPID_Speed_Y.sum_e=0;         //�ۼ����
	PTZ_Motor_FPID_Speed_Y.last_e=0;        //�ϴ����
	PTZ_Motor_FPID_Speed_Y.factor.Ke=1.1;    //���������� ��ʹ��ȷ���½� ���������
	PTZ_Motor_FPID_Speed_Y.factor.Kec=1.1;
	PTZ_Motor_FPID_Speed_Y.factor.Ku_Kp=3.0f*levelInterval;                   //[-12 12]
	PTZ_Motor_FPID_Speed_Y.factor.Ku_Ki=100*levelInterval;     //Y����̨ȥ������
	PTZ_Motor_FPID_Speed_Y.factor.Ku_Kd=1.5f*levelInterval;
	
	PTZ_Motor_FPID_Position_P.Kd=1.0;
	PTZ_Motor_FPID_Position_P.Ki=0.1;
	PTZ_Motor_FPID_Position_P.Kp=30;
	PTZ_Motor_FPID_Position_P.Fuzzy_Kd=0;
	PTZ_Motor_FPID_Position_P.Fuzzy_Ki=0;
	PTZ_Motor_FPID_Position_P.Fuzzy_Kp=0;
	PTZ_Motor_FPID_Position_P.e=0;             //���
	PTZ_Motor_FPID_Position_P.ec=0;            //���仯��
	PTZ_Motor_FPID_Position_P.sum_e=0;         //�ۼ����
	PTZ_Motor_FPID_Position_P.last_e=0;        //�ϴ����
	PTZ_Motor_FPID_Position_P.factor.Ke=1;
	PTZ_Motor_FPID_Position_P.factor.Kec=1;
	PTZ_Motor_FPID_Position_P.factor.Ku_Kp=3.0f*levelInterval;                      //[-12 12]
	PTZ_Motor_FPID_Position_P.factor.Ku_Ki=100*levelInterval;       //P����̨ȥ������
	PTZ_Motor_FPID_Position_P.factor.Ku_Kd=6.0f*levelInterval;
	
	PTZ_Motor_FPID_Speed_P.Kd=0;
	PTZ_Motor_FPID_Speed_P.Ki=0;
	PTZ_Motor_FPID_Speed_P.Kp=20;
	PTZ_Motor_FPID_Speed_P.Fuzzy_Kd=0;
	PTZ_Motor_FPID_Speed_P.Fuzzy_Ki=0;
	PTZ_Motor_FPID_Speed_P.Fuzzy_Kp=0;
	PTZ_Motor_FPID_Speed_P.e=0;             //���
	PTZ_Motor_FPID_Speed_P.ec=0;            //���仯��
	PTZ_Motor_FPID_Speed_P.sum_e=0;         //�ۼ����
	PTZ_Motor_FPID_Speed_P.last_e=0;        //�ϴ����
	PTZ_Motor_FPID_Speed_P.factor.Ke=1.1;
	PTZ_Motor_FPID_Speed_P.factor.Kec=1.1;
	PTZ_Motor_FPID_Speed_P.factor.Ku_Kp=3.0f*levelInterval;                   //[-12 12]
	PTZ_Motor_FPID_Speed_P.factor.Ku_Ki=100*levelInterval;          //P����̨ȥ������
	PTZ_Motor_FPID_Speed_P.factor.Ku_Kd=1.5f*levelInterval;
	
//	FPower_limit.Kd=0;
//	FPower_limit.Ki=1.2;
//	FPower_limit.Kp=0.5;
//	FPower_limit.Fuzzy_Kd=0;
//	FPower_limit.Fuzzy_Ki=0;
//	FPower_limit.Fuzzy_Kp=0;
//	FPower_limit.e=0;             //���
//	FPower_limit.ec=0;            //���仯��
//	FPower_limit.per_e=0;         //�ۼ����
//	FPower_limit.last_e=0;        //�ϴ����
//	FPower_limit.factor.Ke=1;
//	FPower_limit.factor.Kec=1;
//	FPower_limit.factor.Ku_Kp=6.0f*levelInterval;  // 3/1.8=5/3
//	FPower_limit.factor.Ku_Ki=30*levelInterval;  // 3/150=0.02
//	FPower_limit.factor.Ku_Kd=3.0f*levelInterval;  // 3/1.8=5/3
	

}

/**
  * @brief          ��ѯģ�������
  * @param[in]      �½Ǳ� ��������� ���ӽṹ��
  */
float DeFuzzy(int eLevel,int ecLevel,u8 ID_item,struct Factor *factor)
{
 
		switch(ID_item)
		{
		 
				case ID_dKp:
				return fuzzyRuleKp[ecLevel+3][eLevel+3] / factor->Ku_Kp;
				 
				case ID_dKi:
				return fuzzyRuleKi[ecLevel+3][eLevel+3] / factor->Ku_Ki;
				 
				case ID_dKd:
				return fuzzyRuleKd[ecLevel+3][eLevel+3] / factor->Ku_Kd;
				 
				default:
				return 0;
		 
		}
 
}
/**
  * @brief          Kp Ki Kd ��ģ������
  * @param[in]      PID���ڶ���
  * @other            E��[-15,15] EC��[-3,3] U��[-4800,4800]
  */
void Fuzzy(fPID *motor)
{

	int eLeftIndex,eRightIndex,ecLeftIndex,ecRightIndex;
	float eLeftMs,eRightMs,ecLeftMs,ecRightMs;            //������  ���Ǻ�����
	
	motor->e /= motor->factor.Ke;
	motor->ec /= motor->factor.Kec;
	
	eLeftIndex = (motor->e/levelInterval)>3.0f?3.0f:(motor->e/levelInterval)<-3.0f?-4.0f:(motor->e/levelInterval)>0.0f?(int)(motor->e/levelInterval):(int)(motor->e/levelInterval)-1;
	eRightIndex = eLeftIndex + 1;
	 
	eLeftMs = eLeftIndex<-3.0f?0.0f:eLeftIndex==3.0f?1.0f:eRightIndex-motor->e/levelInterval;
	eRightMs = eRightIndex>3.0f?0.0f:eRightIndex==-3.0f?1.0f:motor->e/levelInterval-eLeftIndex;
	
	ecLeftIndex = (motor->ec/levelInterval)>3.0f?3.0f:(motor->ec/levelInterval)<-3.0f?-4.0f:(motor->ec/levelInterval)>0.0f?(int)(motor->ec/levelInterval):(int)(motor->ec/levelInterval)-1;
	ecRightIndex = ecLeftIndex + 1;
	 
	ecLeftMs = ecLeftIndex<-3.0f?0.0f:ecLeftIndex==3.0f?1.0f:ecRightIndex-motor->ec/levelInterval;
	ecRightMs = ecRightIndex>3.0f?0.0f:ecRightIndex==-3.0f?1.0f:motor->ec/levelInterval-ecLeftIndex;
	

	/*************************************��ģ��*************************************/
	motor->Fuzzy_Kp = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKp,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKp,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKp,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKp,&motor->factor));
			
	motor->Fuzzy_Ki = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKi,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKi,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKi,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKi,&motor->factor));
			
	motor->Fuzzy_Kd = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKd,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKd,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKd,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKd,&motor->factor));

}

/**
  * @brief          ģ��PID������� λ��ʽ
  * @param[in]      ����ֵ ʵ��ֵ PID���ƶ���
  */
void Fuzzy_PidControler(float tar, float cur,fPID *motor)
{               
	motor->last_e =motor->e;	             //�����ϴ����
	motor->e = tar-cur;                    //����ֵ-��ǰֵ
	motor->ec = motor->e-motor->last_e;    //�������仯��
  motor->sum_e +=motor->e;               //�����ۼ����
	VAL_LIMIT(motor->sum_e,-10000,10000);  //�����޷�
	Fuzzy(motor);                          //ģ�����Ƶ���  kp��ki��kd
	if(motor==&PTZ_Motor_FPID_Position_Y||motor==&PTZ_Motor_FPID_Speed_Y||motor==&PTZ_Motor_FPID_Speed_P||motor==&PTZ_Motor_FPID_Position_P)
		motor->Fuzzy_Ki=0;  //��̨ȥ��������

  motor->pid_out=(motor->Kp+motor->Fuzzy_Kp)*motor->e +(motor->Kd+motor->Fuzzy_Kd)*motor->ec + (motor->Ki+motor->Fuzzy_Ki)*motor->sum_e;
}


/**
  * @brief          Kp Ki Kd ��ģ������ ����ʽ
  * @param[in]      PID���ڶ���
  */
void Fuzzy_Inc(fPID_Inc *motor)
{

	int eLeftIndex,eRightIndex,ecLeftIndex,ecRightIndex;
	float eLeftMs,eRightMs,ecLeftMs,ecRightMs;            //������
	
	eLeftIndex = (motor->e/levelInterval)>3.0f?3.0f:(motor->e/levelInterval)<-3.0f?-4.0f:(motor->e/levelInterval)>0.0f?(int)(motor->e/levelInterval):(int)(motor->e/levelInterval)-1;
	eRightIndex = eLeftIndex + 1;
	 
	eLeftMs = eLeftIndex<-3.0f?0.0f:eLeftIndex==3.0f?1.0f:eRightIndex-motor->e/levelInterval;
	eRightMs = eRightIndex>3.0f?0.0f:eRightIndex==-3.0f?1.0f:motor->e/levelInterval-eLeftIndex;
	
	ecLeftIndex = (motor->ec/levelInterval)>3.0f?3.0f:(motor->ec/levelInterval)<-3.0f?-4.0f:(motor->ec/levelInterval)>0.0f?(int)(motor->ec/levelInterval):(int)(motor->ec/levelInterval)-1;
	ecRightIndex = ecLeftIndex + 1;
	 
	ecLeftMs = ecLeftIndex<-3.0f?0.0f:ecLeftIndex==3.0f?1.0f:ecRightIndex-motor->ec/levelInterval;
	ecRightMs = ecRightIndex>3.0f?0.0f:ecRightIndex==-3.0f?1.0f:motor->ec/levelInterval-ecLeftIndex;
	

	/*************************************��ģ��*************************************/
	motor->Fuzzy_Kp = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKp,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKp,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKp,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKp,&motor->factor));
			
	motor->Fuzzy_Ki = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKi,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKi,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKi,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKi,&motor->factor));
			
	motor->Fuzzy_Kd = (eLeftMs * ecLeftMs * DeFuzzy(eLeftIndex, ecLeftIndex, ID_dKd,&motor->factor)
			+ eLeftMs * ecRightMs * DeFuzzy(eLeftIndex, ecRightIndex, ID_dKd,&motor->factor)
			+ eRightMs * ecLeftMs * DeFuzzy(eRightIndex, ecLeftIndex, ID_dKd,&motor->factor)
			+ eRightMs * ecRightMs * DeFuzzy(eRightIndex, ecRightIndex, ID_dKd,&motor->factor));

}

/**
  * @brief          ģ��PID������� ����ʽ
  * @param[in]      ����ֵ ʵ��ֵ
  */
int FuzzyPID_Inc(float tar,float cur,fPID_Inc *motor) 
{
     motor->e=tar-cur;
     motor->ec=motor->e-motor->last_e;
	   Fuzzy_Inc(motor);
	   motor->pid_out=(motor->Kp+motor->Fuzzy_Kp)*motor->ec+(motor->Ki+motor->Fuzzy_Ki)*motor->e+
	                          (motor->Kd+motor->Fuzzy_Kd)*(motor->e-2*motor->last_e+motor->per_e);
	   motor->per_e=motor->last_e;  //����ǰһ�ε�ֵ
     motor->last_e= motor->e;    //������һ�ε�ֵ
	   return motor->pid_out;
}

