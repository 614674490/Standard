#ifndef  _CONFIG_H
#define  _CONFIG_H
#define INFANTRY                     3     // ������� 
#define DEBUG_MODE                   1    //1 ��������ģʽ��ֻ����Ħ�����벦�������   0 ����������ģʽ 
#define CHASSIS_FEEDBACK             2     //1���ò���ϵ ͳ    2 ���õ�����/3508ת�ص���  0 �����ù��ʱջ�
#define SHOOT_FEEDBACK               0     //1���ò���ϵͳ(������ȼ�)    2 ��ʱ������  3����ϵͳ(ֻ�Ƕ�ȡ�ȼ�)  0 �����������ջ�
#define Client_Robot_Iteractive      1     //�ͻ��� ������ͨ��
#define REFEREE_SYSTEM               0     //1:��������ϵͳ 0���رղ���ϵͳ

#define DIAL_MOTOR_OLD               0
#define DIAL_MOTOR_NEW               1
#define FRICTION_420S_STATUS         0        //�Ƿ�ʹ��420S���

#define ACCELERA_TIME               50000     //����б��ʱ��
#define DECELERATE_TIME             15000    //����б��ʱ��
#define PTZ_MODE                    0        //Ĭ�ϲ�����ģ��	��̨ģʽ
#define FRICTION_ON_TIME            1000    //Ħ���ֿ���б��ʱ��
#define FRICTION_OFF_TIME           1000    //Ħ���ֹر�б��ʱ��

/*                       Ħ���ֵ�������ֵ                               */
#define Friction_420S_MIN_Value               900
#define Friction_420S_MAX_Value               2000
#define Friction_420S_Period                  20   //420S��������
#define Friction_2312_Domain_Value            1000

#define PRESS_LONG_TIME              100      //��������������ʱ�� ���������嵯

#define PTZ_LOCK_ANGLE               3.0f    //��̨�����Ƕ�
#define CONTINUE_DIAL_POSITION       200
#define REDIAL_BULLET_POSITION       1000
#define REDIAL_BULLET_SPEED          5      //����������ٶȼ���ٶ�
#define KEY_PRESS_DELAY              30      //������Ӧ��ʱʱ��
#define FRICITION_CLOSE_DELAY        75      //Ħ���ֹر���ʱʱ��

#define POWER_VOLTAGE                24.0f   //��Դ��ѹ
#define POWER_LIMIT_UP               80     //�����������
#define CURRENT_OFFSET               1000.0f //��������ϵ��
#define VOLTAGE_OFFSET               1000.0f //��ѹ����ϵ��
/*************************** ������λ��״̬ ***********************************/
#define MPU_NORMAL              0
#define MPU_UPSIDE_DOWN         1
#define MPU_USART               2
/*************************** CANʹ��״̬ ***********************************/
#define CAN1_Chassis               0
#define CAN2_Chassis               1
/*************************** ���ʼ�����ز��� ***********************************/
#define POWER_CHECK_TIME        1.0f        //����ͨѶ����  0.5s
#define POWER_BUFFER_LIMIT      60          //��󻺳����� (�ǲ���ϵͳ ����PID��������) 60J
#define POWER_BUFFER_THRESHOLD  25          //���ʻ�������Σ��ֵ 10J  Ԥ����
#define POWER_BUFFER_DANGER     10          //���ʻ�������Σ��ֵ ��ǰ��
#define RT_CAP_TAL_THRESHOLD    12.0f       //Ť������С����ʱ�ĳ������ݵ�����ֵ
#define CAP_TAL_THRESHOLD       10.5f       //��ͨ״̬�µĳ������ݵ�����ֵ
#define CAP_TAL_BUFFER          3.0f        //�������ݵĵ������� ��ֹ�������ݵ������л�Ƶ�ʹ���
/*************************** ���̵����ز��� ***********************************/
#define M3510                      0
#define M3508                      1
#define FAST_ROTATE_K              3.7f
#define NORMAL_ROTATE_K            3.2f
#if INFANTRY == 3               //�������Ų�����
/**
 *@tip ��� �������� ��С����
 */
/**********************   �ӵ�λ���������޸ġ�  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //һ�ŵ����ӵ�������    ������
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //һ�Ÿ����ӵ�������     ������
		#define 		 FAST_THREEBULLET_HEAT        66   //���������ӵ�������       ��������
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //�ɲ������
		
		#define      DIAL_ONE_POSITION       4320 //һ���ӵ���λ��
    #define      DIAL_THREE_POSITION     12960  //�����ӵ���λ�� 
		#define      DIAL_BACK_POSITION      1800 //������λ��  
		
		#define SONEBULLET_DISTANCE         1200    //һ���ӵ�����Ƶ�������ж�(һ��)
		#define MONEBULLET_DISTANCE         1800    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         3000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       800     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       1000    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       2000    //�����ӵ�����Ƶ�������ж�(����)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //һ���ӵ���λ�� 
    #define      DIAL_THREE_POSITION     4860  //�����ӵ���λ��  
		#define      DIAL_BACK_POSITION      1000   //������λ��
		
		#define SONEBULLET_DISTANCE         500    //һ���ӵ�����Ƶ�������ж�(һ��)   240 360 480
		#define MONEBULLET_DISTANCE         750    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         1000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       200     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       300    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       400    //�����ӵ�����Ƶ�������ж�(����)
		
		#endif
		 //��ѹ��10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //Ħ���ֹر�
		#define      NORMAL_FRICITION_ON_SPEED   1950   //��ͨĦ���ֻ����ٶ� ��Vave=26m/s�� >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //�е�Ħ���ֻ����ٶ� (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //�����Ħ���ֻ����ٶ�(16m/s)	
/***********************    ���ʻ������޸ġ�   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //����ģʽ�µĹ���
		#define      POWER_NORMAL_LIMIT_VALUE  75       //����ģʽ�µĹ���
		#define      POWER_FAST_LIMIT_VALUE    90       //����ģʽ�µĹ���
		#define      POWER_INCREASE            5        //����������ֵ
		#define      POWER_OFFSET              4.6     //���̾�ֹʱ��ʵ�ʹ���
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//����ģʽ��KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//����ģʽ��KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //Ħ���ֿ����ٶ�
		
		#define POWER_KP                   0.5f      //���ʻ�KPֵ
    #define POWER_KI                   1.2f     //���ʻ�KIֵ����ɼ���������
    #define POWER_KD                   0.0f     //���ʻ�KDֵ
/***********************    ��̨�����޸ġ�   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch����
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch����
		#define      YAW_INITIAL_VALUE       1376     //Yaw���ʼֵ  
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID�޷������޸ġ�   ************************/	  //�����ǵĳ�ʼƫ��
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //�������ر����߻�����������Ϊ0ʱ �Ե���������о����޷�
#define POWER_MAX_LIMIT_LEVEL2   1000    //���򳬹��ʿ�Ѫ
#define MPU6500_TEMP_PWM_MAX  100

/***********************    ���PID�����޸ġ�   ************************/
#define CHASSIS_MOTOR_KP 6.0f
#define CHASSIS_MOTOR_KI 0.02f
#define CHASSIS_MOTOR_KD 2.0f

#define HIT_POSITION_KP 1.9f
#define HIT_POSITION_KI 0.0f
#define HIT_POSITION_KD 0.1f
#define HIT_SPEED_KP 3.0f
#define HIT_SPEED_KI 0.0f
#define HIT_SPEED_KD 1.0f

#define CHASSIS_MOTOR_ROTATE_KP 55.0f
#define CHASSIS_MOTOR_ROTATE_KI 0.00f
#define CHASSIS_MOTOR_ROTATE_KD 2500.0f

#define PTZ_MOTOR_YAW_POSITION_KP 11.0f
#define PTZ_MOTOR_YAW_POSITION_KI 0.0f
#define PTZ_MOTOR_YAW_POSITION_KD 5.0f
#define PTZ_MOTOR_YAW_SPEED_KP 110.0f
#define PTZ_MOTOR_YAW_SPEED_KI 0.10f
#define PTZ_MOTOR_YAW_SPEED_KD 0.0f

//30 0 3  35 0.15 0
#define PTZ_MOTOR_PITCH_POSITION_KP 40.0f
#define PTZ_MOTOR_PITCH_POSITION_KI 0.05f
#define PTZ_MOTOR_PITCH_POSITION_KD 3.0f
#define PTZ_MOTOR_PITCH_SPEED_KP 20.0f
#define PTZ_MOTOR_PITCH_SPEED_KI 0.0f
#define PTZ_MOTOR_PITCH_SPEED_KD 3.0f


/***********************    ��������޸ġ�   ************************/			
#define Magaine_ON_PWM                        25
#define Magaine_OFF_PWM                       5

/***********************    �����ǲ����޸ġ�   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN�����޸ġ�   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    ���̵�������޸ġ�   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif


#if INFANTRY == 4               //�����ĺŲ�����
/**
 *@tip ���� �������� С����
 */
/**********************   �ӵ�λ���������޸ġ�  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //һ�ŵ����ӵ�������    ������
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //һ�Ÿ����ӵ�������     ������
		#define 		 FAST_THREEBULLET_HEAT        66   //���������ӵ�������       ��������
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //�ɲ������
		
		#define      DIAL_ONE_POSITION       4320 //һ���ӵ���λ��
    #define      DIAL_THREE_POSITION     12960  //�����ӵ���λ�� 
		#define      DIAL_BACK_POSITION      1800 //������λ��  
		
		#define SONEBULLET_DISTANCE         1200    //һ���ӵ�����Ƶ�������ж�(һ��)
		#define MONEBULLET_DISTANCE         1800    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         3000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       800     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       1000    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       2000    //�����ӵ�����Ƶ�������ж�(����)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //һ���ӵ���λ�� 
    #define      DIAL_THREE_POSITION     4860  //�����ӵ���λ��  
		#define      DIAL_BACK_POSITION      1000   //������λ��
		
		#define SONEBULLET_DISTANCE         500    //һ���ӵ�����Ƶ�������ж�(һ��)   240 360 480
		#define MONEBULLET_DISTANCE         750    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         1000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       200     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       300    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       400    //�����ӵ�����Ƶ�������ж�(����)
		
		#endif
		 //��ѹ��10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //Ħ���ֹر�
		#define      NORMAL_FRICITION_ON_SPEED   1950   //��ͨĦ���ֻ����ٶ� ��Vave=26m/s�� >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //�е�Ħ���ֻ����ٶ� (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //�����Ħ���ֻ����ٶ�(16m/s)	
/***********************    ���ʻ������޸ġ�   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //����ģʽ�µĹ���
		#define      POWER_MIDDLE_LIMIT_VALUE  75       //����ģʽ�µĹ���
		#define      POWER_FAST_LIMIT_VALUE    90       //����ģʽ�µĹ���
		#define      POWER_INCREASE            5        //����������ֵ
		#define      POWER_OFFSET              4.6     //���̾�ֹʱ��ʵ�ʹ���
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//����ģʽ��KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//����ģʽ��KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //Ħ���ֿ����ٶ�
		
		#define POWER_KP                   0.5f      //���ʻ�KPֵ
    #define POWER_KI                   1.2f     //���ʻ�KIֵ����ɼ���������
    #define POWER_KD                   0.0f     //���ʻ�KDֵ
/***********************    ��̨�����޸ġ�   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch����
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch����
		#define      YAW_UP_LIMIT           40.0f      //yaw����
		#define      YAW_DOWM_LIMIT        -40.0f     //yaw����
		#define      YAW_INITIAL_VALUE       1483 //Yaw���ʼֵ
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID�޷������޸ġ�   ************************/	  //�����ǵĳ�ʼƫ��
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //�������ر����߻�����������Ϊ0ʱ �Ե���������о����޷�
#define POWER_MAX_LIMIT_LEVEL2   1000    //���򳬹��ʿ�Ѫ
#define MPU6500_TEMP_PWM_MAX  100

/***********************    ���PID�����޸ġ�   ************************/
#define CHASSIS_MOTOR_KP 6.0f
#define CHASSIS_MOTOR_KI 0.02f
#define CHASSIS_MOTOR_KD 2.0f

#define HIT_POSITION_KP 1.9f
#define HIT_POSITION_KI 0.0f
#define HIT_POSITION_KD 0.1f
#define HIT_SPEED_KP 3.0f
#define HIT_SPEED_KI 0.0f
#define HIT_SPEED_KD 1.0f

#define CHASSIS_MOTOR_ROTATE_KP 55.0f
#define CHASSIS_MOTOR_ROTATE_KI 0.00f
#define CHASSIS_MOTOR_ROTATE_KD 2500.0f

#define PTZ_MOTOR_YAW_POSITION_KP 30.0f
#define PTZ_MOTOR_YAW_POSITION_KI 0.0f
#define PTZ_MOTOR_YAW_POSITION_KD 5.0f
#define PTZ_MOTOR_YAW_SPEED_KP 110.0f
#define PTZ_MOTOR_YAW_SPEED_KI 0.10f
#define PTZ_MOTOR_YAW_SPEED_KD 0.0f

//30 0 3  35 0.15 0
#define PTZ_MOTOR_PITCH_POSITION_KP 30.0f
#define PTZ_MOTOR_PITCH_POSITION_KI 0.05f
#define PTZ_MOTOR_PITCH_POSITION_KD 3.0f
#define PTZ_MOTOR_PITCH_SPEED_KP 40.0f
#define PTZ_MOTOR_PITCH_SPEED_KI 0.0f
#define PTZ_MOTOR_PITCH_SPEED_KD 3.0f


/***********************    ��������޸ġ�   ************************/			
#define Magaine_ON_PWM                        25
#define Magaine_OFF_PWM                       5

/***********************    �����ǲ����޸ġ�   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN�����޸ġ�   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    ���̵�������޸ġ�   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif

#if INFANTRY == 5               //������Ų�����
/**
 *@tip  �������� С����
 */
/**********************   �ӵ�λ���������޸ġ�  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //һ�ŵ����ӵ�������    ������
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //һ�Ÿ����ӵ�������     ������
		#define 		 FAST_THREEBULLET_HEAT        66   //���������ӵ�������       ��������
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //�ɲ������
		
		#define      DIAL_ONE_POSITION       4320 //һ���ӵ���λ��
    #define      DIAL_THREE_POSITION     12960  //�����ӵ���λ�� 
		#define      DIAL_BACK_POSITION      1800 //������λ��  
		
		#define SONEBULLET_DISTANCE         1200    //һ���ӵ�����Ƶ�������ж�(һ��)
		#define MONEBULLET_DISTANCE         1800    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         3000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       800     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       1000    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       2000    //�����ӵ�����Ƶ�������ж�(����)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //һ���ӵ���λ�� 
    #define      DIAL_THREE_POSITION     4860  //�����ӵ���λ��  
		#define      DIAL_BACK_POSITION      1000   //������λ��
		
		#define SONEBULLET_DISTANCE         500    //һ���ӵ�����Ƶ�������ж�(һ��)   240 360 480
		#define MONEBULLET_DISTANCE         750    //һ���ӵ�����Ƶ�������ж�(����)
		#define LONEBULLET_DISTANCE         1000    //һ���ӵ�����Ƶ�������ж�(����)

		#define STHREEBULLET_DISTANCE       200     //�����ӵ�����Ƶ�������ж�(һ��)
		#define MTHREEBULLET_DISTANCE       300    //�����ӵ�����Ƶ�������ж�(����)
		#define LTHREEBULLET_DISTANCE       400    //�����ӵ�����Ƶ�������ж�(����)
		
		#endif
		//��ѹ��10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //Ħ���ֹر�
		#define      NORMAL_FRICITION_ON_SPEED   1950   //��ͨĦ���ֻ����ٶ� ��Vave=26m/s�� >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //�е�Ħ���ֻ����ٶ� (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //�����Ħ���ֻ����ٶ�(16m/s)	
/***********************    ���ʻ������޸ġ�   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //����ģʽ�µĹ���
		#define      POWER_MIDDLE_LIMIT_VALUE  75       //����ģʽ�µĹ���
		#define      POWER_FAST_LIMIT_VALUE    90       //����ģʽ�µĹ���
		#define      POWER_INCREASE            5        //����������ֵ
		#define      POWER_OFFSET              4.6     //���̾�ֹʱ��ʵ�ʹ���
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//����ģʽ��KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//����ģʽ��KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //Ħ���ֿ����ٶ�
		
		#define POWER_KP                   0.5f      //���ʻ�KPֵ
    #define POWER_KI                   1.2f     //���ʻ�KIֵ����ɼ���������
    #define POWER_KD                   0.0f     //���ʻ�KDֵ
/***********************    ��̨�����޸ġ�   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch����
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch����
		#define      YAW_UP_LIMIT           40.0f      //yaw����
		#define      YAW_DOWM_LIMIT        -40.0f     //yaw����
		#define      YAW_INITIAL_VALUE       4047    //Yaw���ʼֵ  
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID�޷������޸ġ�   ************************/	  //�����ǵĳ�ʼƫ��
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //�������ر����߻�����������Ϊ0ʱ �Ե���������о����޷�
#define POWER_MAX_LIMIT_LEVEL2   1000    //���򳬹��ʿ�Ѫ
#define MPU6500_TEMP_PWM_MAX  100

/***********************    ���PID�����޸ġ�   ************************/
#define CHASSIS_MOTOR_KP 6.0f
#define CHASSIS_MOTOR_KI 0.02f
#define CHASSIS_MOTOR_KD 2.0f

#define HIT_POSITION_KP 1.9f
#define HIT_POSITION_KI 0.0f
#define HIT_POSITION_KD 0.1f
#define HIT_SPEED_KP 3.0f
#define HIT_SPEED_KI 0.0f
#define HIT_SPEED_KD 1.0f

#define CHASSIS_MOTOR_ROTATE_KP 55.0f
#define CHASSIS_MOTOR_ROTATE_KI 0.00f
#define CHASSIS_MOTOR_ROTATE_KD 2500.0f

#define PTZ_MOTOR_YAW_POSITION_KP 11.0f
#define PTZ_MOTOR_YAW_POSITION_KI 0.0f
#define PTZ_MOTOR_YAW_POSITION_KD 5.0f
#define PTZ_MOTOR_YAW_SPEED_KP 110.0f
#define PTZ_MOTOR_YAW_SPEED_KI 0.10f
#define PTZ_MOTOR_YAW_SPEED_KD 0.0f

//30 0 3  35 0.15 0
#define PTZ_MOTOR_PITCH_POSITION_KP 25.0f
#define PTZ_MOTOR_PITCH_POSITION_KI 0.05f
#define PTZ_MOTOR_PITCH_POSITION_KD 3.0f
#define PTZ_MOTOR_PITCH_SPEED_KP 45.0f
#define PTZ_MOTOR_PITCH_SPEED_KI 0.0f
#define PTZ_MOTOR_PITCH_SPEED_KD 3.0f


/***********************    ��������޸ġ�   ************************/			
#define Magaine_ON_PWM                        5
#define Magaine_OFF_PWM                       25

/***********************    �����ǲ����޸ġ�   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN�����޸ġ�   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    ���̵�������޸ġ�   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif

#endif



