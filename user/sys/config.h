#ifndef  _CONFIG_H
#define  _CONFIG_H
#define INFANTRY                     3     // 步兵编号 
#define DEBUG_MODE                   1    //1 启动调试模式（只开启摩擦轮与拨弹电机）   0 不启动调试模式 
#define CHASSIS_FEEDBACK             2     //1启用裁判系 统    2 启用电流计/3508转矩电流  0 不启用功率闭环
#define SHOOT_FEEDBACK               0     //1启用裁判系统(热量与等级)    2 定时器计算  3裁判系统(只是读取等级)  0 不启用热量闭环
#define Client_Robot_Iteractive      1     //客户端 机器人通信
#define REFEREE_SYSTEM               0     //1:开启裁判系统 0：关闭裁判系统

#define DIAL_MOTOR_OLD               0
#define DIAL_MOTOR_NEW               1
#define FRICTION_420S_STATUS         0        //是否使用420S电调

#define ACCELERA_TIME               50000     //加速斜坡时间
#define DECELERATE_TIME             15000    //减速斜坡时间
#define PTZ_MODE                    0        //默认不开启模糊	云台模式
#define FRICTION_ON_TIME            1000    //摩擦轮开启斜坡时间
#define FRICTION_OFF_TIME           1000    //摩擦轮关闭斜坡时间

/*                       摩擦轮电调相关域值                               */
#define Friction_420S_MIN_Value               900
#define Friction_420S_MAX_Value               2000
#define Friction_420S_Period                  20   //420S控制周期
#define Friction_2312_Domain_Value            1000

#define PRESS_LONG_TIME              100      //鼠标左键持续按下时间 用于连发清弹

#define PTZ_LOCK_ANGLE               3.0f    //云台锁死角度
#define CONTINUE_DIAL_POSITION       200
#define REDIAL_BULLET_POSITION       1000
#define REDIAL_BULLET_SPEED          5      //拨弹电机的速度检测速度
#define KEY_PRESS_DELAY              30      //按键响应延时时间
#define FRICITION_CLOSE_DELAY        75      //摩擦轮关闭延时时间

#define POWER_VOLTAGE                24.0f   //电源电压
#define POWER_LIMIT_UP               80     //输出功率上限
#define CURRENT_OFFSET               1000.0f //电流比例系数
#define VOLTAGE_OFFSET               1000.0f //电压比例系数
/*************************** 陀螺仪位置状态 ***********************************/
#define MPU_NORMAL              0
#define MPU_UPSIDE_DOWN         1
#define MPU_USART               2
/*************************** CAN使用状态 ***********************************/
#define CAN1_Chassis               0
#define CAN2_Chassis               1
/*************************** 功率计算相关参数 ***********************************/
#define POWER_CHECK_TIME        1.0f        //功率通讯周期  0.5s
#define POWER_BUFFER_LIMIT      60          //最大缓冲能量 (是裁判系统 还是PID控制周期) 60J
#define POWER_BUFFER_THRESHOLD  25          //功率缓冲能量危险值 10J  预测用
#define POWER_BUFFER_DANGER     10          //功率缓冲能量危险值 当前用
#define RT_CAP_TAL_THRESHOLD    12.0f       //扭腰或者小陀螺时的超级电容电量阈值
#define CAP_TAL_THRESHOLD       10.5f       //普通状态下的超级电容电量阈值
#define CAP_TAL_BUFFER          3.0f        //超级电容的电量缓冲 防止超级电容的来回切换频率过快
/*************************** 底盘电机相关参数 ***********************************/
#define M3510                      0
#define M3508                      1
#define FAST_ROTATE_K              3.7f
#define NORMAL_ROTATE_K            3.2f
#if INFANTRY == 3               //步兵三号参数↓
/**
 *@tip 大符 超级电容 无小陀螺
 */
/**********************   子弹位置与热量修改↓  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //一颗低速子弹的热量    连发用
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //一颗高速子弹的热量     单发用
		#define 		 FAST_THREEBULLET_HEAT        66   //三颗中速子弹的热量       三连发用
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //旧拨弹电机
		
		#define      DIAL_ONE_POSITION       4320 //一颗子弹的位置
    #define      DIAL_THREE_POSITION     12960  //三颗子弹的位置 
		#define      DIAL_BACK_POSITION      1800 //倒弹的位置  
		
		#define SONEBULLET_DISTANCE         1200    //一颗子弹的射频距离敏感度(一级)
		#define MONEBULLET_DISTANCE         1800    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         3000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       800     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       1000    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       2000    //三颗子弹的射频距离敏感度(三级)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //一颗子弹的位置 
    #define      DIAL_THREE_POSITION     4860  //三颗子弹的位置  
		#define      DIAL_BACK_POSITION      1000   //倒弹的位置
		
		#define SONEBULLET_DISTANCE         500    //一颗子弹的射频距离敏感度(一级)   240 360 480
		#define MONEBULLET_DISTANCE         750    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         1000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       200     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       300    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       400    //三颗子弹的射频距离敏感度(三级)
		
		#endif
		 //稳压：10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //摩擦轮关闭
		#define      NORMAL_FRICITION_ON_SPEED   1950   //普通摩擦轮基础速度 （Vave=26m/s） >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //中等摩擦轮基础速度 (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //大火力摩擦轮基础速度(16m/s)	
/***********************    功率环参数修改↓   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //低速模式下的功率
		#define      POWER_NORMAL_LIMIT_VALUE  75       //中速模式下的功率
		#define      POWER_FAST_LIMIT_VALUE    90       //高速模式下的功率
		#define      POWER_INCREASE            5        //功率增长阈值
		#define      POWER_OFFSET              4.6     //底盘静止时的实际功率
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//低速模式的KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//高速模式的KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //摩擦轮开启速度
		
		#define POWER_KP                   0.5f      //功率环KP值
    #define POWER_KI                   1.2f     //功率环KI值增大可减缓超功率
    #define POWER_KD                   0.0f     //功率环KD值
/***********************    云台参数修改↓   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch上限
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch下限
		#define      YAW_INITIAL_VALUE       1376     //Yaw轴初始值  
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID限幅参数修改↓   ************************/	  //陀螺仪的初始偏差
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //当功率特别大或者缓冲能量即将为0时 对底盘输出进行绝对限幅
#define POWER_MAX_LIMIT_LEVEL2   1000    //当因超功率扣血
#define MPU6500_TEMP_PWM_MAX  100

/***********************    电机PID参数修改↓   ************************/
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


/***********************    舵机参数修改↓   ************************/			
#define Magaine_ON_PWM                        25
#define Magaine_OFF_PWM                       5

/***********************    陀螺仪参数修改↓   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN参数修改↓   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    底盘电机参数修改↓   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif


#if INFANTRY == 4               //步兵四号参数↓
/**
 *@tip 自瞄 超级电容 小陀螺
 */
/**********************   子弹位置与热量修改↓  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //一颗低速子弹的热量    连发用
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //一颗高速子弹的热量     单发用
		#define 		 FAST_THREEBULLET_HEAT        66   //三颗中速子弹的热量       三连发用
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //旧拨弹电机
		
		#define      DIAL_ONE_POSITION       4320 //一颗子弹的位置
    #define      DIAL_THREE_POSITION     12960  //三颗子弹的位置 
		#define      DIAL_BACK_POSITION      1800 //倒弹的位置  
		
		#define SONEBULLET_DISTANCE         1200    //一颗子弹的射频距离敏感度(一级)
		#define MONEBULLET_DISTANCE         1800    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         3000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       800     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       1000    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       2000    //三颗子弹的射频距离敏感度(三级)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //一颗子弹的位置 
    #define      DIAL_THREE_POSITION     4860  //三颗子弹的位置  
		#define      DIAL_BACK_POSITION      1000   //倒弹的位置
		
		#define SONEBULLET_DISTANCE         500    //一颗子弹的射频距离敏感度(一级)   240 360 480
		#define MONEBULLET_DISTANCE         750    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         1000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       200     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       300    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       400    //三颗子弹的射频距离敏感度(三级)
		
		#endif
		 //稳压：10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //摩擦轮关闭
		#define      NORMAL_FRICITION_ON_SPEED   1950   //普通摩擦轮基础速度 （Vave=26m/s） >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //中等摩擦轮基础速度 (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //大火力摩擦轮基础速度(16m/s)	
/***********************    功率环参数修改↓   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //低速模式下的功率
		#define      POWER_MIDDLE_LIMIT_VALUE  75       //中速模式下的功率
		#define      POWER_FAST_LIMIT_VALUE    90       //高速模式下的功率
		#define      POWER_INCREASE            5        //功率增长阈值
		#define      POWER_OFFSET              4.6     //底盘静止时的实际功率
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//低速模式的KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//高速模式的KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //摩擦轮开启速度
		
		#define POWER_KP                   0.5f      //功率环KP值
    #define POWER_KI                   1.2f     //功率环KI值增大可减缓超功率
    #define POWER_KD                   0.0f     //功率环KD值
/***********************    云台参数修改↓   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch上限
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch下限
		#define      YAW_UP_LIMIT           40.0f      //yaw上限
		#define      YAW_DOWM_LIMIT        -40.0f     //yaw下限
		#define      YAW_INITIAL_VALUE       1483 //Yaw轴初始值
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID限幅参数修改↓   ************************/	  //陀螺仪的初始偏差
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //当功率特别大或者缓冲能量即将为0时 对底盘输出进行绝对限幅
#define POWER_MAX_LIMIT_LEVEL2   1000    //当因超功率扣血
#define MPU6500_TEMP_PWM_MAX  100

/***********************    电机PID参数修改↓   ************************/
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


/***********************    舵机参数修改↓   ************************/			
#define Magaine_ON_PWM                        25
#define Magaine_OFF_PWM                       5

/***********************    陀螺仪参数修改↓   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN参数修改↓   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    底盘电机参数修改↓   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif

#if INFANTRY == 5               //步兵五号参数↓
/**
 *@tip  超级电容 小陀螺
 */
/**********************   子弹位置与热量修改↓  ***********************/
    #define Dial_Motor_Status            DIAL_MOTOR_NEW 
		#define      SLOW_ONEBULLET_HEAT          16.0f   //一颗低速子弹的热量    连发用
		#define   	 FAST_ONEBULLET_HEAT          26.5f  //一颗高速子弹的热量     单发用
		#define 		 FAST_THREEBULLET_HEAT        66   //三颗中速子弹的热量       三连发用
		
    #if Dial_Motor_Status==DIAL_MOTOR_OLD   //旧拨弹电机
		
		#define      DIAL_ONE_POSITION       4320 //一颗子弹的位置
    #define      DIAL_THREE_POSITION     12960  //三颗子弹的位置 
		#define      DIAL_BACK_POSITION      1800 //倒弹的位置  
		
		#define SONEBULLET_DISTANCE         1200    //一颗子弹的射频距离敏感度(一级)
		#define MONEBULLET_DISTANCE         1800    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         3000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       800     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       1000    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       2000    //三颗子弹的射频距离敏感度(三级)
		
		#else
	  #define      DIAL_ONE_POSITION       1620  //一颗子弹的位置 
    #define      DIAL_THREE_POSITION     4860  //三颗子弹的位置  
		#define      DIAL_BACK_POSITION      1000   //倒弹的位置
		
		#define SONEBULLET_DISTANCE         500    //一颗子弹的射频距离敏感度(一级)   240 360 480
		#define MONEBULLET_DISTANCE         750    //一颗子弹的射频距离敏感度(二级)
		#define LONEBULLET_DISTANCE         1000    //一颗子弹的射频距离敏感度(三级)

		#define STHREEBULLET_DISTANCE       200     //三颗子弹的射频距离敏感度(一级)
		#define MTHREEBULLET_DISTANCE       300    //三颗子弹的射频距离敏感度(二级)
		#define LTHREEBULLET_DISTANCE       400    //三颗子弹的射频距离敏感度(三级)
		
		#endif
		//稳压：10.5V
		#define      NORMAL_FRICITION_OFF_SPEED  1000   //摩擦轮关闭
		#define      NORMAL_FRICITION_ON_SPEED   1950   //普通摩擦轮基础速度 （Vave=26m/s） >1800
		#define      MIDDLE_FRICITION_ON_SPEED   1750   //中等摩擦轮基础速度 (Vave=21.1m/s) 
    #define      LOW_FRICITION_ON_SPEED      1530   //大火力摩擦轮基础速度(16m/s)	
/***********************    功率环参数修改↓   ************************/		

		#define      POWER_LOW_LIMIT_VALUE     70       //低速模式下的功率
		#define      POWER_MIDDLE_LIMIT_VALUE  75       //中速模式下的功率
		#define      POWER_FAST_LIMIT_VALUE    90       //高速模式下的功率
		#define      POWER_INCREASE            5        //功率增长阈值
		#define      POWER_OFFSET              4.6     //底盘静止时的实际功率
		#define      KEY_LOWSPEED_KP         NORMAL_ZHI_LOWSPEED_KP//低速模式的KP
    #define      KEY_HIGHSPEED_KP        NORMAL_ZHI_HIGHSPEED_KP//高速模式的KP
		#define      FRICITION_ON_SPEED      NORMAL_FRICITION_ON_SPEED //摩擦轮开启速度
		
		#define POWER_KP                   0.5f      //功率环KP值
    #define POWER_KI                   1.2f     //功率环KI值增大可减缓超功率
    #define POWER_KD                   0.0f     //功率环KD值
/***********************    云台参数修改↓   ************************/				
		#define      PITCH_UP_LIMIT          25.0f   //pitch上限
		#define      PITCH_DOWM_LIMIT        -14.0f   //pitch下限
		#define      YAW_UP_LIMIT           40.0f      //yaw上限
		#define      YAW_DOWM_LIMIT        -40.0f     //yaw下限
		#define      YAW_INITIAL_VALUE       4047    //Yaw轴初始值  
		#define      YAW_TRUN_180_VALUE      0
		#define      YAW_TRUN_45_VALUE       0
		#define      AIMAUTO_STATUS          1
/***********************   PID限幅参数修改↓   ************************/	  //陀螺仪的初始偏差
#define HIT_MAX_OUT       15000
#define PITCH_MAX_OUT     5000
#define YAW_MAX_OUT       29000
#define ROTATE_MAX_OUT    4000
#define CHASSIS_MAX_OUT   16300
#define POWER_MAX_LIMIT_LEVEL1   2000    //当功率特别大或者缓冲能量即将为0时 对底盘输出进行绝对限幅
#define POWER_MAX_LIMIT_LEVEL2   1000    //当因超功率扣血
#define MPU6500_TEMP_PWM_MAX  100

/***********************    电机PID参数修改↓   ************************/
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


/***********************    舵机参数修改↓   ************************/			
#define Magaine_ON_PWM                        5
#define Magaine_OFF_PWM                       25

/***********************    陀螺仪参数修改↓   ************************/		
#define MPU_STATUS              MPU_USART

/***********************    CAN参数修改↓   ************************/		
#define CHASSIS_CAN_STATUS      CAN2_Chassis

/***********************    底盘电机参数修改↓   ************************/		
#define CHASSIS_MOTOR_TYPE     M3508
#endif

#endif



