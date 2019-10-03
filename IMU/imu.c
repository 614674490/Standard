#include "mpu6500_reg.h"
#include "include.h"

float yaw_raw=0;
volatile float yaw_filter=0,pitch_filter=0;  //记录滤波后的角度 ggg
float single_off =0,pitch_single_off=0;    //记录单次偏差
float all_off=0,pitch_all_off=0;        //记录所有历史偏差
volatile float yaw_off=0,pitch_off=0;  //保存angle[0]
Diff_ANGLE Diff_Yaw;
Diff_ANGLE Diff_Pitch;
const float  NUM1[3] = {0.99778,-1.99556, 0.99778};  //高通 1000 0.25
const float  DEN1[3] = {1,   -1.99556, 0.99557};

//const float  NUM1[3] = { 0.05849, 0, -0.05849};  //带通滤波  Fs=1000 Fc1=0.25 Fc2=20
//const float  DEN1[3] ={ 1,-1.88283,  0.88301};   //当magnitude=3.0时 后面的数据便失真 只要3.0之前的数据

u32 js = 0,pitch_js=0;
u16 count_number=0;
float ang_save[1024] = {0};
float diff_save[512] = {0};
u8 deal = 0;                         //零点票差标志位
float force = 0;                     //用于零点票差计算

float temp_gx, temp_gy, temp_gz;
//volatile 确保本指令不会因编译器的优化而省略 且要求每次直接读值
volatile float exInt, eyInt, ezInt;  // 误差积分
volatile float q0 = 1.0f;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;

volatile float mygetqval[9];	//用于存放传感器转换结果的数组
static volatile float gx, gy, gz, ax, ay, az,halfT;   //作用域仅在此文件中

static volatile float q[4]; //　四元数
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us
volatile float angle[3] = {0};
volatile float yaw_temp=0,pitch_temp,roll_temp;
volatile float last_yaw_temp=0,last_pitch_temp,last_roll_temp;
volatile float yaw_angle=0,pitch_angle,roll_angle,mpu_temp; //使用到的角度值 温度值


volatile float yaw_angle_last=0,pitch_angle_last=0,roll_angle_last=0;

extern IMUDataTypedef imu_data;//定义成全局变量
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x)
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getValues(volatile float * values)
*功　　能:	 读取加速度 陀螺仪 磁力计 的当前值
输入参数： 将结果存放的数组首地址
加速度值：原始数据，-8192-+8192
角速度值：deg/s
磁力计值：原始数据
输出参数：没有
*******************************************************************************/
void IMU_getValues(volatile float * values) {
    int16_t accgyroval[6];
    int i;
    //读取加速度和陀螺仪的当前ADC
    MPU_Get_Data(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
    MPU6500_Raw_Data.Accel_X = accgyroval[0];
    MPU6500_Raw_Data.Accel_Y = accgyroval[1];
    MPU6500_Raw_Data.Accel_Z = accgyroval[2];
    MPU6500_Raw_Data.Gyro_X = accgyroval[3];
    MPU6500_Raw_Data.Gyro_Y = accgyroval[4];
    MPU6500_Raw_Data.Gyro_Z = accgyroval[5];
    for(i = 0; i<6; i++)
    {
        if(i < 3)
        {
            values[i] = (float) accgyroval[i]/4096.0f;
        }
        else
        {
            values[i] = ((float) accgyroval[i]) / 16.38f; //转成度每秒
        }
    }


}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate   AHRS:航资参考系统 依赖于磁场和重力场
*功　　能:	 更新AHRS 更新四元数
输入参数：   当前的测量值。
输出参数：   没有
*******************************************************************************/
#define Kp 6.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.1f   // integral gain governs rate of convergence of gyroscope biases
void IMU_AHRSupdate(void) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    gx = mygetqval[3] * M_PI/180;
    gy = mygetqval[4] * M_PI/180;
    gz = mygetqval[5] * M_PI/180;
    ax = mygetqval[0];
    ay = mygetqval[1];
    az = mygetqval[2];
    now = Get_Time2_Micros();  //读取时间 单位是us

    if(now<lastUpdate)          //利用前后两次数据计算的时间差和算法来消除误差
    {
        //halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else
    {
        halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间
    //快速求平方根算法
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    //把加计的三维向量转成单位向量。

  
    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;
        // 用叉积误差来做PI修正陀螺零偏
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
    }
    // 四元数微分方程
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // 四元数规范化
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getQ(float * q)
*功　　能:	 更新四元数 返回当前的四元数组值
输入参数： 将要存放四元数的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getQ(volatile float * q) {

    IMU_getValues(mygetqval);	 //获取原始数据,加速度计和磁力计是原始值，陀螺仪转换成了deg/s
    IMU_AHRSupdate();
    q[0] = q0; //返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_getYawPitchRoll(float * angles)
*功　　能:	 更新四元数 返回当前解算后的姿态数据
输入参数： 将要存放姿态角的数组首地址
输出参数：没有
*******************************************************************************/
void IMU_getYawPitchRoll(volatile float * angles)
{
    IMU_getQ(q); //更新全局四元数
    //四元数转换成欧拉角，经过三角函数计算即可
	  angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw        -pi----pi
	  angles[1]= asin(-2 * q1 * q3 + 2 * q0* q2)* 180/M_PI;
	  angles[2]= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 180/M_PI; // roll

}
int yaw_count = 0;
//static int pitch_count = 0;

void GetPitchYawGxGyGz()
{

    temp_gx = MPU6500_Real_Data.Gyro_X;
    temp_gz = MPU6500_Real_Data.Gyro_Z;

    MPU6500_Real_Data.Gyro_X = mygetqval[3];
    MPU6500_Real_Data.Gyro_Y = -mygetqval[4];
    MPU6500_Real_Data.Gyro_Z = mygetqval[5];

    if(MPU6500_Real_Data.Gyro_X + 2.0f > temp_gx &&  MPU6500_Real_Data.Gyro_X < temp_gx + 2.0f)
		{
			MPU6500_Real_Data.Gyro_X = temp_gx;
			if(MPU6500_Real_Data.Gyro_X <= 2.0f && MPU6500_Real_Data.Gyro_X >= -2.0f)
				MPU6500_Real_Data.Gyro_X = 0;
		}  
    if(soft_js_time>=4500&&soft_js_time<=7700&&deal==0)                                 
	  {                                 
			 ang_save[count_number]=angle[0];
			 count_number++;
			 if(count_number%1024==0)
			 {
					deal = 1;
					count_number=0;
			 }
	  }
		
		yaw_angle_last = yaw_angle;
    last_yaw_temp = yaw_temp;
		
		if(soft_js_time>8000)
		{
			yaw_filter=diff_convert_yaw(angle[0],&Diff_Yaw);
			
			if(yaw_filter<=-0.10f||yaw_filter>=0.10f)   //云台开始移动，计算所有历史偏差值
			{
				js++;
				all_off+=single_off;
				yaw_temp =angle[0]-all_off+ force * js;
				single_off=0;
				yaw_off=angle[0];
			}
			else                            //云台静止，开始计算单次零漂值
			{   
				 single_off=angle[0]-yaw_off;  
			}
						
//			if(pitch_filter<=-0.10f||pitch_filter>=0.10f)   //云台开始移动，计算所有历史偏差值
//			{
//				pitch_js++;
//				pitch_all_off+=pitch_single_off;
//				pitch_temp =angle[2]-pitch_all_off+force*pitch_js;
//				pitch_single_off=0;
//				pitch_off=angle[2];
//			}
//			else                            //云台静止，开始计算单次零漂值
//			{   
//				 pitch_single_off=angle[2]-pitch_off;  
//			}
	  }
		else
		{
			 yaw_temp = angle[0];
		}
    if(yaw_temp-last_yaw_temp>=330)  //yaw轴角度经过处理后变成连续的
    {
        yaw_count--;
    }
    else if (yaw_temp-last_yaw_temp<=-330)
    {
        yaw_count++;
    }
    yaw_angle = (yaw_temp + yaw_count*360);  //yaw轴角度
		
		//将pitch角处理成连续方式  
		//kalman_filter_calc(&mpu_pitch_filter,angle[2],angle[2] );
//		pitch_temp=angle[2];
//		
//		pitch_angle_last = pitch_angle;
//		last_pitch_temp = pitch_temp;

//    if(pitch_temp-last_pitch_temp>=330)  //roll轴角度经过处理后变成连续的
//    {
//        pitch_count--;
//    }
//    else if (pitch_temp-last_pitch_temp<=-330)
//    {
//        pitch_count++;
//    }
//    pitch_angle = pitch_temp + pitch_count*360;  //roll轴角度
//		if(pitch_angle>0.5f) pitch_angle=0.0f;
//		else if(pitch_angle<-0.5f) pitch_angle=0.0f;
}

void MPU_Get_Temperature(void)   //获取陀螺仪温度
{
	static float mpu_temp_last=0;
	
	mpu_temp_last=mpu_temp;
  mpu_temp=21+imu_data.temp/333.87;
	if(_fabsf(mpu_temp-42.5f)<=0.02f)   //去除波动
		mpu_temp=mpu_temp_last;
}

void IMU_Init(void)
{
	  IMU_getYawPitchRoll(angle);
		error_0();
		MPU_Get_Temperature();
		mpu_pwm+=PID_Increment(mpu_temp,MPU_temp.expect,&MPU_temp);   //保持陀螺仪恒温
		if(mpu_pwm>MPU6500_TEMP_PWM_MAX) 
		mpu_pwm=MPU6500_TEMP_PWM_MAX;
		if(mpu_pwm<0)
		mpu_pwm=0;
		TIM_SetTIM3Compare2(mpu_pwm);
}

/**
  * @brief          滤波函数 适用于二阶低通 高通 贷通滤波  抑制陀螺仪yaw的零漂
  * @author         
  * @param[in]      初始数据 相关滤波结构体    
  * @retval         滤波后的角度
  */
float  diff_convert_yaw(float init_data,Diff_ANGLE *imu_type)
{
   imu_type->x_now=init_data;
   imu_type->y_now=NUM1[0]*imu_type->x_now+NUM1[1]*imu_type->x_last+NUM1[2]*imu_type->x_next-DEN1[1]*imu_type->y_last-DEN1[2]*imu_type->y_next;
	 imu_type->y_next=imu_type->y_last;  //记录前一 二次数据 当前数据
	 imu_type->y_last=imu_type->y_now;
	 imu_type->x_next=imu_type->x_last;
	 imu_type->x_last=imu_type->x_now;
   return imu_type->y_now;
}

