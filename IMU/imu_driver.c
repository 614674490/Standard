#include "mpu6500_reg.h"
#include "spi.h"
#include "include.h"
uint8_t MPU_id = 0;

MagMaxMinData_t MagMaxMinData;

volatile MPU6500_RAW_DATA    MPU6500_Raw_Data;    //原始数据
volatile MPU6500_REAL_DATA   MPU6500_Real_Data;
uint8_t isMPU6500_is_DRY = 0;   // mpu6500 interrupt中断标志

uint8_t mpu_buff[14];

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

int16_t MPU6500_FIFO[6][11] = {0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值
int16_t gxset = 0, gyset= 0 , gzset = 0, axset = 0, ayset =0, azset =0;//步兵3


//Write a register to MPU6500
uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg&0x7f;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  MPU_Tx = data;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return 0;
}

//Read a register from MPU6500
uint8_t MPU6500_Read_Reg(uint8_t const reg)
{
  static uint8_t MPU_Rx, MPU_Tx;
  
  MPU6500_NSS_Low();
  
  MPU_Tx = reg|0x80;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  
  MPU6500_NSS_High();
  return MPU_Rx;
}

//Read registers from MPU6500,address begin with regAddr  
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len)
{
  static uint8_t MPU_Rx, MPU_Tx, MPU_Tx_buff[14] = {0xff};
  MPU6500_NSS_Low();
  
  MPU_Tx = regAddr|0x80;
  MPU_Tx_buff[0] = MPU_Tx;
  HAL_SPI_TransmitReceive(&hspi5, &MPU_Tx, &MPU_Rx, 1, 55);
  HAL_SPI_TransmitReceive(&hspi5, MPU_Tx_buff, pData, len, 55);
  
  MPU6500_NSS_High();
  return 0;
}
//Initialize the MPU6500  初始化MPU6500  输出 加速度 角速度
uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[20][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
		{MPU6500_INT_PIN_CFG,   0x10},
		{MPU6500_INT_ENABLE,    0x01},
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro  加速度输出
		{MPU6500_SMPLRT_DIV,    0x00},      //采样频率 1KHZ
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz  低通滤波器
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps  角速度
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G  加速度
    {MPU6500_ACCEL_CONFIG_2,0x00},     
		{MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
		{MPU6500_USER_CTRL,     0x00},     
    {MPU6500_USER_CTRL,     0x10},    
    {MPU6500_USER_CTRL,     0x30},      
  };
  
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 20; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }
  return 0;
}


//Set the accelerated velocity resolution  设置加速度分辨率
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution  设置角速度分辨率
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}


/***********************************************************
                       华丽的分割线
***********************************************************/
/**************************实现函数********************************************
*函数原型:	  void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*功　　能:	  将MPU6500_ax,MPU6500_ay, MPU6500_az,MPU6500_gx, MPU6500_gy, MPU6500_gz去10次平均数后处理后存储 加速度  角速度
*输入参数：   存在MPU6500_FIFO[i][10]
*输出参数：    
*******************************************************************************/
void MPU6500_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]为最近10次数据 [10]为10次数据的平均值
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6500_FIFO[0][i-1]=MPU6500_FIFO[0][i];
		MPU6500_FIFO[1][i-1]=MPU6500_FIFO[1][i];
		MPU6500_FIFO[2][i-1]=MPU6500_FIFO[2][i];
		MPU6500_FIFO[3][i-1]=MPU6500_FIFO[3][i];
		MPU6500_FIFO[4][i-1]=MPU6500_FIFO[4][i];
		MPU6500_FIFO[5][i-1]=MPU6500_FIFO[5][i];
	}
	
	MPU6500_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
	MPU6500_FIFO[1][9]=ay;
	MPU6500_FIFO[2][9]=az;
	MPU6500_FIFO[3][9]=gx;
	MPU6500_FIFO[4][9]=gy;
	MPU6500_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=MPU6500_FIFO[0][i];
	}
	MPU6500_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6500_FIFO[1][i];
	}
	MPU6500_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6500_FIFO[2][i];
	}
	MPU6500_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6500_FIFO[3][i];
	}
	MPU6500_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6500_FIFO[4][i];
	}
	MPU6500_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6500_FIFO[5][i];
	}
	MPU6500_FIFO[5][10]=sum/10;
  if(soft_js_time>=4000&&soft_js_time<=4400)   
  {
		axset=-MPU6500_FIFO[0][10];
	  ayset=MPU6500_FIFO[1][10];
		#if MPU_STATUS==MPU_UPSIDE_DOWN
	  azset=(-MPU6500_FIFO[2][10])-4096;
	  #elif  MPU_STATUS==MPU_NORMAL
		azset=MPU6500_FIFO[2][10]-4096;
		#endif
	  gxset=MPU6500_FIFO[3][10];
	  gyset=MPU6500_FIFO[4][10];
	  gzset=MPU6500_FIFO[5][10];
  }
	
}


//Get 6 axis data from MPU6500
void MPU_Get_Data(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
  if(isMPU6500_is_DRY)//MPU触发1KHz外部中断
  {
	isMPU6500_is_DRY = 0;
    MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
    imu_data.ax = mpu_buff[0]<<8 |mpu_buff[1];  //加速度
    imu_data.ay = mpu_buff[2]<<8 |mpu_buff[3];
    imu_data.az = mpu_buff[4]<<8 |mpu_buff[5];
  
    imu_data.temp = mpu_buff[6]<<8 |mpu_buff[7];
  
    imu_data.gx = mpu_buff[8]<<8  |mpu_buff[9]  - imu_data_offest.gx;  //角速度
    imu_data.gy = mpu_buff[10]<<8 |mpu_buff[11] - imu_data_offest.gy;
    imu_data.gz = mpu_buff[12]<<8 |mpu_buff[13] - imu_data_offest.gz;
	
	MPU6500_DataSave(imu_data.ax,imu_data.ay,imu_data.az,imu_data.gx,imu_data.gy,imu_data.gz);  		
	*ax = MPU6500_FIFO[0][10]-axset;//=MPU6500_FIFO[0][10];
	*ay = MPU6500_FIFO[1][10]-ayset;//=MPU6500_FIFO[1][10];
	#if MPU_STATUS==MPU_UPSIDE_DOWN
	*az = -MPU6500_FIFO[2][10]-azset;//=MPU6500_FIFO[2][10];
	#elif  MPU_STATUS==MPU_NORMAL
	*az = MPU6500_FIFO[2][10]-azset;//=MPU6500_FIFO[2][10];
	#endif
	*gx = MPU6500_FIFO[3][10]-gxset;
	*gy = MPU6500_FIFO[4][10]-gyset;
	*gz = MPU6500_FIFO[5][10]-gzset;

		
	}
	else
	{       //读取上一次的值
		*ax = MPU6500_FIFO[0][10]-axset;//=MPU6500_FIFO[0][10];
		*ay = MPU6500_FIFO[1][10]-ayset;//=MPU6500_FIFO[1][10];
		#if MPU_STATUS==MPU_UPSIDE_DOWN
	  *az = -MPU6500_FIFO[2][10]-azset;//=MPU6500_FIFO[2][10];
  	#elif  MPU_STATUS==MPU_NORMAL
	  *az = MPU6500_FIFO[2][10]-azset;//=MPU6500_FIFO[2][10];
	  #endif
		*gx = MPU6500_FIFO[3][10]-gxset;
		*gy = MPU6500_FIFO[4][10]-gyset;
		*gz = MPU6500_FIFO[5][10]-gzset;
	}
}
void error_0(void)          //抑制零点漂移的速度 减慢他的自增自减速度
{
	static int ii = 0;              
  static float  sum = 0;
	sum=0;
	
	if(deal == 1)                     //标志位的判断不能忽略
	{
		for(ii = 0; ii <512; ii++)
		{
			diff_save[ii] = ang_save[ii+512] - ang_save[ii];  //计算偏差
		}

		for(ii = 0; ii <512 ; ii++)
		{
				sum += diff_save[ii];             //偏差求和
		}
		if(force==0)
		{
			force =  -sum /262144.0f;//262144=512*512   //第一次偏差计算
		}
		else
		{
		  force = force * 0.3f - (sum /262144.0f) * 0.7f;//262144=512*512
		}
		deal =0;
  }
}
