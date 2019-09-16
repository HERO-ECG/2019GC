#include "m_imu.h"
#include "mpu6500_reg.h"
#include "IST8310_reg.h"
#include "spi.h"
#include "math.h"

uint8_t MPU_id = 0;	

IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0,0,0,0};

IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};

uint8_t MPU6500_Write_Reg(uint8_t const reg, uint8_t const data);
uint8_t MPU6500_Read_Reg(uint8_t const reg);
uint8_t MPU6500_Read_Regs(uint8_t const regAddr, uint8_t *pData, uint8_t len);
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

//Write IST8310 register through MPU6500
static void IST_Reg_Write_By_MPU(uint8_t addr, uint8_t data)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, addr);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, data);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x080 | 0x01);
  HAL_Delay(10);
}

//Write IST8310 register through MPU6500
static uint8_t IST_Reg_Read_By_MPU(uint8_t addr)
{
  uint8_t data;
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_REG, addr);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x80);
  HAL_Delay(10);
  data = MPU6500_Read_Reg(MPU6500_I2C_SLV4_DI);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  return data;
}

//Initialize the MPU6500 I2C Slave0 for I2C reading
static void MPU_Auto_Read_IST_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  HAL_Delay(2);
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x03);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  HAL_Delay(2);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  HAL_Delay(6);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  HAL_Delay(7);
}

//Initialize the IST8310
uint8_t IST8310_Init(void)
{
  MPU6500_Write_Reg(MPU6500_USER_CTRL, 0x30);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_MST_CTRL, 0x0d);
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x01);
  if(IST8310_DEVICE_ID_A != IST_Reg_Read_By_MPU(IST8310_WHO_AM_I))
    return 1; //error
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFA, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFA) != 0x00)
    return 2;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_R_CONFB, 0x00);
  if(IST_Reg_Read_By_MPU(IST8310_R_CONFB) != 0x00)
    return 3;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_AVGCNTL, 0x24);
  if(IST_Reg_Read_By_MPU(IST8310_AVGCNTL) != 0x24)
    return 4;
  HAL_Delay(10);
  
  IST_Reg_Write_By_MPU(IST8310_PDCNTL, 0xc0);
  if(IST_Reg_Read_By_MPU(IST8310_PDCNTL) != 0xc0)
    return 5;
  HAL_Delay(10);
  
  MPU6500_Write_Reg(MPU6500_I2C_SLV1_CTRL, 0x00);
  HAL_Delay(10);
  MPU6500_Write_Reg(MPU6500_I2C_SLV4_CTRL, 0x00);
  HAL_Delay(10);
  
  MPU_Auto_Read_IST_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  HAL_Delay(100);
  return 0;
}

//Set the accelerated velocity resolution
uint8_t MPU6500_Set_Accel_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_ACCEL_CONFIG, fsr<<3);
}

//Set the angular velocity resolution
uint8_t MPU6500_Set_Gyro_Fsr(uint8_t fsr)
{
  return MPU6500_Write_Reg(MPU6500_GYRO_CONFIG, fsr<<3);
}




#define imu_length	30


float pitch_history[imu_length];
float yaw_history[imu_length];
float roll_history[imu_length];

void GyroQueue(float *pitch,float *yaw,float *roll,uint16_t index,uint8_t mode)
{
	static uint16_t pointer;


	if(mode==SET)
	{
		pointer++;
		if(pointer==imu_length)
			pointer=0;
		pitch_history[pointer]=*pitch;
		yaw_history[pointer]=*yaw;
		roll_history[pointer]=*roll;
	}
	else
	{
		if(index>=imu_length)
		{
			*pitch=0;
			*yaw=0;
			*roll=0;
		}
		else
		{
			if(index<pointer)
			{
				*pitch=pitch_history[pointer-index];
				*yaw=yaw_history[pointer-index];
				*roll=roll_history[pointer-index];
			}
			else
			{
				*pitch=pitch_history[imu_length-index+pointer-1];
				*yaw=yaw_history[imu_length-index+pointer-1];
				*roll=roll_history[imu_length-index+pointer-1];
			}
		}
	}
}

float GetVariance(float *data,uint16_t length)
{
	float average;
	float sum=0;
	float variance;
	uint16_t i;
	
	for(i=0;i<length;i++)
		sum+=data[i];
	average=sum/length;
	for(i=0;i<length;i++)
		variance+=(data[i]-average)*(data[i]-average)/length;
	return variance;
}

float GetAverage(float *data,uint16_t length)
{
	float average;
	float sum=0;
	uint16_t i;
	
	for(i=0;i<length;i++)
		sum+=data[i];
	average=sum/length;
	return average;
}
uint16_t still_counter;
float variance_sum;
float average[3];
void AutoOffset()
{
	static uint8_t autojust_flag;

	GyroQueue(&imu_data.gx_origin,&imu_data.gy_origin,&imu_data.gz_origin,0,SET);
	variance_sum=sqrt(GetVariance(pitch_history,imu_length)+GetVariance(yaw_history,imu_length)+GetVariance(roll_history,imu_length));
	/*矫正状态需要保证底盘静止不动*/
	average[0]=GetAverage(pitch_history,imu_length);
	average[1]=GetAverage(yaw_history,imu_length);
	average[2]=GetAverage(roll_history,imu_length);

	if((imu_data.temp>(imu_data.temp_set-2))&&autojust_flag==0)//之后加温控要改温度
	{
		if(variance_sum<45&&(fabs(average[0])+fabs(average[1])+fabs(average[2])<50.0f))
		{
			if(still_counter<0xffff)
				still_counter++;
		}
		else
			still_counter=0;
		if(still_counter>4000)
		{
			imu_data_offest.gx=average[0];
			imu_data_offest.gy=average[1];
			imu_data_offest.gz=average[2];
			autojust_flag=1;
		}
	//	else
	}

}


//Get 6 axis data from MPU6500
#define sensitivity_g (256.0f*16)
void IMU_Get_Data()
{
	static uint8_t cnt;
  uint8_t mpu_buff[14];
  MPU6500_Read_Regs(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
  
  imu_data.ax = (int16_t)(mpu_buff[0]<<8 |mpu_buff[1])/sensitivity_g;
  imu_data.ay = (int16_t)(mpu_buff[2]<<8 |mpu_buff[3])/sensitivity_g;
  imu_data.az = (int16_t)(mpu_buff[4]<<8 |mpu_buff[5])/sensitivity_g;
  
  imu_data.temp = (mpu_buff[6]<<8 |mpu_buff[7])/256.0f;
		imu_data.gx_origin = (int16_t)(mpu_buff[8]<<8 |mpu_buff[9]) ;
		imu_data.gy_origin = (int16_t)(mpu_buff[10]<<8 |mpu_buff[11]);
		imu_data.gz_origin = (int16_t)(mpu_buff[12]<<8 |mpu_buff[13]);
	
	if(PidCalc(&imu_data.temp_pid,imu_data.temp,imu_data.temp_set)>0)
		TIM3->CCR2=imu_data.temp_pid.output.pos_out;
	else
		TIM3->CCR2=0;
	
	if(cnt<100){//0.1s初始化时间
		cnt++;
		imu_data_offest.gx += (int16_t)(mpu_buff[8]<<8 |mpu_buff[9])/100.0f;
		imu_data_offest.gy += (int16_t)(mpu_buff[10]<<8 |mpu_buff[11])/100.0f;
		imu_data_offest.gz += (int16_t)(mpu_buff[12]<<8 |mpu_buff[13])/100.0f;
		
		imu_data.init_sta = 0;
	}
	else
	{
		AutoOffset();
		imu_data.init_sta = 1;
	}
  imu_data.angley=asin(imu_data.ay/sqrt(imu_data.ay*imu_data.ay+imu_data.ax*imu_data.ax+imu_data.az*imu_data.az))*180/3.1415926f;//前翻为正(20)
	imu_data.anglex=asin(imu_data.ax/sqrt(imu_data.ay*imu_data.ay+imu_data.ax*imu_data.ax+imu_data.az*imu_data.az))*180/3.1415926f;//右倒为正(15)
	imu_data.gx=imu_data.gx_origin - imu_data_offest.gx;
	imu_data.gy=imu_data.gy_origin - imu_data_offest.gy;
	imu_data.gz=imu_data.gz_origin - imu_data_offest.gz;
}



//Initialize the MPU6500
uint8_t MPU6500_Init(void)
{
  uint8_t index = 0;
  uint8_t MPU6500_Init_Data[10][2] = 
  {
    {MPU6500_PWR_MGMT_1,    0x80},      // Reset Device
    {MPU6500_PWR_MGMT_1,    0x03},      // Clock Source - Gyro-Z
    {MPU6500_PWR_MGMT_2,    0x00},      // Enable Acc & Gyro
    {MPU6500_CONFIG,        0x02},      // LPF 98Hz
    {MPU6500_GYRO_CONFIG,   0x18},      // +-2000dps
    {MPU6500_ACCEL_CONFIG,  0x10},      // +-8G
    {MPU6500_ACCEL_CONFIG_2,0x02},      // enable LowPassFilter  Set Acc LPF
    {MPU6500_USER_CTRL,     0x20},      // Enable AUX
  };
	
	
	
	imu_data.temp_pid.pid_mode=POSITION_PID;
	imu_data.temp_pid.max_output=999;
	imu_data.temp_pid.integral_uplimit=700;
	imu_data.temp_pid.integral_downlimit=-700;
	imu_data.temp_pid.p=400;
	imu_data.temp_pid.i=0.05;
	imu_data.temp_pid.d=0;
	imu_data.temp_pid.deadband=0;
	imu_data.temp_pid.f=0;
	imu_data.temp_pid.max_err=50;
	imu_data.temp_pid.mid_err=10;
	imu_data.temp_pid.min_err=0;
	imu_data.temp_pid.f_out_limit=0;
	
  imu_data.temp_set=40;
	
  HAL_Delay(100);
  MPU_id = MPU6500_Read_Reg(MPU6500_WHO_AM_I);  //read id of device,check if MPU6500 or not
  
  for(index = 0; index < 10; index++)
  {
    MPU6500_Write_Reg(MPU6500_Init_Data[index][0], MPU6500_Init_Data[index][1]);
    HAL_Delay(1);
  }

  return 0;
}


