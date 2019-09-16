/*******************************************************************************************
  * @Include @Headfile
 *******************************************************************************************/
#include "ks103.h"
#include "usart.h"

/*******************************************************************************************
  * @Parameter @Statement @Function
 *******************************************************************************************/
//Ks103_t Ks103_Front;
//Ks103_t Ks103_Left;
//Ks103_t Ks103_Right;

/*KS103超声波模块通信协议文件*/

/*USART模式*/
void KS103_SensorInit(Ks103_t *ks103){
	ks103->KS103_SensorInit = &KS103_SensorInit;
	ks103->KS103_ParaInit = &KS103_ParaInit;
	ks103->KS103_DataDemand = &KS103_DataDemand;
	ks103->KS103_DataProcess = &KS103_DataProcess;
}

void KS103_ParaInit(Ks103_t *ks103, uint8_t address, UART_HandleTypeDef* huart, uint8_t* usartData){
	ks103->Command[0] = address;
	ks103->Command[1] = 0x02;
	ks103->Command[2] = 0xB4;
	ks103->address = address;
	ks103->huart = huart;
	ks103->ReceiveData = usartData;
}

void KS103_DataDemand(Ks103_t *ks103){
	delay_us(200);
	HAL_UART_Transmit(ks103->huart, ks103->Command, sizeof(ks103->Command[0]),10);
	delay_us(40);
	HAL_UART_Transmit(ks103->huart, ks103->Command+1, sizeof(ks103->Command[1]),10);
	delay_us(40);
	HAL_UART_Transmit(ks103->huart, ks103->Command+2, sizeof(ks103->Command[2]),10);
	delay_us(200);
}

void KS103_DataProcess(Ks103_t *ks103){
	uint8_t i;
	for(i=0;i<6;i++){
		if((ks103->ReceiveData[i]==0x00)||(ks103->ReceiveData[i]==0x01)){ 
			ks103->distance = (uint16_t)(ks103->ReceiveData[i]<<8 | ks103->ReceiveData[i+1]);
		}			 
	}
}	

/*IIC模式*/
void KS103_WriteOneByte(u16 WriteAddr,u8 Num,u8 command)
{
	IIC_Start();
	IIC_Send_Byte(WriteAddr); //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(Num);//发送高地址
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //发送低地址
	IIC_Wait_Ack();
	IIC_Stop();//产生一个停止条件
}

uint8_t KS103_ReadOneByte(u16 ReadAddr,u8 DataToRead)
{
	u16 data=0;
    IIC_Start();
    IIC_Send_Byte(ReadAddr); //发送低地址
    IIC_Wait_Ack();
    IIC_Send_Byte(DataToRead); //发送低地址
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(ReadAddr + 1); //进入接收模式
    IIC_Wait_Ack();
    delay_us(50); //增加此代码通信成功
    data=IIC_Read_Byte(0); //读寄存器 3
    IIC_Stop();//产生一个停止条件
  return data;
}

void Change_Addr(u8 Original_Addr,u8 Now_Addr)//改I2C地址的函数，Original_Addr是超声波模块的现有的地址，改成地址Now_Addr
{																							//这个程序不能在while(1)里运行，只运行一次就可以，运行过程中不得断电!!!!!!!
	KS103_WriteOneByte(Original_Addr,0x02,0x9a);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,0x92);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,0x9e);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,Now_Addr);
	delay_ms(100);
}

void Return_distance(u16 I2C_Addr,u16 Mode,u16 *DISTANCE)   //这个函数为读取超声波测得的距离的函数，I2C_Addr对应超声波的I2C地址
{                                                         	//Mode为超声波模块的档位选择，Mode建议用0xb4
	u16 distance = 0;
	KS103_WriteOneByte(I2C_Addr,0x02,Mode);
	delay_ms(80);
	distance = KS103_ReadOneByte(I2C_Addr, 0x02);
	distance <<= 8;
	distance += KS103_ReadOneByte(I2C_Addr, 0x03);
	delay_ms(1);
//	delay_ms(1);
//	if(READ_SCL){
//		distance = KS103_ReadOneByte(I2C_Addr, 0x02);
//		distance <<= 8;
//		distance += KS103_ReadOneByte(I2C_Addr, 0x03);
//		delay_ms(1);
//	}
	if(distance<1000)
		*DISTANCE = distance;
}
