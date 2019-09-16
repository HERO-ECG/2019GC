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

/*KS103������ģ��ͨ��Э���ļ�*/

/*USARTģʽ*/
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

/*IICģʽ*/
void KS103_WriteOneByte(u16 WriteAddr,u8 Num,u8 command)
{
	IIC_Start();
	IIC_Send_Byte(WriteAddr); //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(Num);//���͸ߵ�ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(command); //���͵͵�ַ
	IIC_Wait_Ack();
	IIC_Stop();//����һ��ֹͣ����
}

uint8_t KS103_ReadOneByte(u16 ReadAddr,u8 DataToRead)
{
	u16 data=0;
    IIC_Start();
    IIC_Send_Byte(ReadAddr); //���͵͵�ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(DataToRead); //���͵͵�ַ
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte(ReadAddr + 1); //�������ģʽ
    IIC_Wait_Ack();
    delay_us(50); //���Ӵ˴���ͨ�ųɹ�
    data=IIC_Read_Byte(0); //���Ĵ��� 3
    IIC_Stop();//����һ��ֹͣ����
  return data;
}

void Change_Addr(u8 Original_Addr,u8 Now_Addr)//��I2C��ַ�ĺ�����Original_Addr�ǳ�����ģ������еĵ�ַ���ĳɵ�ַNow_Addr
{																							//�����������while(1)�����У�ֻ����һ�ξͿ��ԣ����й����в��öϵ�!!!!!!!
	KS103_WriteOneByte(Original_Addr,0x02,0x9a);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,0x92);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,0x9e);
	delay_ms(1);
	KS103_WriteOneByte(Original_Addr,0x02,Now_Addr);
	delay_ms(100);
}

void Return_distance(u16 I2C_Addr,u16 Mode,u16 *DISTANCE)   //�������Ϊ��ȡ��������õľ���ĺ�����I2C_Addr��Ӧ��������I2C��ַ
{                                                         	//ModeΪ������ģ��ĵ�λѡ��Mode������0xb4
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
