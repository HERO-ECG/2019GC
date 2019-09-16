#ifndef __MYIIC_H
#define __MYIIC_H

#include "mytype.h"
#include "sys.h"

//IO��������
 
#define SDA_IN()  {GPIOE->MODER&=~(3<<(4*2));GPIOE->MODER|=(0<<(4*2));}
#define SDA_OUT() {GPIOE->MODER&=~(3<<(4*2));GPIOE->MODER|=(1<<(4*2));}

//IO��������	 
#define IIC_SCL_ON    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET) //SCL
#define IIC_SDA_ON    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET) //SDA
#define IIC_SCL_OFF   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET) //SCL
#define IIC_SDA_OFF   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET) //SDA

#define READ_SDA   	PEin(4)  //����SDA 
#define READ_SCL		PEin(5)

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void delay_us(u32 nus);
void delay_ms(u16 ums);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
uint8_t I2C_CheckDevice(uint8_t address);//1�ɹ� 0ʧ��;
#endif
















