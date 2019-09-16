#ifndef __MYIIC_H
#define __MYIIC_H

#include "mytype.h"
#include "sys.h"

//IO方向设置
 
#define SDA_IN()  {GPIOE->MODER&=~(3<<(4*2));GPIOE->MODER|=(0<<(4*2));}
#define SDA_OUT() {GPIOE->MODER&=~(3<<(4*2));GPIOE->MODER|=(1<<(4*2));}

//IO操作函数	 
#define IIC_SCL_ON    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET) //SCL
#define IIC_SDA_ON    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET) //SDA
#define IIC_SCL_OFF   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET) //SCL
#define IIC_SDA_OFF   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET) //SDA

#define READ_SDA   	PEin(4)  //输入SDA 
#define READ_SCL		PEin(5)

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void delay_us(u32 nus);
void delay_ms(u16 ums);

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
uint8_t I2C_CheckDevice(uint8_t address);//1成功 0失败;
#endif
















