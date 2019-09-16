#include "iic.h"
#include "gpio.h"

void IIC_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/*IIC复用IO口PE4/PE5*/
	GPIO_InitStruct.Pin = IIC_PIN_SDA|IIC_PIN_SCL;	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	IIC_SCL_ON;
	IIC_SDA_ON;
}

void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*168; 									//需要的节拍数 
	told=SysTick->VAL;        			//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;					//时间超过/等于要延迟的时间,则退出.
		}  
	};
}

void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_ON;
	IIC_SCL_ON;
	delay_us(10);
	
 	IIC_SDA_OFF;//START:when CLK is high,DATA change form high to low
	delay_us(10);
	IIC_SCL_OFF;//钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_OFF;
	IIC_SDA_OFF;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC_SCL_ON;
	IIC_SDA_ON;//发送I2C总线结束信号
	delay_us(10);
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入
	IIC_SDA_ON;
	delay_us(6);
	IIC_SCL_ON;
	delay_us(6);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_OFF;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_OFF;
	SDA_OUT();
	IIC_SDA_OFF;
	delay_us(10);
	IIC_SCL_ON;
	delay_us(10);
	IIC_SCL_OFF;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_OFF;
	SDA_OUT();
	IIC_SDA_OFF;
	delay_us(10);
	IIC_SCL_ON;
	delay_us(10);
	IIC_SCL_OFF;
}					 	

uint8_t I2C_CheckDevice(uint8_t address)//1成功 0失败
{
	uint8_t ack;
	IIC_Start();
	IIC_Send_Byte(address|0);
	ack=IIC_Wait_Ack();
	IIC_Stop();
	return ack;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	SDA_OUT(); 	    
	IIC_SCL_OFF;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_ON;
		else
			IIC_SDA_OFF;
		txd<<=1; 	  
		delay_us(10);   //对TEA5767这三个延时都是必须的
		IIC_SCL_ON;
		delay_us(10); 
		IIC_SCL_OFF;	
		delay_us(10);
	}	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		IIC_SCL_OFF;
		delay_us(10);
		IIC_SCL_ON;
		receive<<=1;
		if(READ_SDA)receive++;
		delay_us(10);
	}
	if (!ack)
		IIC_NAck();//发送nACK
	else
		IIC_Ack(); //发送ACK
	return receive;
}
