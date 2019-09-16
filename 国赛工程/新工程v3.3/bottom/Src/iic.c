#include "iic.h"
#include "gpio.h"

void IIC_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	/*IIC����IO��PE4/PE5*/
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
	u32 reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*168; 									//��Ҫ�Ľ����� 
	told=SysTick->VAL;        			//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;					//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
}

void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_ON;
	IIC_SCL_ON;
	delay_us(10);
	
 	IIC_SDA_OFF;//START:when CLK is high,DATA change form high to low
	delay_us(10);
	IIC_SCL_OFF;//ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_OFF;
	IIC_SDA_OFF;//STOP:when CLK is high DATA change form low to high
 	delay_us(10);
	IIC_SCL_ON;
	IIC_SDA_ON;//����I2C���߽����ź�
	delay_us(10);
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����
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
	IIC_SCL_OFF;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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

uint8_t I2C_CheckDevice(uint8_t address)//1�ɹ� 0ʧ��
{
	uint8_t ack;
	IIC_Start();
	IIC_Send_Byte(address|0);
	ack=IIC_Wait_Ack();
	IIC_Stop();
	return ack;
}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	SDA_OUT(); 	    
	IIC_SCL_OFF;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_ON;
		else
			IIC_SDA_OFF;
		txd<<=1; 	  
		delay_us(10);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_ON;
		delay_us(10); 
		IIC_SCL_OFF;	
		delay_us(10);
	}	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
		IIC_NAck();//����nACK
	else
		IIC_Ack(); //����ACK
	return receive;
}
