#include "iic.h" 

typedef enum{
	MODE_IIC = 0,
	MODE_USART = 1
}WorkingMode_e;

typedef struct __Ks103_t{
	uint8_t Command[3];
	uint8_t address;
	uint16_t distance;
	UART_HandleTypeDef* huart;
	uint8_t* ReceiveData;
	void (*KS103_SensorInit)(struct __Ks103_t *ks103);
	void (*KS103_ParaInit)(struct __Ks103_t *ks103, uint8_t address, UART_HandleTypeDef* huart, uint8_t* usartData);
	void (*KS103_DataDemand)(struct __Ks103_t *ks103);	
	void (*KS103_DataProcess)(struct __Ks103_t *ks103);
}Ks103_t;

//extern Ks103_t Ks103_Front;
//extern Ks103_t Ks103_Left;
//extern Ks103_t Ks103_Right;

/*USART模式*/
void KS103_SensorInit(Ks103_t *ks103);
void KS103_ParaInit(Ks103_t *ks103, uint8_t address, UART_HandleTypeDef* huart, uint8_t* usartData);
void KS103_DataDemand(Ks103_t *ks103);			//串口指令发送
void KS103_DataProcess(Ks103_t *ks103);		//串口数据读取

/*IIC模式*/
void KS103_WriteOneByte(u16 WriteAddr,u8 Num,u8 command);
uint8_t KS103_ReadOneByte(u16 ReadAddr,u8 DataToRead);
void Change_Addr(u8 Original_Addr,u8 Now_Addr);
void Return_distance(u16 I2C_Addr,u16 Mode,u16 *DISTANCE);	//IIc读取函数
