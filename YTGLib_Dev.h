#ifndef _YTGLib_Dev_H
#define _YTGLib_Dev_H  


/*********************选择你要开启的模块***********************/

#define OLED_INIT_IIC1		//初始化OLED,并选中为IIC1驱动(硬件iic)
#define DELAY_US  				//初始化微妙延时
#define MYIIC							//初始化模拟IIC
#define MPU6050						//初始化MPU6050

/**************************************************************/


#ifdef STM32F407xx
#include "stm32f4xx_hal.h"
#define HCLKTICK 84 
#elif STM32F103xx
#include "stm32f1xx_hal.h"
#define HCLKTICK 72 
#elif STM32H7xx
#include "stm32h7xx_hal.h"
#endif

#ifdef DELAY_US
void delay_init(void);
void delay_us(uint32_t usDelay);
void HAL_Delay(uint32_t Delay);
#endif

#ifdef OLED_INIT_IIC1
#include "i2c.h"

#define IIC_ADRESS &hi2c1
#define OLED0561_ADD	0x78  // OLED的I2C地址（禁止修改）
#define COM				0x00  // OLED 指令（禁止修改）
#define DAT 			0x40  // OLED 数据（禁止修改）
void WriteCmd(unsigned char I2C_Command);//写命令
void WriteDat(unsigned char I2C_Data);//写数据
void OLED_Init(void);//初始化
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);//全屏填充
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//显示字符串
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);//显示汉字
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//显示图片

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2);//size2(16|12)
#elif OLED_INIT_IIC2
#include "i2c.h"

#define IIC_ADRESS &hi2c2
#define OLED0561_ADD	0x78  // OLED的I2C地址（禁止修改）
#define COM				0x00  // OLED 指令（禁止修改）
#define DAT 			0x40  // OLED 数据（禁止修改）
void WriteCmd(unsigned char I2C_Command);//写命令
void WriteDat(unsigned char I2C_Data);//写数据
void OLED_Init(void);//初始化
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);//全屏填充
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);//显示字符串
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);//显示汉字
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);//显示图片

void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size2);//size2(16|12)
#endif

/*********************************************************************/
/*						仅需要初始化一次	
如：IIC_Init(GPIOB,8,GPIOB,9); //IIC初始化,GPIOB8作为SCL,GPIOB9作为SDA
		IIC_Start();//发送IIC开始信号											 
		mpu_dmp_get_data(&pitch,&roll,&yaw);//得到MPU6050欧拉角					 */
/*********************************************************************/

#ifdef MYIIC

extern GPIO_TypeDef* GPIO_SCL;
extern uint16_t GPIO_Pin_SCL;
extern GPIO_TypeDef* GPIO_SDA;
extern uint16_t GPIO_Pin_SDA;

void IIC_Init(GPIO_TypeDef* GPIO_SCL_Init, uint16_t GPIO_Pin_SCL_Init, GPIO_TypeDef* GPIO_SDA_Init, uint16_t GPIO_Pin_SDA_Init);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len);//指定地址开始写入指定长度的数据
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);   	//从指定地址开始读出指定长度的数据

uint8_t AT24CXX_Check(void);  //检查器件
void AT24CXX_Init(GPIO_TypeDef* GPIO_SCL_Init, uint16_t GPIO_Pin_SCL_Init, GPIO_TypeDef* GPIO_SDA_Init, uint16_t GPIO_Pin_SDA_Init); //初始化IIC
#endif

#ifdef MPU6050

/*********************************************************************/
/*				仅需要初始化一次，且使用陀螺仪前必须要初始化
			当陀螺仪初始化后，可直接调用mpu_dmp_init()进行DMP初始化
如：MPU_Init(GPIOB,8,GPIOB,9); //陀螺仪初始化，PB8为SCL，PB9为SDA		*/
/*********************************************************************/

#include "../YTGLib_Dev/eMPL/inv_mpu.h"
#include "../YTGLib_Dev/eMPL/inv_mpu_dmp_motion_driver.h" 

uint8_t MPU_Init(GPIO_TypeDef* GPIO_SCL_Init, uint16_t GPIO_Pin_SCL_Init, GPIO_TypeDef* GPIO_SDA_Init, uint16_t GPIO_Pin_SDA_Init); 								//初始化MPU6050
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//IIC连续写
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf); //IIC连续读 
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);				//IIC写一个字节
uint8_t MPU_Read_Byte(uint8_t reg);						//IIC读一个字节

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);


short MPU_Get_Temperature(void);			//得到MPU6050温度
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);	//得到陀螺仪值
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);	//得到加速度值
#endif

#endif
