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
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
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

#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68


////因为开发板接GND,所以转为读写地址后,为0XD1和0XD0(如果接GND,则为0XD3和0XD2)  
//#define MPU_READ    0XD1
//#define MPU_WRITE   0XD0

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
