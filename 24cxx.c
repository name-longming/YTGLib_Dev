#include "../YTGLib_Dev/YTGLib_Dev.h"

//初始化IIC接口
void AT24CXX_Init(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	IIC_Init(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);//IIC初始化
}
//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
uint8_t AT24CXX_ReadOneByte(uint16_t ReadAddr,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{				  
	uint8_t temp=0;		  	    																 
  
	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);  
	IIC_Send_Byte(0XA0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);   //发送器件地址0XA0,写数据 	   
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); 
  IIC_Send_Byte(ReadAddr,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);   //发送低地址
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);	    
	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);  	 	   
	IIC_Send_Byte(0XA1,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);           //进入接收模式			   
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);	 
  temp=IIC_Read_Byte(0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);		   
  IIC_Stop(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);//产生一个停止条件	
//	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(0xA0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(ReadAddr,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(0xA1,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	temp=IIC_Read_Byte(0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); //读出地址中的值
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(1,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Stop(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(uint16_t WriteAddr,uint8_t DataToWrite,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{				   	  	    																 
	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);  
	IIC_Send_Byte(0XA0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);   //发送器件地址0XA0,写数据 	 
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);	   
	IIC_Send_Byte(WriteAddr,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);   //发送低地址
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); 	 										  		   
	IIC_Send_Byte(DataToWrite,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);     //发送字节							   
	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);  		    	   
	IIC_Stop(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);//产生一个停止条件 
	delay_ms(10);

//	IIC_Start(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(0xA0,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); //器件地址，0为写操作 
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(WriteAddr,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); //要存的地址
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Send_Byte(DataToWrite,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); //要存的数据
//	IIC_Wait_Ack(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	IIC_Stop(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
//	delay_ms(10);		
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(uint16_t WriteAddr,uint32_t DataToWrite,uint8_t Len,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{  	
	uint8_t t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
uint32_t AT24CXX_ReadLenByte(uint16_t ReadAddr,uint8_t Len,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{  	
	uint8_t t;
	uint32_t temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
uint8_t AT24CXX_Check(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	uint8_t temp;
	temp=AT24CXX_ReadOneByte(255,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
	    temp=AT24CXX_ReadOneByte(255,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer,GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
		WriteAddr++;
		pBuffer++;
	}
}
