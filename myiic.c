#include "../YTGLib_Dev/YTGLib_Dev.h"

//IIC初始化
void IIC_Init(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
 
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA);
}

//产生IIC起始信号
void IIC_Start(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=1<<GPIO_Pin_SDA*2;
	
	GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA);  	  
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	delay_us(4);
 	GPIO_SDA->BSRR |= 1<<(GPIO_Pin_SDA+16);
	delay_us(4);
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
}	  
//产生IIC停止信号
void IIC_Stop(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=1<<GPIO_Pin_SDA*2;
	
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
	GPIO_SDA->BSRR |= 1<<(GPIO_Pin_SDA+16);
 	delay_us(4);
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA); 
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	uint8_t ucErrTime=0;
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=0<<GPIO_Pin_SDA*2; 
	
	GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA);
	delay_us(1);	   
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	delay_us(1);	 
	while(GPIO_SDA->IDR & 1<<GPIO_Pin_SDA) //HAL_GPIO_ReadPin(GPIO_SDA,(uint16_t)0x0001<<GPIO_Pin_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(GPIO_SCL,GPIO_Pin_SCL,GPIO_SDA,GPIO_Pin_SDA);
			return 1;
		}
	}
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
	
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=1<<GPIO_Pin_SDA*2;
	
	GPIO_SDA->BSRR |= 1<<(GPIO_Pin_SDA+16);
	delay_us(2);
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	delay_us(2);
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
}
//不产生ACK应答		    
void IIC_NAck(GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=1<<GPIO_Pin_SDA*2;
	
	GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA);
	delay_us(2);
	GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL);
	delay_us(2);
	GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(uint8_t txd,GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{                        
  uint8_t t; 

	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=1<<GPIO_Pin_SDA*2;
	
  GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
	
	for(t=0;t<8;t++)
	{              
		if(((txd&0x80)>>7) != 0)
		{
			GPIO_SDA->BSRR |= (1<<GPIO_Pin_SDA);
			
		}else{
			GPIO_SDA->BSRR |= 1<<(GPIO_Pin_SDA+16);
		}

		txd<<=1; 	  
		delay_us(2);
		GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL); 
		delay_us(2); 
		GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16); 	
		delay_us(2);
	}	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char ack, GPIO_TypeDef* GPIO_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIO_SDA, uint16_t GPIO_Pin_SDA)
{
	unsigned char i,receive=0;
	GPIO_SDA->MODER&=~(3<<(GPIO_Pin_SDA*2));
	GPIO_SDA->MODER|=0<<GPIO_Pin_SDA*2; 
	for(i=0;i<8;i++ )
	{
		GPIO_SCL->BSRR |= 1<<(GPIO_Pin_SCL+16);
		delay_us(2);
		GPIO_SCL->BSRR |= (1<<GPIO_Pin_SCL); 
		receive<<=1;
		if(GPIO_SDA->IDR & 1<<GPIO_Pin_SDA)receive++;   
		delay_us(1); 
	}					 
	if (!ack)
		IIC_NAck(GPIO_SCL, GPIO_Pin_SCL, GPIO_SDA, GPIO_Pin_SDA);//发送nACK
	else
		IIC_Ack(GPIO_SCL, GPIO_Pin_SCL, GPIO_SDA, GPIO_Pin_SDA); //发送ACK   
	return receive;
}


