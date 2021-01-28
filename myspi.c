#include "../YTGLib_Dev/YTGLib_Dev.h"

#ifdef __spi_H

SPI_HandleTypeDef SPIx;
GPIO_TypeDef* GPIO_CS;
uint16_t GPIO_Pin_CS;

void SPIx_Select_Init(SPI_HandleTypeDef hspix,GPIO_TypeDef* GPIO_CS_Init, uint16_t GPIO_Pin_CS_Init)
{
	SPIx = hspix;	
	GPIO_CS=GPIO_CS_Init;
	GPIO_Pin_CS=GPIO_Pin_CS_Init;

	SPIx_CS_SetStation(GPIO_PIN_SET);
}

void SPIx_CS_SetStation(GPIO_PinState GPIO_PIN_State)
{
	if(GPIO_PIN_State == GPIO_PIN_RESET)
	{
		GPIO_CS->BSRR |= 1<<(GPIO_Pin_CS+16);
	}
	else
	{
		GPIO_CS->BSRR |= 1<<GPIO_Pin_CS;
	}
}

uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
  uint8_t Rxdata;
  HAL_SPI_TransmitReceive(&hspi1,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}
#endif
