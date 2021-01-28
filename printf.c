#include "../YTGLib_Dev/YTGLib_Dev.h"

int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0); 
	USART1->DR = (uint8_t) ch;      
	return ch;
}
