#include "stm32f10x.h"                  // Device header
#include "Serial.h"

void Camera_GetXY(float  *X, float  *Y )
{
	
	uint16_t y1,y2,y3,x1,x2,x3;
	USART_Cmd(USART1, ENABLE);	
	while (Serial_RxFlag == 0)
	{
	}
	x1=Serial_RxPacket[0];
	 x2=Serial_RxPacket[1];
	 x3=Serial_RxPacket[2];
	 *X=(x1-48)*100+(x2-48)*10+(x3-48);
	 y1=Serial_RxPacket[3];
	 y2=Serial_RxPacket[4];
	 y3=Serial_RxPacket[5];
	 *Y=(y1-48)*100+(y2-48)*10+(y3-48);
	USART_Cmd(USART1, DISABLE);
	  Serial_RxFlag = 0;		
}


