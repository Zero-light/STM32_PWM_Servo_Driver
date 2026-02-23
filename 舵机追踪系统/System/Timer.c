#include "ZHUIDIAN.h"
#include "stm32f10x.h" 
#include "PWM.h"
#include "zbsjcll.h"
#include "Delay.h"
#include <math.h>
#include <stdio.h>
#include "ZHUIDIAN.h"
#include "stm32f10x.h" 
#include "PWM.h"
#include "zbsjcll.h"
#include "Delay.h"
#include <math.h>
#include "Timer.h"

void Timer1_Init(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		
		TIM_InternalClockConfig(TIM1);
		
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;
		TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
		
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_Cmd(TIM1, ENABLE);
}


void TIM1_UP_IRQHandler(void)
{
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
			static uint16_t Count;		
		
		Count ++;		
		if (Count >=40)		
		{
			Count = 0;

			   XIA.Target = 320;
         SH.Target = 240;        
       Camera_GetXY(&XIA.Actual, &SH.Actual);
			XIA.Error0 =XIA.Target -XIA .Actual ;
			SH.Error0 =SH.Target -SH .Actual ;
			if(XIA.Error0>50){
            AddAngle_X(-2);
			}
			if(SH.Error0>50){
						AddAngle_Y(2);	
			}
			if(XIA.Error0<-50){
            AddAngle_X(2);
			}
			if(SH.Error0<-50){
						AddAngle_Y(-2);	
			}
	if(XIA.Error0<50&&XIA.Error0>20){
            AddAngle_X(-1);
			}
	if(SH.Error0<50&&SH.Error0>20){
						AddAngle_Y(1);	
			}
	if(XIA.Error0>-50&&XIA.Error0<-20){
            AddAngle_X(1);
			}
	if(SH.Error0>-50&&SH.Error0<-20){
						AddAngle_Y(-1);	
			}
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	}
}
	
