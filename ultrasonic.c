#include "ultrasonic.h" 
#include "gpio.h"
#include "tim.h"
#include "delay.h"  
uint32_t Diff=0;//存放捕获插插值
uint8_t MeasureFlag=0;//测量完成标志位，0未完成，1完成
uint8_t CapIndex=0;//捕获指示，0未开始，1完成捕获
uint32_t CapVal1=0;//存放第一次捕获值
uint32_t CapVal2=0;//存放第二次捕获值
uint32_t Distance=0;//距离值
void Ultrasonic_Init(void)
{
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);//启动定时器3通道4的输入捕获功能
}

void Ultrasonic_Start(void)
{
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);
	delay_ms(45);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);
	while(1)
	{
		if(MeasureFlag==1)//测量完成
		{
			if(CapVal2>=CapVal1)//两次捕获在一个周期内
			{
				Diff=CapVal2-CapVal1;
			}
			else
			{
				Diff=((65535-CapVal1)+CapVal2);
			}
			Distance=Diff*34000/1000000/2;
			MeasureFlag=0;//清除标志位
			HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);//开启捕获
			break;
		}
		
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
		{
			if(CapIndex==0)
			{
				CapVal1=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_4);
				CapIndex=1;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4,TIM_ICPOLARITY_FALLING);
				//设定捕获通道的捕获边沿为下降沿触发
				MeasureFlag=1;
			}
			else if(CapIndex==1)
			{
				CapVal2=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_4);
				HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);//关闭定时器捕获
				CapIndex=0;
				__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4,TIM_ICPOLARITY_RISING);
				//设定捕获通道的捕获边沿为上升沿触发
				MeasureFlag=1;
			}
			else 
			{
				Error_Handler();
			}
				
		}
	}
}

