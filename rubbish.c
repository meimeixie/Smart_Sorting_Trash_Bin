#include "rubbish.h" 
#include "main.h"
#include "servor.h" 
#include "delay.h" 
#include "uart.h"
#include "ultrasonic.h"
extern uint8_t data;
/**
 * @brief       垃圾桶分类执行程序
 * @param:      无
 * @retval      无
 * @usage	      放在主函数里面
 */
void Rubbish_Start()
{
	if(data==49)//可回收垃圾
	{
		HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_3);
		Servor12_Control(60,30);
		delay_ms(500);
		Servor12_Control(97,30);
		delay_ms(37);
		Servor12_Control(97,50);
		delay_ms(20);
		data=0;
		UART1_Rx_Buf[1]=0;
		
		HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
//		printf("618");
	}
	if(data==50)//有害垃圾
	{
		HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_3);
		Servor12_Control(150,60);
		delay_ms(500);
		Servor12_Control(97,50);
		delay_ms(140);
		data=0;
		UART1_Rx_Buf[1]=0;
		HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
//		printf("628");
	}
	if(data==51)//其他垃圾
	{
		HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_3);
		Servor12_Control(97,128);
		delay_ms(500);
		Servor12_Control(150,128);
		delay_ms(500);
		Servor12_Control(97,50);
		delay_ms(500);
		data=0;
		UART1_Rx_Buf[1]=0;
		HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
//		printf("638");
	}
	if(data==52)//厨余垃圾
	{
		HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_3);
		Servor12_Control(97,170);
		delay_ms(500);
		Servor12_Control(40,170);
		delay_ms(500);
		Servor12_Control(97,50);
		delay_ms(500);
		data=0;
		UART1_Rx_Buf[1]=0;
		HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
	  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
//		printf("648");
	}
}


