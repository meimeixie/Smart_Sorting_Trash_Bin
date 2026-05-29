#include "key.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
// NANO STM32F4开发板
//KEY驱动代码	   
									  
//////////////////////////////////////////////////////////////////////////////////

//按键初始化函数
//void KEY_Init(void)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
//    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
//    __HAL_RCC_GPIOD_CLK_ENABLE();           //开启GPIOD时钟
//    
//    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
//    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//    
//	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9; //PC8,9
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
//    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
//    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
//	
//	GPIO_Initure.Pin=GPIO_PIN_2; //PD2
//	HAL_GPIO_Init(GPIOD,&GPIO_Initure);
//    
//}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
//返回值：
//0，没有任何按键按下
//KEY0_PRES，KEY0按下
//KEY1_PRES，KEY1按下
//KEY2_PRES，KEY2按下
//WKUP_PRES，WK_UP按下 
//注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
uint8_t KEY_Scan(u8 mode)
{	 
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&(SW2 ==0||SW3 ==0||SW1 ==1))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(SW2 ==0)
			return SW2_PRES;
		else if(SW3 ==0)
			return SW3_PRES;
		else if(SW1 ==1)
			return SW1_PRES; 
	}else if(SW2 ==1&&SW3 ==1&&SW1 ==0)key_up=1; 	     
	return 0;// 无按键按下
}
