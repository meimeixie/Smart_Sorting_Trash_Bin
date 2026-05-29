#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 

// NANO STM32F4开发板
//KEY驱动代码	   
								  
//////////////////////////////////////////////////////////////////////////////////

//下面的方式是通过位带操作方式读取IO
//#define KEY0   PCin(8)   	
//#define KEY1   PCin(9)
//#define KEY2   PDin(2)	 
//#define WK_UP  PAin(0)	


//下面的方式是通过直接操作HAL库函数方式读取IO
#define SW1        HAL_GPIO_ReadPin(SW1_GPIO_Port ,SW1_Pin)  //KEY0按键PC8
#define SW2        HAL_GPIO_ReadPin(SW2_GPIO_Port ,SW2_Pin)  //KEY1按键PC9
#define SW3        HAL_GPIO_ReadPin(SW3_GPIO_Port ,SW3_Pin)  //KEY2按键PD2
//#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0

#define SW1_PRES	1	//KEY0  
#define SW2_PRES	2	//KEY1 
#define SW3_PRES	3	//KEY2
//#define WKUP_PRES	4	//WK_UP 

void KEY_Init(void);
u8 KEY_Scan(u8 mode);
#endif
