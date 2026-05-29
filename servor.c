#include "servor.h"
#include "delay.h"
#include "math.h"

float Servor1_Current_Angle;//舵机x当前角度
float Servor1_Target_Angle;//舵机x目标角度
float Servor2_Current_Angle;//舵机y当前角度
float Servor2_Target_Angle;//舵机y目标角度
uint8_t Servor1_Step;//舵机x距离目标角度的步数
uint8_t Servor2_Step;//舵机y距离目标角度的步数
//static uint16_t Servor_Pulse;
uint8_t select;
void Servor_Reset(void)//仅在装配时使用
{
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,1500);//上舵机复位到90度
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,1500);//下舵机复位到90度
}
void Servor_Init(float Servor1_Angle,float Servor2_Angle)//装配成品之后的复位
{
	HAL_TIM_Base_Start(&htim9);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	static u16 Servor_Pulse[2];
	Servor_Pulse[0]=500+100*Servor1_Angle/9;//先乘后除，误差小
	Servor_Pulse[1]=500+100*Servor2_Angle/9;
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,Servor_Pulse[0]);//复位到？？度
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,Servor_Pulse[1]);
	delay_ms(500);
	Servor1_Current_Angle=Servor1_Angle;//舵机初始化之后的角度
	Servor2_Current_Angle=Servor2_Angle;
}

/**
 * @brief       舵机控制函数
 * @param:      一号舵机：Servor1_Angle
 * @param:      二号舵机：Servor2_Angle
 * @usage	      主函数里面
 * @
 */
void Servor12_Control(float Servor1_Angle,float Servor2_Angle)
{
//	if(Servor1_Angle>=35&&Servor1_Angle<=135&&Servor2_Angle>=55&&Servor2_Angle<=140)
//	{
		Servor1_Target_Angle=Servor1_Angle ;
		Servor2_Target_Angle=Servor2_Angle ;
		Servor1_Step =floor(fabs(Servor1_Target_Angle -Servor1_Current_Angle)) ;
		Servor2_Step =floor(fabs(Servor2_Target_Angle -Servor2_Current_Angle)) ;
		HAL_TIM_Base_Start_IT(&htim10);//十毫秒定时
//	}
}
/**
 * @brief       舵机启动函数
 * @param:      无
 * @retval      无
 * @usage	      放在中断回调函数里面
 */
void Servor_start(void)
{
	static uint16_t Servor1_Pulse,Servor2_Pulse;
		if(Servor1_Step >0)//说明舵机x有步数要走
		{
			if(Servor1_Target_Angle >Servor1_Current_Angle)
			{
				Servor1_Current_Angle ++;
			}
			else 
			{
				Servor1_Current_Angle --;
			}
				Servor1_Pulse=100*Servor1_Current_Angle/9+500;
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,Servor1_Pulse);
				Servor1_Step --;
			
		}
		if(Servor2_Step >0)//说明舵机y有步数要走
		{
			if(Servor2_Target_Angle >Servor2_Current_Angle)
			{
				Servor2_Current_Angle ++;
			}
			else 
			{
				Servor2_Current_Angle --;
			}
				Servor2_Pulse=100*Servor2_Current_Angle/9+500;
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,Servor2_Pulse);
				Servor2_Step --;
			
		}	
		if(Servor1_Step==0&&Servor2_Step==0)//当12舵机都无步数走时，结束中断
		{
			Servor1_Pulse=100*Servor1_Current_Angle/9+500;
			Servor2_Pulse=100*Servor2_Current_Angle/9+500;
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,Servor1_Pulse);
			__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,Servor2_Pulse);
			HAL_TIM_Base_Stop_IT(&htim10);
		}
		
}




