#include "uart.h"
#include "main.h" 
#include "delay.h"  
extern DMA_HandleTypeDef hdma_usart1_rx;
__IO uint8_t UART1_Rx_Count=0;
__IO uint8_t UART1_Rx_Flag;     /*接收完成标志:为0表示接受未完成，为1表示接收完成*/
__IO uint8_t UART1_Rx_Buf[uart1_rx_buf_length];
uint8_t data[1];
//__asm (".global __use_no_semihosting\n\t"); 
////避免使用半主机模式
//void _sys_exit(int x)
//{
//	x = x;
//}
////标准库需要支持的函数
//FILE __stdout; 

//#pragma import (__use_no_semihosting_swi)
/*标准库需要的支持函数，use_no_semihosting_swi以避免使用半主机模式*/
//struct __FILE
//{
//    int handle;
//};

//FILE __stdout;
//FILE __stdin;

////定义_sys_exit()以避免使用半主机模式
//void _sys_exit(int return_code){
//    //x = x;
//label:goto label;
//}

//重定向printf
int fputc(int ch, FILE *f)//printf
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,0xffff);  //发送一个字节的数据到你希望的串口
	return (ch);
}
//重定向scanf
int fgetc(FILE *f)
{
  uint8_t  ch;
	HAL_UART_Receive(&huart1,(uint8_t *)&ch, 1, 0xFFFF);
	return  ch;
}
/**
 * @brief       开启DMA接受函数，并且使能IDE中断
 * @param       无
 * @retval      无
 * @usage	      放在函数初始化里面
 */
void DMA_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)UART1_Rx_Buf,uart1_rx_buf_length);
}
/**
 * @brief       帧判别函数
 * @param:      UART1_Rx_Buf[0]帧头
 * @param:			UART1_Rx_Buf[1]功能码
 * @param:			UART1_Rx_Buf[2]帧尾
 * @retval      无
 * @usage	      放在主函数里面
 */
void Frame_Judge(void)
{
	if(UART1_Rx_Flag)
	{
		//清标志位
		UART1_Rx_Flag=0;
    //发生空闲中断时，已接收数据个数等于数据总量减去DMA接收的数据个数
		UART1_Rx_Count=uart1_rx_buf_length-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		//用DMA方式将数据发回pc
		HAL_UART_Transmit_DMA(&huart1,(uint8_t*)UART1_Rx_Buf,UART1_Rx_Count);
		//计数清零
		UART1_Rx_Count=0;
		//设置DMA_DISABLE,触发DMA中断，调用接收中断回调函数重启下一次DMA接收
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
    // 遍历数组，找到所有帧头和帧尾
		for (int i = 0; i <uart1_rx_buf_length - 2; i++) 
		{
			// 检查当前元素是否为帧头
			if (UART1_Rx_Buf[i] == FRAME_HEAD) 
			{// 从当前位置开始，查找帧尾
				for (int j = i + 2; j < 30; j++) 
				{ // 帧尾至少要在帧头后面2个位置
					if (UART1_Rx_Buf[j] == FRAME_TAIL) 
					{// 确保帧头和帧尾之间有足够的空间存放1个数据
						if (j - i ==2) 
						{// 将帧内数据存入全局变量
							data[0]=UART1_Rx_Buf[i+1];
							// 跳出循环，处理下一组数据
							i = j;
							break;
						}
					}
				}
			}
		}
		UART1_Rx_Buf[0]=0;
		UART1_Rx_Buf[2]=0;//清除头尾	
	}
}
/**
 * @brief       用户编写的Idle中断回调函数
 * @retval      无
 */
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//判断是不是串口1引起的串口接收中断
	{
		UART1_Rx_Flag=1;
	}
}
/**
 * @brief       串口DMA接收完成中断回调函数
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==USART1)//判断是不是串口1引起的串口接收中断
	{
		HAL_UART_Receive_DMA(&huart1,(uint8_t*)UART1_Rx_Buf,uart1_rx_buf_length);
	}
}

