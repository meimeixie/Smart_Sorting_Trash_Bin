#include "stdio.h"
#include "usart.h"
#include "string.h"
#define uart1_rx_buf_length 10  
#define FRAME_HEAD 54
#define FRAME_TAIL 56
extern __IO uint8_t UART1_Rx_Count;
extern __IO uint8_t UART1_Rx_Flag;     //接收完成标志:为0表示接受未完成，为1表示接收完成
extern __IO uint8_t UART1_Rx_Buf[uart1_rx_buf_length];
/*函数声明*/
void Frame_Judge(void);
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart);
void DMA_Init(void);
