/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "sys.h"
#include "smoto.h" 
#include "key.h"   
#include "uart.h" 
#include "servor.h" 
#include "rubbish.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t data;
extern uint32_t Diff;//存放捕获插插值
extern uint8_t MeasureFlag;//测量完成标志位，0未完成，1完成
extern uint8_t CapIndex;//捕获指示，0未开始，1完成捕获
extern uint32_t CapVal1;//存放第一次捕获值
extern uint32_t CapVal2;//存放第二次捕获值
extern uint32_t Distance;//距离值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	delay_init(84);
	Servor_Init(97,50);
	Smoto_Init();
	DMA_Init();

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	Smoto_Control_Single(0,SPR*1,100,80,60,1);
//	Smoto_Control_Single(1,SPR*1,100,80,60,1);
//	Smoto_Control_Single(2,SPR*1,100,80,60,1);
//	Smoto_Control_Single(3,SPR*1,100,80,60,1);
	  Servor12_Control(180,180);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//进入中断
{
    if(htim->Instance==TIM10)
	{
		Servor_start();
	}
}
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
// 
//    __IO uint32_t tim_count = 0;														/* 用来保存定时器计数值 */
//    __IO uint32_t tmp = 0;																	/* 用来保存值 */
//    uint16_t new_step_delay = 0;                            /* 保存新（下）一个延时周期的变量 */
//    __IO static uint16_t last_accel_delay = 0;              /* 加速过程中最后一次延时（脉冲周期） */
//    __IO static uint32_t step_count = 0;                    /* 总移动步数计数器*/
//    __IO static int32_t rest = 0;                           /* 记录new_step_delay中的余数，提高下一步计算的精度 */
//    __IO static uint8_t i = 0;                              /* 因为定时器使用翻转模式，所以需要判断是否是两次中断，才能输出一个完整脉冲 */

//    if(htim->Instance==TIM3)
//    {
//       
//        tim_count = __HAL_TIM_GET_COUNTER(&htim3);
//        tmp = tim_count + A_smoto_control.step_delay/2;               /* 整个C值里边是需要翻转两次的所以需要除以2 */
//        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,tmp);
//        i++;                                                /* 定时器中断次数计数值 */
//        if(i == 2)                                          /* 2次，说明已经输出一个完整脉冲 */
//        {
//            i = 0;                                          /* 清零定时器中断次数计数值 */
//            switch(A_smoto_control.run_state)                         /* 加减速曲线阶段 */
//            {
//            case STOP:
//                step_count = 0;                             /* 清零步数计数器 */
//                rest = 0;                                   /* 清零余值 */
//                /* 关闭通道*/
//                HAL_TIM_OC_Stop_IT(&htim3,TIM_CHANNEL_1);
//                ST1_EN(EN_OFF);
//                g_motion_sta = 0;                           /* 电机为停止状态  */
//                break;

//            case ACCEL:
//                g_add_pulse_count++;                        /* 只用于记录相对位置转动了多少度 */
//                step_count++;                               /* 步数加1*/
//                if(A_smoto_control.dir == CW)
//                {
//                    g_step_position++;                      /* 绝对位置加1  记录绝对位置转动多少度*/
//                }
//                else
//                {
//                    g_step_position--;                      /* 绝对位置减1*/
//                }
//                A_smoto_control.accel_count++;                        /* 加速计数值加1*/
//                new_step_delay = A_smoto_control.step_delay - (((2 *A_smoto_control.step_delay) + rest)/(4 * A_smoto_control.accel_count + 1));/* 计算新(下)一步脉冲周期(时间间隔) */
//                rest = ((2 * A_smoto_control.step_delay)+rest)%(4 * A_smoto_control.accel_count + 1);                                /* 计算余数，下次计算补上余数，减少误差 */
//                if(step_count >= A_smoto_control.decel_start)         /* 检查是否到了需要减速的步数 */
//                {
//                    A_smoto_control.accel_count = A_smoto_control.decel_val;    /* 加速计数值为减速阶段计数值的初始值 */
//                    A_smoto_control.run_state = DECEL;                /* 下个脉冲进入减速阶段 */
//                }
//                else if(new_step_delay <= A_smoto_control.min_delay)  /* 检查是否到达期望的最大速度 计数值越小速度越快，当你的速度和最大速度相等或更快就进入匀速*/
//                {
//                    last_accel_delay = new_step_delay;      /* 保存加速过程中最后一次延时（脉冲周期）*/
//                    new_step_delay = A_smoto_control.min_delay;       /* 使用min_delay（对应最大速度speed）*/
//                    rest = 0;                               /* 清零余值 */
//                    A_smoto_control.run_state = RUN;                  /* 设置为匀速运行状态 */
//                }
//                break;

//            case RUN:
//                g_add_pulse_count++;
//                step_count++;                               /* 步数加1 */
//                if(A_smoto_control.dir == CW)
//                {
//                    g_step_position++;                      /* 绝对位置加1 */
//                }
//                else
//                {
//                    g_step_position--;                      /* 绝对位置减1*/
//                }
//                new_step_delay = A_smoto_control.min_delay;           /* 使用min_delay（对应最大速度speed）*/
//                if(step_count >= A_smoto_control.decel_start)         /* 需要开始减速 */
//                {
//                    A_smoto_control.accel_count = A_smoto_control.decel_val;    /* 减速步数做为加速计数值 */
//                    new_step_delay = last_accel_delay;      /* 加阶段最后的延时做为减速阶段的起始延时(脉冲周期) */
//                    A_smoto_control.run_state = DECEL;                /* 状态改变为减速 */
//                }
//                break;

//            case DECEL:
//                step_count++;                               /* 步数加1 */
//                g_add_pulse_count++;
//                if(A_smoto_control.dir == CW)
//                {
//                    g_step_position++;                      /* 绝对位置加1 */
//                }
//                else
//                {
//                    g_step_position--;                      /* 绝对位置减1 */
//                }
//                A_smoto_control.accel_count++;
//                new_step_delay = A_smoto_control.step_delay - (((2 * A_smoto_control.step_delay) + rest)/(4 * A_smoto_control.accel_count + 1));  /* 计算新(下)一步脉冲周期(时间间隔) */
//                rest = ((2 * A_smoto_control.step_delay)+rest)%(4 * A_smoto_control.accel_count + 1);                                   /* 计算余数，下次计算补上余数，减少误差 */

//                /* 检查是否为最后一步 */
//                if(A_smoto_control.accel_count >= 0)                  /* 判断减速步数是否从负值加到0是的话 减速完成 */
//                {
//                    A_smoto_control.run_state = STOP;
//                }
//                break;
//            }
//            A_smoto_control.step_delay = new_step_delay;              /* 为下个(新的)延时(脉冲周期)赋值 */
//        }
//    }
//}
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
// 
//    __IO uint32_t tim_count = 0;														/* 用来保存定时器计数值 */
//    __IO uint32_t tmp = 0;																	/* 用来保存值 */
//    uint16_t new_step_delay =             0;                            /* 保存新（下）一个延时周期的变量 */
//    __IO static uint16_t last_accel_delay = 0;              /* 加速过程中最后一次延时（脉冲周期） */
//    __IO static uint32_t step_count = 0;                    /* 总移动步数计数器*/
//    __IO static int32_t rest = 0;                           /* 记录new_step_delay中的余数，提高下一步计算的精度 */
//    __IO static uint8_t i = 0;                              /* 因为定时器使用翻转模式，所以需要判断是否是两次中断，才能输出一个完整脉冲 */

//    if(htim->Instance==TIM3)
//    {
//      if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
//			{
//				__IO uint16_t count1=0;
//				count1 =__HAL_TIM_GET_COUNTER(&htim3);
//				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,count1+Toggle_Pulse1);
//			}
//		  if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
//			{
//				__IO uint16_t count2=0;
//				count2 =__HAL_TIM_GET_COUNTER(&htim3);
//				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,count2+Toggle_Pulse2);
//			}
//			if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
//			{
//				__IO uint16_t count3=0;
//				count3 =__HAL_TIM_GET_COUNTER(&htim3);
//				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,count3+Toggle_Pulse3);
//			}
//    }
//}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
