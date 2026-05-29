#include "tim.h"
#include "main.h" 
#include "smoto.h" 
#include "delay.h"  
#include "math.h"

__IO uint8_t Stop_or_not[4]={0,0,0,0};       /* 全局标志，是否需要一直运行 */
__IO uint16_t tim_count;        /* 达到最大速度时的计数 */
__IO uint32_t max_s_lim;        /* 达到最大速度的步数，如果没有达到最大速度，该值为0 */
__IO uint32_t accel_lim;        /* 减速步数 */
__IO uint16_t SMOTO_CH[4]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};/* 电机对应通道，总共四个 */
speedRampData smoto_control[SMOTO_NUM] = {
    {STOP, CR, 0, 0, 0, 0, 0},
    {STOP, CR, 0, 0, 0, 0, 0},
    {STOP, CR, 0, 0, 0, 0, 0},
    {STOP, CR, 0, 0, 0, 0, 0}
};  /* 电机速度控制数组 */

__IO int32_t  Step_Position[4]     = {0,0,0,0};                    /* 当前位置数组 */
__IO uint8_t  Motion_State[4]      = {0,0,0,0};                    /* 是否正在运行数组，0停止，1运行 */
__IO uint32_t Add_Pulse_Count[4]   = {0,0,0,0};                    /* 脉冲计数数组 */
__IO uint32_t Target_Steps[4]      = {0,0,0,0};                    /* 每个电机的目标步数数组 */

/*******************************************************************************************************************************************************/
/**
 * @brief       初始化步进电机控制
 * @param       无
 * @retval      无
 */
void Smoto_Init(void)
{
    HAL_TIM_Base_Start(&htim3);
}

/**
 * @brief       启动单个电机
 * @param       motor_num: 电机编号
 * @retval      无
 */
void Smoto_Start_Single(uint8_t motor_num)
{
    switch (motor_num)             
    {
		case 0:
			ST1_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
		break;

        case 1:
			ST2_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
        break;
		
		case 2:
			ST3_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
        break;

        case 3:
			ST4_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_4);
        break;
	}
}

/**
 * @brief       停止单个电机
 * @param       motor_num: 电机编号
 * @retval      无
 */
void Smoto_Stop_Single(uint8_t motor_num)
{
    switch(motor_num)
    {
        case 0:
            HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
			ST1_EN(EN_OFF);
            break;
        case 1:
            HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_2);
			ST2_EN(EN_OFF);
            break;
        case 2:
            HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_3);
			ST3_EN(EN_OFF);
            break;
        case 3:
            HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_4);
			ST4_EN(EN_OFF);
            break;
        default:
            break;
    }		
}
/**
 * @brief       控制单个电机运动
 * @param       motor_num: 电机编号
 * @param       step: 电机要移动的步数
 * @param       accel: 加速度
 * @param       decel: 减速度
 * @param       speed: 最大速度
 * @param       stop_or_not: 电机到达总步数后是否停止，0：停止，1：一直运动
 * @retval      无
 */
void Smoto_Control_Single(uint8_t motor_num, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed, uint8_t stop_or_not)
{
	if (Motion_State[motor_num] != STOP)
		return;
    if (step < 0) 
	{
        smoto_control[motor_num].dir = CCR;
		switch (motor_num)              /* 加减速曲线阶段 */
		{
			case 0:
				ST1_DIR(CCR);
			break;

			case 1:
				ST2_DIR(CCR);
			break;
			
			case 2:
				ST3_DIR(CCR);
			break;

			case 3:
				ST4_DIR(CCR);
			break;
		}
        step = -step;
    } 
	else 
	{
        smoto_control[motor_num].dir = CR;
        switch (motor_num)              
		{
			case 0:
				ST1_DIR(CR);
			break;

			case 1:
				ST2_DIR(CR);
			break;
			
			case 2:
				ST3_DIR(CR);
			break;

			case 3:
				ST4_DIR(CR);
			break;
		}
    }

    if (step == 1) 
	{
        smoto_control[motor_num].accel_count = -1;
        smoto_control[motor_num].run_state = DECEL;
        smoto_control[motor_num].step_delay = 1000;
    } 
	else if (step != 0) 
	{
        smoto_control[motor_num].min_delay = (int32_t)(A_T_x10 / speed);
        smoto_control[motor_num].step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ /accel))/10);
        max_s_lim = (uint32_t)(speed * speed/(A_x200 * accel/10));
        accel_lim = (uint32_t)(step * decel/(accel+decel));

        if (max_s_lim == 0) 
		{
            max_s_lim = 1;
        }
        if (accel_lim == 0) 
		{
            accel_lim = 1;
        }

        if (accel_lim <= max_s_lim) 
		{
            smoto_control[motor_num].decel_val = accel_lim - step;
        } 
		else 
		{
            smoto_control[motor_num].decel_val = -(max_s_lim * accel / decel);
        }

        if (smoto_control[motor_num].decel_val == 0) 
		{
            smoto_control[motor_num].decel_val = -1;
        }
        smoto_control[motor_num].decel_start = step + smoto_control[motor_num].decel_val;

        if (smoto_control[motor_num].step_delay <= smoto_control[motor_num].min_delay) 
		{
            smoto_control[motor_num].step_delay = smoto_control[motor_num].min_delay;
            smoto_control[motor_num].run_state = RUN;
        } 
		else 
		{
            smoto_control[motor_num].run_state = ACCEL;
        }
        smoto_control[motor_num].accel_count = 0;

        if (stop_or_not == 1) 
		{
            Stop_or_not[motor_num] = 1;
        } 
		else 
		{
            Stop_or_not[motor_num] = 0;
        }
    }

    Motion_State[motor_num] = 1;
	switch (motor_num)              /* 加减速曲线阶段 */
    {
		case 0:
			ST1_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);
		break;

        case 1:
			ST2_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_2);
        break;
		
		case 2:
			ST3_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_3);
        break;

        case 3:
			ST4_EN(EN_ON);
			HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_4);
        break;
	}
	
}


/**
  * @brief  定时器比较中断
  * @param  htim：定时器句柄指针
  * @note   无
  * @retval 无
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    __IO uint32_t tim_count = 0;                             /* 用来保存定时器计数值 */
    __IO uint32_t tmp[4] = {0,0,0,0};                      /* 用来保存修改过的arr值 */
    uint16_t new_step_delay[4] = {0,0,0,0};                /* 保存新（下）一个延时周期的变量 */
    __IO static uint16_t last_accel_delay[4] = {0,0,0,0};  /* 加速过程中最后一次延时（脉冲周期） */
    __IO static uint32_t step_count[4] = {0,0,0,0};        /* 总移动步数计数器 */
    __IO static int32_t rest[4] = {0,0,0,0};               /* 记录new_step_delay中的余数，提高下一步计算的精度 */
    __IO static uint8_t i[4] = {0,0,0,0};                               /* 因为定时器使用翻转模式，所以需要判断是否是两次中断，才能输出一个完整脉冲 */
    __IO static uint8_t n = 0;                               /* 判断电机型号 */

    if (htim->Instance == TIM3)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			tim_count = __HAL_TIM_GET_COUNTER(&htim3);
			tmp[0] = tim_count + smoto_control[0].step_delay / 2;    /* 整个C值里边是需要翻转两次的所以需要除以2 */
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, tmp[0]);
			i[0]++;                                                     /* 定时器中断次数计数值 */
			if (i[0]== 2)                                              /* 2次，说明已经输出一个完整脉冲 */
			{
				i[0]= 0;                                               /* 清零定时器中断次数计数值 */
				switch (smoto_control[0].run_state)              /* 加减速曲线阶段 */
				{
					case STOP:
						step_count[0] = 0;                       /* 清零步数计数器 */
						rest[0] = 0;                             /* 清零余值 */
						Smoto_Stop_Single(0);
						Motion_State[0] = 0;                     /* 电机为停止状态  */
						break;

					case ACCEL:
						Add_Pulse_Count[0]++;                    /* 只用于记录相对位置转动了多少度 */
						step_count[0]++;                         /* 步数加1 */
						if (smoto_control[0].dir == CR)
						{
							Step_Position[0]++;                  /* 绝对位置加1  记录绝对位置转动多少度 */
						}
						else 
						{
							Step_Position[0]--;                  /* 绝对位置减1 */
						}
						smoto_control[0].accel_count++;          /* 加速计数值加1 */
						new_step_delay[0] = smoto_control[0].step_delay - (((2 * smoto_control[0].step_delay) + rest[0]) / (4 * smoto_control[0].accel_count + 1));
						rest[0] = ((2 * smoto_control[0].step_delay) + rest[0]) % (4 * smoto_control[0].accel_count + 1);
						/* 检查是否到了需要减速的步数 */
						if (step_count[0] >= smoto_control[0].decel_start)
						{
							if (Stop_or_not[0] == 1)
							{
								last_accel_delay[0] = new_step_delay[0];
								new_step_delay[0] = smoto_control[0].min_delay;
								rest[0] = 0;
								smoto_control[0].run_state = RUN;    /* 下个脉冲进入匀速阶段 */
							}
							else 
							{
								smoto_control[0].accel_count = smoto_control[0].decel_val;
								smoto_control[0].run_state = DECEL;  /* 下个脉冲进入减速阶段 */
							}
						}
						else if (new_step_delay[0] <= smoto_control[0].min_delay)
						{
							last_accel_delay[0] = new_step_delay[0];
							new_step_delay[0] = smoto_control[0].min_delay;
							rest[0] = 0;
							smoto_control[0].run_state = RUN;        /* 设置为匀速运行状态 */
						}
						break;

					case RUN:
						Add_Pulse_Count[0]++;
						step_count[0]++;                             /* 步数加1 */
						if (smoto_control[0].dir == CR)
						{
							Step_Position[0]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[0]--;                      /* 绝对位置减1 */
						}
						new_step_delay[0] = smoto_control[0].min_delay;
						if (step_count[0] >= smoto_control[0].decel_start)
						{
							if (Stop_or_not[0]!=1)
							{
								smoto_control[0].accel_count = smoto_control[0].decel_val;
								new_step_delay[0] = last_accel_delay[0];
								smoto_control[0].run_state = DECEL;    /* 状态改变为匀速 */
							}         
						}
						break;

					case DECEL:
						step_count[0]++;                             /* 步数加1 */
						Add_Pulse_Count[0]++;
						if (smoto_control[0].dir == CR)
						{
							Step_Position[0]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[0]--;                      /* 绝对位置减1 */
						}
						smoto_control[0].accel_count++;
						new_step_delay[0] = smoto_control[0].step_delay - (((2 * smoto_control[0].step_delay) + rest[0]) / (4 * smoto_control[0].accel_count + 1));
						rest[0] = ((2 * smoto_control[0].step_delay) + rest[0]) % (4 * smoto_control[0].accel_count + 1);
						/* 检查是否为最后一步 */
						if (smoto_control[0].accel_count >= 0 )
						{
							smoto_control[0].run_state = STOP;
						}
						break;
				}
					smoto_control[0].step_delay = new_step_delay[0];     /* 为下个(新的)延时(脉冲周期)*/
			}			
		}
            
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{

			tim_count = __HAL_TIM_GET_COUNTER(&htim3);
			tmp[1] = tim_count + smoto_control[1].step_delay / 2;    /* 整个C值里边是需要翻转两次的所以需要除以2 */
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2, tmp[1]);
			i[1]++;                                                     /* 定时器中断次数计数值 */
			if (i[1]== 2)                                              /* 2次，说明已经输出一个完整脉冲 */
			{
				i[1] = 0;                                               /* 清零定时器中断次数计数值 */
				switch (smoto_control[1].run_state)              /* 加减速曲线阶段 */
				{
					case STOP:
						step_count[1] = 0;                       /* 清零步数计数器 */
						rest[1] = 0;                             /* 清零余值 */
						Smoto_Stop_Single(1);
						Motion_State[1] = 0;                     /* 电机为停止状态  */
						break;

					case ACCEL:
						Add_Pulse_Count[1]++;                    /* 只用于记录相对位置转动了多少度 */
						step_count[1]++;                         /* 步数加1 */
						if (smoto_control[1].dir == CR)
						{
							Step_Position[1]++;                  /* 绝对位置加1  记录绝对位置转动多少度 */
						}
						else 
						{
							Step_Position[1]--;                  /* 绝对位置减1 */
						}
						smoto_control[1].accel_count++;          /* 加速计数值加1 */
						new_step_delay[1] = smoto_control[1].step_delay - (((2 * smoto_control[1].step_delay) + rest[1]) / (4 * smoto_control[1].accel_count + 1));
						rest[1] = ((2 * smoto_control[1].step_delay) + rest[1]) % (4 * smoto_control[1].accel_count + 1);
						/* 检查是否到了需要减速的步数 */
						if (step_count[1] >= smoto_control[1].decel_start)
						{
							if (Stop_or_not[1] == 1)
							{
								last_accel_delay[1] = new_step_delay[1];
								new_step_delay[1] = smoto_control[1].min_delay;
								rest[1] = 0;
								smoto_control[1].run_state = RUN;    /* 下个脉冲进入匀速阶段 */
							}
							else 
							{
								smoto_control[1].accel_count = smoto_control[1].decel_val;
								smoto_control[1].run_state = DECEL;  /* 下个脉冲进入减速阶段 */
							}
						}
						else if (new_step_delay[1] <= smoto_control[1].min_delay)
						{
							last_accel_delay[1] = new_step_delay[1];
							new_step_delay[1] = smoto_control[1].min_delay;
							rest[1] = 0;
							smoto_control[1].run_state = RUN;        /* 设置为匀速运行状态 */
						}
						break;

					case RUN:
						Add_Pulse_Count[1]++;
						step_count[1]++;                             /* 步数加1 */
						if (smoto_control[1].dir == CR)
						{
							Step_Position[1]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[1]--;                      /* 绝对位置减1 */
						}
						new_step_delay[1] = smoto_control[1].min_delay;
						if (step_count[1] >= smoto_control[1].decel_start)
						{
							if (Stop_or_not[1]!=1)
							{
								smoto_control[1].accel_count = smoto_control[1].decel_val;
								new_step_delay[1] = last_accel_delay[1];
								smoto_control[1].run_state = DECEL;    /* 状态改变为匀速 */
							}         
						}
						break;

					case DECEL:
						step_count[1]++;                             /* 步数加1 */
						Add_Pulse_Count[1]++;
						if (smoto_control[1].dir == CR)
						{
							Step_Position[1]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[1]--;                      /* 绝对位置减1 */
						}
						smoto_control[1].accel_count++;
						new_step_delay[1] = smoto_control[1].step_delay - (((2 * smoto_control[1].step_delay) + rest[1]) / (4 * smoto_control[1].accel_count + 1));
						rest[1] = ((2 * smoto_control[1].step_delay) + rest[1]) % (4 * smoto_control[1].accel_count + 1);
						/* 检查是否为最后一步 */
						if (smoto_control[1].accel_count >= 0 )
						{
							smoto_control[1].run_state = STOP;
						}
						break;
				}
					smoto_control[1].step_delay = new_step_delay[1];     /* 为下个(新的)延时(脉冲周期)*/
			}
		}
           
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
		{

			tim_count = __HAL_TIM_GET_COUNTER(&htim3);
			tmp[2] = tim_count + smoto_control[2].step_delay / 2;    /* 整个C值里边是需要翻转两次的所以需要除以2 */
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3, tmp[2]);
			i[2]++;                                                     /* 定时器中断次数计数值 */
			if (i[2]== 2)                                              /* 2次，说明已经输出一个完整脉冲 */
			{
				i[2]= 0;                                               /* 清零定时器中断次数计数值 */
				switch (smoto_control[2].run_state)              /* 加减速曲线阶段 */
				{
					case STOP:
						step_count[2] = 0;                       /* 清零步数计数器 */
						rest[2] = 0;                             /* 清零余值 */
						Smoto_Stop_Single(2);
						Motion_State[2] = 0;                     /* 电机为停止状态  */
						break;

					case ACCEL:
						Add_Pulse_Count[2]++;                    /* 只用于记录相对位置转动了多少度 */
						step_count[2]++;                         /* 步数加1 */
						if (smoto_control[2].dir == CR)
						{
							Step_Position[2]++;                  /* 绝对位置加1  记录绝对位置转动多少度 */
						}
						else 
						{
							Step_Position[2]--;                  /* 绝对位置减1 */
						}
						smoto_control[2].accel_count++;          /* 加速计数值加1 */
						new_step_delay[2] = smoto_control[2].step_delay - (((2 * smoto_control[2].step_delay) + rest[2]) / (4 * smoto_control[2].accel_count + 1));
						rest[2] = ((2 * smoto_control[2].step_delay) + rest[2]) % (4 * smoto_control[2].accel_count + 1);
						/* 检查是否到了需要减速的步数 */
						if (step_count[2] >= smoto_control[2].decel_start)
						{
							if (Stop_or_not[2] == 1)
							{
								last_accel_delay[2] = new_step_delay[2];
								new_step_delay[2] = smoto_control[2].min_delay;
								rest[2] = 0;
								smoto_control[2].run_state = RUN;    /* 下个脉冲进入匀速阶段 */
							}
							else 
							{
								smoto_control[2].accel_count = smoto_control[2].decel_val;
								smoto_control[2].run_state = DECEL;  /* 下个脉冲进入减速阶段 */
							}
						}
						else if (new_step_delay[2] <= smoto_control[2].min_delay)
						{
							last_accel_delay[2] = new_step_delay[2];
							new_step_delay[2] = smoto_control[2].min_delay;
							rest[2] = 0;
							smoto_control[2].run_state = RUN;        /* 设置为匀速运行状态 */
						}
						break;

					case RUN:
						Add_Pulse_Count[2]++;
						step_count[2]++;                             /* 步数加1 */
						if (smoto_control[2].dir == CR)
						{
							Step_Position[2]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[2]--;                      /* 绝对位置减1 */
						}
						new_step_delay[2] = smoto_control[2].min_delay;
						if (step_count[2] >= smoto_control[2].decel_start)
						{
							if (Stop_or_not[2]!=1)
							{
								smoto_control[2].accel_count = smoto_control[2].decel_val;
								new_step_delay[2] = last_accel_delay[2];
								smoto_control[2].run_state = DECEL;    /* 状态改变为匀速 */
							}         
						}
						break;

					case DECEL:
						step_count[2]++;                             /* 步数加1 */
						Add_Pulse_Count[2]++;
						if (smoto_control[2].dir == CR)
						{
							Step_Position[2]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[2]--;                      /* 绝对位置减1 */
						}
						smoto_control[2].accel_count++;
						new_step_delay[2] = smoto_control[2].step_delay - (((2 * smoto_control[2].step_delay) + rest[2]) / (4 * smoto_control[2].accel_count + 1));
						rest[2] = ((2 * smoto_control[2].step_delay) + rest[2]) % (4 * smoto_control[2].accel_count + 1);
						/* 检查是否为最后一步 */
						if (smoto_control[2].accel_count >= 0 )
						{
							smoto_control[2].run_state = STOP;
						}
						break;
				}
					smoto_control[2].step_delay = new_step_delay[2];     /* 为下个(新的)延时(脉冲周期)*/
			}
		}
            
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
		{

			tim_count = __HAL_TIM_GET_COUNTER(&htim3);
			tmp[3] = tim_count + smoto_control[3].step_delay / 2;    /* 整个C值里边是需要翻转两次的所以需要除以2 */
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, tmp[3]);
			i[3]++;                                                     /* 定时器中断次数计数值 */
			if (i[3]== 2)                                              /* 2次，说明已经输出一个完整脉冲 */
			{
				i[3]= 0;                                               /* 清零定时器中断次数计数值 */
				switch (smoto_control[3].run_state)              /* 加减速曲线阶段 */
				{
					case STOP:
						step_count[3] = 0;                       /* 清零步数计数器 */
						rest[3] = 0;                             /* 清零余值 */
						Smoto_Stop_Single(3);
						Motion_State[3] = 0;                     /* 电机为停止状态  */
						break;

					case ACCEL:
						Add_Pulse_Count[3]++;                    /* 只用于记录相对位置转动了多少度 */
						step_count[3]++;                         /* 步数加1 */
						if (smoto_control[3].dir == CR)
						{
							Step_Position[3]++;                  /* 绝对位置加1  记录绝对位置转动多少度 */
						}
						else 
						{
							Step_Position[3]--;                  /* 绝对位置减1 */
						}
						smoto_control[3].accel_count++;          /* 加速计数值加1 */
						new_step_delay[3] = smoto_control[3].step_delay - (((2 * smoto_control[3].step_delay) + rest[3]) / (4 * smoto_control[3].accel_count + 1));
						rest[3] = ((2 * smoto_control[3].step_delay) + rest[3]) % (4 * smoto_control[3].accel_count + 1);
						/* 检查是否到了需要减速的步数 */
						if (step_count[3] >= smoto_control[3].decel_start)
						{
							if (Stop_or_not[3] == 1)
							{
								last_accel_delay[3] = new_step_delay[3];
								new_step_delay[3] = smoto_control[3].min_delay;
								rest[3] = 0;
								smoto_control[3].run_state = RUN;    /* 下个脉冲进入匀速阶段 */
							}
							else 
							{
								smoto_control[3].accel_count = smoto_control[3].decel_val;
								smoto_control[3].run_state = DECEL;  /* 下个脉冲进入减速阶段 */
							}
						}
						else if (new_step_delay[3] <= smoto_control[3].min_delay)
						{
							last_accel_delay[3] = new_step_delay[3];
							new_step_delay[3] = smoto_control[3].min_delay;
							rest[3] = 0;
							smoto_control[3].run_state = RUN;        /* 设置为匀速运行状态 */
						}
						break;

					case RUN:
						Add_Pulse_Count[3]++;
						step_count[3]++;                             /* 步数加1 */
						if (smoto_control[3].dir == CR)
						{
							Step_Position[3]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[3]--;                      /* 绝对位置减1 */
						}
						new_step_delay[3] = smoto_control[3].min_delay;
						if (step_count[3] >= smoto_control[3].decel_start)
						{
							if (Stop_or_not[3]!=1)
							{
								smoto_control[3].accel_count = smoto_control[3].decel_val;
								new_step_delay[3] = last_accel_delay[3];
								smoto_control[3].run_state = DECEL;    /* 状态改变为匀速 */
							}         
						}
						break;

					case DECEL:
						step_count[3]++;                             /* 步数加1 */
						Add_Pulse_Count[3]++;
						if (smoto_control[3].dir == CR)
						{
							Step_Position[3]++;                      /* 绝对位置加1 */
						}
						else
						{
							Step_Position[3]--;                      /* 绝对位置减1 */
						}
						smoto_control[3].accel_count++;
						new_step_delay[3] = smoto_control[3].step_delay - (((2 * smoto_control[3].step_delay) + rest[3]) / (4 * smoto_control[3].accel_count + 1));
						rest[3] = ((2 * smoto_control[3].step_delay) + rest[3]) % (4 * smoto_control[3].accel_count + 1);
						/* 检查是否为最后一步 */
						if (smoto_control[3].accel_count >= 0 )
						{
							smoto_control[3].run_state = STOP;
						}
						break;
				}
					smoto_control[3].step_delay = new_step_delay[3];     /* 为下个(新的)延时(脉冲周期)*/
			}
		}
    } 
    
}