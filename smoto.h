#ifndef __SMOTO_H
#define __SMOTO_H

#include "sys.h"
#include "main.h" 
#include "gpio.h" 

#define SMOTO_NUM           4                               /* 步进电机数量 */
#define TIM_FREQ            84000000U                       /* 定时器频率 */
#define MAX_STEP_ANGLE      0.00225                           /* 最小步距角(1.8/MICRO_STEP) */
#define PAI                 3.1415926                       /* 圆周率 */
#define FSPR                200                             /* 每转步数 */
#define MICRO_STEP          125                             /* 微步细分 */
#define T1_FREQ             (TIM_FREQ/84)                   /* 频率ft值 */
#define SPR                 (FSPR*MICRO_STEP)               /* 转一圈需要的步数 */

/* 数学常量 */
#define ALPHA               ((float)(2*PAI/SPR))            /* 步距角 = 2*pi/spr */
#define A_T_x10             ((float)(10*ALPHA*T1_FREQ))		/* 加速计算用 */
#define T1_FREQ_148         ((float)((T1_FREQ*0.69)/10))    /* 0.69为误差修正值 */
#define A_SQ                ((float)(2*100000*ALPHA))       /* T1_FREQ_148和A_SQ是计算C的常量，用于速度变化 */
#define A_x200              ((float)(200*ALPHA))            /* 电机旋转一圈的角度 */

/* 速度控制结构体，用于存储每个电机的速度变化参数 */
typedef struct
{
    __IO uint8_t  run_state;                                /* 电机旋转状态 */
    __IO uint8_t  dir;                                      /* 电机旋转方向 */
    __IO int32_t  step_delay;                               /* 当前步延迟，定时器周期，该值越小时速度越快 */
    __IO uint32_t decel_start;                              /* 开始减速位置 */
    __IO int32_t  decel_val;                                /* 减速步数 */
    __IO int32_t  min_delay;                                /* 最小延迟，最大速度，最大值和最小值(启动速度，最大速度) */
    __IO int32_t  accel_count;                              /* 加速计数器 */
} speedRampData;

enum EN
{
    EN_OFF = 0,                                             /* 使能关闭，电机失能，停止转动 */
    EN_ON                                                      /* 使能开启 */
};
enum STA
{
    STOP = 0,                                               /* 加减速曲线状态：1停止*/
    ACCEL,                                                  /* 加减速曲线状态：2加速阶段*/
    DECEL,                                                  /* 加减速曲线状态：3减速阶段*/
    RUN                                                     /* 加减速曲线状态：4匀速阶段*/
};

enum DIR
{
 CCR = 0,                                                   /* 逆时针 */ 
 CR                                                         /* 顺时针 */
};

/* 步进电机引脚定义*/

#define STEPPER_MOTOR_1       1                             /* 步进电机接口序号 */
#define STEPPER_MOTOR_2       2
#define STEPPER_MOTOR_3       3
#define STEPPER_MOTOR_4       4

/*----------------------- 方向引脚控制 -----------------------------------*/
/* 共阳极接法，低电平的时候电机顺时针旋转，高电平逆时针旋转*/  
#define ST1_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin, GPIO_PIN_RESET) : \
                              HAL_GPIO_WritePin(DIR1_GPIO_Port,DIR1_Pin, GPIO_PIN_SET); \
                          }while(0)  

#define ST2_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin, GPIO_PIN_RESET) : \
                              HAL_GPIO_WritePin(DIR2_GPIO_Port,DIR2_Pin, GPIO_PIN_SET); \
                          }while(0)  

#define ST3_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin, GPIO_PIN_RESET) : \
                              HAL_GPIO_WritePin(DIR3_GPIO_Port,DIR3_Pin, GPIO_PIN_SET); \
                          }while(0)  
#define ST4_DIR(x)    do{ x ? \
                              HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin, GPIO_PIN_RESET) : \
                              HAL_GPIO_WritePin(DIR4_GPIO_Port,DIR4_Pin, GPIO_PIN_SET); \
                          }while(0)  
 
/*----------------------- 脱机引脚控制 -----------------------------------*/
/* 由于我们使用的是共阳极接法，并且硬件对电平做了取反操作，所以当 x = 0 有效，x = 1时无效*/                          
#define ST1_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, GPIO_PIN_RESET); \
                        }while(0)    

#define ST2_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, GPIO_PIN_RESET); \
                        }while(0)    

#define ST3_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(EN3_GPIO_Port, EN3_Pin, GPIO_PIN_RESET); \
                        }while(0)                   
#define ST4_EN(x)      do{ x ? \
                          HAL_GPIO_WritePin(EN4_GPIO_Port, EN4_Pin, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(EN4_GPIO_Port, EN4_Pin, GPIO_PIN_RESET); \
                        }while(0)                                          

// 电机初始化函数
void Smoto_Init(void);

// 启动单个电机
void Smoto_Start_Single(uint8_t motor_num);

// 停止单个电机
void Smoto_Stop_Single(uint8_t motor_num);

// 控制单个电机运动
void Smoto_Control_Single(uint8_t motor_num, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed, uint8_t stop_or_not);

#endif