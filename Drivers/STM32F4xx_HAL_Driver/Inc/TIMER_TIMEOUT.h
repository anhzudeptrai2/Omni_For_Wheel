#ifndef TIMER_TIMEOUT_H
#define TIMER_TIMEOUT_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f407xx.h"
#include "stm32f4xx_hal_tim.h"

#define TASK_NUMS 3 // Extend number of task for larger number task

typedef struct
{
    uint8_t Time_Out_Flag; // Be set if timeout occur
    uint16_t Time_Out_Set;
    uint16_t Task_Time; // ms unit
} Task_Timeout;

extern Task_Timeout Task_TO[TASK_NUMS]; // declare this struct in main.c

void Timer_Timeout_Init(Task_Timeout *t_to, uint16_t timeout_ms);
void Timer_Timeout_Start(TIM_HandleTypeDef *htim);
/*Put this function after a completed task to reset timeout timer*/
void Timer_Timeout_Reset(Task_Timeout *t_to);
/*Put this function in 1ms timer interrupt handle to update all available timeout*/
void Timer_Timeout_Check(Task_Timeout *t_to);


#endif // TIMER_TIMEOUT_H