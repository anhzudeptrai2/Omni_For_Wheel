#ifndef DRV_PID_TGC_H
#define DRV_PID_TGC_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f407xx.h"

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

typedef struct
{
    uint8_t dir;
    uint8_t add;
    uint8_t speed;
} DRIVER_PID_TGC;

void PID_TGC_Init(UART_HandleTypeDef *uart_par);
void Assign_PID_TGC_Id(DRIVER_PID_TGC *drv, uint8_t id);
void Send_Speed_TGC(DRIVER_PID_TGC *drv, int16_t speed_par);
void Brake_Motor_TGC(DRIVER_PID_TGC *drv);

#endif // DRV_PID_TGC_H