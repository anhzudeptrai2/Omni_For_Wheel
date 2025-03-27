#ifndef DRIVER_PID_AML_H
#define DRIVER_PID_AML_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

// extern UART_HandleTypeDef huart4;

typedef struct
{
    uint8_t ID;
    int16_t Set_Point;
} Motor_Driver;

void Driver_PID_AML_Init(UART_HandleTypeDef *uart_par);
void Driver_Home_Request(Motor_Driver motor);
void Driver_Set_Zero_Position(Motor_Driver motor);
void Driver_Send_Setpoint(Motor_Driver motor);

#endif // DRIVER_PID_AML_H
