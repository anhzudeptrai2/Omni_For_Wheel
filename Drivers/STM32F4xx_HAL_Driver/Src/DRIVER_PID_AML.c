#include "DRIVER_PID_AML.h"

#define Low_Byte(w) ((uint8_t)((w) & 0xff))
#define High_Byte(w) ((uint8_t)((w) >> 8))

UART_HandleTypeDef *driver_pid_uart;

uint8_t buffer[4];

void Driver_PID_AML_Init(UART_HandleTypeDef *uart_par)
{
    driver_pid_uart = uart_par;
}

void Driver_Home_Request(Motor_Driver motor)
{
    buffer[0] = motor.ID;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xf3;

    HAL_UART_Transmit(driver_pid_uart, (uint8_t *)buffer, 4, 200);
}

void Driver_Set_Zero_Position(Motor_Driver motor)
{
    buffer[0] = motor.ID;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xf2;

    HAL_UART_Transmit(driver_pid_uart, (uint8_t *)buffer, 4, 200);
}

void Driver_Send_Setpoint(Motor_Driver motor)
{
    buffer[0] = motor.ID;
    buffer[1] = Low_Byte(motor.Set_Point);
    buffer[2] = High_Byte(motor.Set_Point);
    buffer[3] = 0xff;

    HAL_UART_Transmit(driver_pid_uart, (uint8_t *)buffer, 4, 200);
}