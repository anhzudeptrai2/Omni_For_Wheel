#ifndef WT901C_H
#define WT901C_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f407xx.h"

inline uint8_t Low_Byte(uint16_t w) { return (uint8_t)(w & 0xFF); }
inline uint8_t High_Byte(uint16_t w) { return (uint8_t)(w >> 8); }

typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
    UART_HandleTypeDef *WT901C_UART;
} WT901C;

/*MODBUS RTU protocol command frame defines*/

#define WT901C_ADDR 0x01 // WT901C Device Address - Update this if the WT901C-485 RTU address changes

#define TX_BUFFER_SIZE 8
#define MODBUS_RESPONSE_SIZE 11
#define ANGLE_RESET_BUFFER_SIZE 24
#define READ_FUNC 0x03
#define WRITE_FUNC 0x06
#define ROLL_ADDR 0x3D
#define PITCH_ADDR 0x3E
#define YAW_ADDR 0x3F
#define WT901C_REQ_MAX_TIME 100
#define ANGLE_RESET_COMMAND {                                    \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x69, 0xB5, 0x88, 0x2F, 0x20, \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x01, 0x00, 0x04, 0xD9, 0xC9, \
    WT901C_ADDR, WRITE_FUNC, 0x00, 0x00, 0x00, 0x00, 0x89, 0xCA}

/*Init with UART & Rx Interrupt*/
void WT901C_Init(WT901C *imu, UART_HandleTypeDef *wt901_uart);
/*Requests angle update in timer interrupt*/
void WT901C_Angle_Request(WT901C *imu);
/*Resets WT901C angles*/
void WT901C_Reset_Angles(WT901C *imu); 
/* Example
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    WT901C_UART_Rx_IDLE_Hanlde(&IMU);
  }
}
*/
/*Place in main.c uart idle callback func*/
void WT901C_UART_Rx_IDLE_Hanlde(WT901C *imu);

#endif // WT901C_H
