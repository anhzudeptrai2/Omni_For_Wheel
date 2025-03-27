#include "WT901C.h"
#include "CRC_16.h"
#include "TIMER_TIMEOUT.h"

static uint8_t rx_modbus[MODBUS_RESPONSE_SIZE];

static int16_t Yaw_Continuous_Angle = 0;
static int16_t Yaw_Roll_Count = 0;
static int16_t Prev_Yaw_Angle = 0;

void Infinite_Yaw(int16_t current_angle)
{
    int16_t delta = current_angle - Prev_Yaw_Angle;

    if (delta > 180)
    {
        Yaw_Roll_Count--;
    }
    else if (delta < -180)
    {
        Yaw_Roll_Count++;
    }

    Yaw_Continuous_Angle = current_angle + (Yaw_Roll_Count * 360);
    Prev_Yaw_Angle = current_angle;
}

void WT901C_Init(WT901C *imu, UART_HandleTypeDef *wt901_uart)
{
    imu->WT901C_UART = wt901_uart;
    imu->Yaw = 0;
    imu->Roll = 0;
    imu->Pitch = 0;
}

void WT901C_Angle_Request(WT901C *imu)
{
    uint8_t tx_buffer[] = {0x01, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x94, 0x07};
    HAL_UART_Transmit(imu->WT901C_UART, tx_buffer, 8, WT901C_REQ_MAX_TIME);
    HAL_UARTEx_ReceiveToIdle_IT(imu->WT901C_UART, rx_modbus, MODBUS_RESPONSE_SIZE);
}

void WT901C_Reset_Angles(WT901C *imu)
{
    static const uint8_t tx_angle_rst_buffer[ANGLE_RESET_BUFFER_SIZE] = ANGLE_RESET_COMMAND;
    HAL_UART_Transmit_IT(imu->WT901C_UART, (uint8_t *)tx_angle_rst_buffer, ANGLE_RESET_BUFFER_SIZE);

    Yaw_Continuous_Angle = 0;
    Yaw_Roll_Count = 0;
    Prev_Yaw_Angle = 0;

    imu->Yaw = 0;
    imu->Roll = 0;
    imu->Pitch = 0;
}

void WT901C_UART_Rx_IDLE_Hanlde(WT901C *imu)
{
    uint16_t received_crc = (rx_modbus[MODBUS_RESPONSE_SIZE - 1] << 8) | rx_modbus[MODBUS_RESPONSE_SIZE - 2];
    if (CRC_16(rx_modbus, MODBUS_RESPONSE_SIZE - 2) == received_crc)
    {
        imu->Roll = ((int16_t)((rx_modbus[3] << 8) | rx_modbus[4])) * (180.0f / 32768.0f);
        imu->Pitch = ((int16_t)((rx_modbus[5] << 8) | rx_modbus[6])) * (180.0f / 32768.0f);
        Infinite_Yaw((int16_t)((rx_modbus[7] << 8) | rx_modbus[8]) * (180.0f / 32768.0f));
        imu->Yaw = Yaw_Continuous_Angle;
    }
}
