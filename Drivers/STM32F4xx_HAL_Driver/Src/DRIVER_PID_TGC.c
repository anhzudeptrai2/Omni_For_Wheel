#include "DRIVER_PID_TGC.h"

#define BRAKE_VAL 2

UART_HandleTypeDef *Driver_TGC_UART;

void PID_TGC_Init(UART_HandleTypeDef *uart_par)
{
    Driver_TGC_UART = uart_par;
}

void Assign_PID_TGC_Id(DRIVER_PID_TGC *drv, uint8_t id)
{
    drv->add = id;
}

void Send_Speed_TGC(DRIVER_PID_TGC *drv, int16_t speed_par)
{
    if (speed_par < -255)
    {
        speed_par = -255;
    }
    else if (speed_par > 255)
    {
        speed_par = 255;
    }
    drv->dir = (speed_par >= 0) ? 1 : 0;
    speed_par = abs(speed_par);
    if (speed_par == 2)
    {
        speed_par = 1;
    }
    drv->speed = speed_par;
    uint8_t bf_tx[3];
    bf_tx[0] = (drv->dir << 7) | (drv->add & 0x7F);
    bf_tx[1] = drv->speed;
    bf_tx[2] = 0xFF;
    HAL_UART_Transmit(Driver_TGC_UART, (uint8_t *)bf_tx, 3, HAL_MAX_DELAY);
}

void Brake_Motor_TGC(DRIVER_PID_TGC *drv)
{
    uint8_t bf_tx_br[3]=0;
    bf_tx_br[0] = (drv->dir << 7) | (drv->add & 0x7F);
    bf_tx_br[1] = 0x02;
    bf_tx_br[2] = 0xFF;
    HAL_UART_Transmit(Driver_TGC_UART, (uint8_t *)bf_tx_br, 3, HAL_MAX_DELAY);
}