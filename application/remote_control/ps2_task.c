/**
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
  * @file       ps2_task.c/h
  * @brief      主要负责和ps2手柄进行通信，接收数据
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-05-2025     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 PolarBear****************************
*/
#include "ps2_task.h"

#include <stdbool.h>

#include "bsp_delay.h"
#include "bsp_spi.h"
#include "cmsis_os.h"
#include "ps2.h"
#include "ps2_typedef.h"
#include "stm32f4xx_hal.h"

#define PS2_TASK_TIME_MS 10  // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ps2_high_water;
#endif

Ps2Data_t ps2_data;  // PS2数据结构体

/**
  * @brief          使用Spi2请求ps2的数据
  * @param[in]      pRxData :接收数据的指针，长度为9
  * @retval         
  */
void Spi2RequestPs2Data(uint8_t * pRxData)
{
    uint8_t cmd[3] = {0x01, 0x42, 0x00};  // 请求接受数据

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);  //CS_L
    delay_us(10);

    // 发送0x01，请求接受数据
    HAL_SPI_TransmitReceive(&hspi2, &cmd[0], &pRxData[0], 1, 100);
    delay_us(10);

    // 发送0x42，接受0x01（PS2表示开始通信）
    HAL_SPI_TransmitReceive(&hspi2, &cmd[1], &pRxData[1], 1, 100);
    delay_us(10);

    // 发送0x00，接受ID（红绿灯模式）
    HAL_SPI_TransmitReceive(&hspi2, &cmd[2], &pRxData[2], 1, 100);
    delay_us(10);

    for (uint8_t i = 3; i < 9; i++) {  // 接受数据
        HAL_SPI_TransmitReceive(&hspi2, &cmd[2], &pRxData[i], 1, 100);
        delay_us(10);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  //CS_H
}

void ps2_task(void const * pvParameters)
{
    vTaskDelay(10);
    
    while (1) {
        Spi2RequestPs2Data(ps2_data.ps2.raw.data);  // SPI请求数据

        // 系统延时
        vTaskDelay(PS2_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        ps2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetPs2Axis                                      */
/*                GetPs2Key                                       */
/******************************************************************/

float GetPs2Axis(Ps2Key_e key)
{
    switch (key) {
        case PS2_LX:
            return (ps2_data.ps2.val.lx - 128) / 128.0f;

        case PS2_LY:
            return (ps2_data.ps2.val.ly - 128) / 128.0f;

        case PS2_RX:
            return (ps2_data.ps2.val.rx - 128) / 128.0f;

        case PS2_RY:
            return (ps2_data.ps2.val.ry - 128) / 128.0f;

        default:
            return 0.0f;
    }
}

bool GetPs2Key(Ps2Key_e key)
{
    switch (key) {
        case PS2_L1:
            return ps2_data.ps2.val.l1;

        case PS2_L2:
            return ps2_data.ps2.val.l2;

        case PS2_L3:
            return ps2_data.ps2.val.l3;

        case PS2_R1:
            return ps2_data.ps2.val.r1;

        case PS2_R2:
            return ps2_data.ps2.val.r2;

        case PS2_R3:
            return ps2_data.ps2.val.r3;

        case PS2_SELECT:
            return ps2_data.ps2.val.select;

        case PS2_START:
            return ps2_data.ps2.val.start;

        case PS2_UP:
            return ps2_data.ps2.val.up;

        case PS2_DOWN:
            return ps2_data.ps2.val.down;

        case PS2_LEFT:
            return ps2_data.ps2.val.left;

        case PS2_RIGHT:
            return ps2_data.ps2.val.right;

        case PS2_TRIANGLE:
            return ps2_data.ps2.val.triangle;

        case PS2_CIRCLE:
            return ps2_data.ps2.val.circle;

        case PS2_CROSS:
            return ps2_data.ps2.val.cross;

        case PS2_SQUARE:
            return ps2_data.ps2.val.square;

        default:
            return false;
    }
}

/*------------------------------ End of File ------------------------------*/
