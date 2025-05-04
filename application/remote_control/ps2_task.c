#include "ps2_task.h"

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

    HAL_SPI_TransmitReceive(&hspi2, &cmd[0], &pRxData[0], 1, 100);  // 发送0x01，请求接受数据
    delay_us(10);
    HAL_SPI_TransmitReceive(
        &hspi2, &cmd[1], &pRxData[1], 1, 100);  // 发送0x42，接受0x01（PS2表示开始通信）
    delay_us(10);
    HAL_SPI_TransmitReceive(
        &hspi2, &cmd[2], &pRxData[2], 1, 100);  // 发送0x00，接受ID（红绿灯模式）
    delay_us(10);

    for (uint8_t i = 3; i < 9; i++) {
        HAL_SPI_TransmitReceive(&hspi2, &cmd[2], &pRxData[i], 1, 100);  // 接受数据
        delay_us(10);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  //CS_H
}

void ps2_task(void const * pvParameters)
{
    while (1) {
        Spi2RequestPs2Data(ps2_data.ps2.raw.data);  // SPI请求数据

        // 系统延时
        vTaskDelay(PS2_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        ps2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
/*------------------------------ End of File ------------------------------*/
