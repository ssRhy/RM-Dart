/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       vision_uart.c/h
 * @brief      飞镖板视觉串口接收模块
 * @note       仅在 DART_CHASSIS 下编译。
 *             USART6_IRQHandler 在本文件中定义（referee_usart_task.c 中的同名
 *             函数需用 #if (CHASSIS_TYPE != DART_CHASSIS) 宏保护，避免重定义）。
 *
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025            HY              1. done
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "vision_uart.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include <string.h>

#include "bsp_usart.h"
#include "CRC8_CRC16.h"
#include "fifo.h"
#include "usart.h"

/* ======================== Private Constants ======================== */

#define VISION_USART_RX_BUF_LEN  64u
#define VISION_FIFO_BUF_LEN      256u

#define VISION_HEADER             0xA5u
#define VISION_CRC8_INIT          0xFFu

/* ======================== Private Types ======================== */

/**
 * @brief 视觉数据包（9 字节，packed）
 *        与上位机协议严格对应：header / crc8 / yaw_error(float) / target_status / crc16
 */
typedef struct {
    uint8_t  header;         /**< 固定 0xA5 */
    uint8_t  crc8;           /**< CRC8( byte[0] ) */
    float    yaw_error;      /**< 横向像素差 */
    uint8_t  target_status;  /**< 0=丢失 1=识别到 */
    uint16_t crc16;          /**< CRC16( byte[0..6] ) */
} __attribute__((packed)) VisionData_t;

/* ======================== Private Variables ======================== */

static uint8_t    s_rx_buf[2][VISION_USART_RX_BUF_LEN];
static fifo_s_t   s_fifo;
static uint8_t    s_fifo_buf[VISION_FIFO_BUF_LEN];

static float    s_yaw_error     = 0.0f;
static uint8_t  s_target_status = 0u;

/* ======================== Private Function ======================== */

static void VisionDataParse(uint8_t *frame);

/* ======================== Public Functions ======================== */

/**
 * @brief  初始化视觉串口（USART6，460800 baud，DMA 双缓冲）
 */
void VisionUartInit(void)
{
    /* 重配 USART6 波特率为 460800（覆盖 CubeMX 默认的 115200） */
    huart6.Init.BaudRate = 460800;
    HAL_UART_Init(&huart6);

    fifo_s_init(&s_fifo, (char *)s_fifo_buf, VISION_FIFO_BUF_LEN);
    usart6_init(s_rx_buf[0], s_rx_buf[1], VISION_USART_RX_BUF_LEN);
}

/**
 * @brief  USART6 IDLE 中断处理（DMA 双缓冲切换 + 推入 FIFO）
 * @note   仅在 DART_CHASSIS 下编译，referee_usart_task.c 的同名函数须被
 *         #if (CHASSIS_TYPE != DART_CHASSIS) 宏保护。
 */
void USART6_IRQHandler(void)
{
    if (USART6->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart6);

        uint16_t rx_len = 0;

        if ((huart6.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            rx_len = VISION_USART_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, VISION_USART_RX_BUF_LEN);
            huart6.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&s_fifo, (char *)s_rx_buf[0], rx_len);
        }
        else
        {
            __HAL_DMA_DISABLE(huart6.hdmarx);
            rx_len = VISION_USART_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
            __HAL_DMA_SET_COUNTER(huart6.hdmarx, VISION_USART_RX_BUF_LEN);
            huart6.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart6.hdmarx);
            fifo_s_puts(&s_fifo, (char *)s_rx_buf[1], rx_len);
        }
    }
}

/**
 * @brief  从接收 FIFO 中扫描并解析视觉数据包
 * @note   每控制周期在 DartMainObserver() 中调用
 */
void VisionUartTaskLoop(void)
{
    uint8_t buf[VISION_FRAME_LEN];

    /* 至少有一整帧才尝试解析 */
    while (fifo_s_used(&s_fifo) >= (int)VISION_FRAME_LEN)
    {
        /* 扫描帧头 */
        uint8_t byte = (uint8_t)fifo_s_get(&s_fifo);
        if (byte != VISION_HEADER)
            continue;

        /* 找到帧头后确认剩余字节够一帧 */
        if (fifo_s_used(&s_fifo) < (int)(VISION_FRAME_LEN - 1u))
            break;  /* 数据尚不完整，等下一次调用 */

        buf[0] = VISION_HEADER;
        for (uint8_t i = 1u; i < VISION_FRAME_LEN; i++)
            buf[i] = (uint8_t)fifo_s_get(&s_fifo);

        VisionDataParse(buf);
    }
}

/* ======================== Getters ======================== */

float GetVisionYawError(void)
{
    return s_yaw_error;
}

uint8_t GetVisionTargetStatus(void)
{
    return s_target_status;
}

/* ======================== Private Implementation ======================== */

/**
 * @brief  校验并解析一帧视觉数据
 * @param  frame  指向 VISION_FRAME_LEN 字节的帧缓冲
 */
static void VisionDataParse(uint8_t *frame)
{
    /* CRC8：覆盖 frame[0]（1 字节），结果应与 frame[1] 一致 */
    uint8_t crc8_calc = get_CRC8_check_sum(frame, 1u, VISION_CRC8_INIT);
    if (crc8_calc != frame[1])
        return;

    /* CRC16：覆盖 frame[0..6]（7 字节），结果与 frame[7..8] 比较 */
    if (!verify_CRC16_check_sum(frame, VISION_FRAME_LEN))
        return;

    const VisionData_t *vd = (const VisionData_t *)frame;
    s_yaw_error     = vd->yaw_error;
    s_target_status = vd->target_status;
}

#endif  /* CHASSIS_TYPE == DART_CHASSIS */
/*------------------------------ End of File ------------------------------*/
