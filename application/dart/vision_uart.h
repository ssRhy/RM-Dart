/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       vision_uart.c/h
 * @brief      飞镖板视觉串口接收模块
 * @note       仅在 DART_CHASSIS 主控板（DART_BOARD_MAIN）上编译。
 *             使用 USART6（3-pin，460800 baud）接收上位机视觉数据包。
 *
 *             视觉→电控 数据包（9 字节，packed）：
 *               [0]   header        固定 0xA5
 *               [1]   crc8          CRC8( byte[0] )，初始值 0xFF
 *               [2-5] yaw_error     float，绿灯相对原点横向像素差
 *               [6]   target_status 0=丢失目标  1=识别到目标
 *               [7-8] crc16         CRC16( byte[0..6] )，初始值 0xFFFF
 *
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025            HY              1. done
 *
 @verbatim
 ==============================================================================
 调用顺序（在 dart_main.c 中）：
   Init    : VisionUartInit()         ← DartMainInit() 调用一次
   Loop    : VisionUartTaskLoop()     ← DartMainObserver() 每周期调用
   Getters : GetVisionYawError()
             GetVisionTargetStatus()
 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef VISION_UART_H
#define VISION_UART_H

#include "robot_param.h"
#include "struct_typedef.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#define VISION_FRAME_LEN  9u   /**< VisionData 帧总长度（字节） */

/**
 * @brief  初始化 USART6 DMA 双缓冲接收（460800 baud）
 */
extern void VisionUartInit(void);

/**
 * @brief  从接收 FIFO 中解析视觉数据包，每控制周期调用一次
 */
extern void VisionUartTaskLoop(void);

/**
 * @brief  获取视觉 yaw 误差（像素）
 * @return float  正/负对应视觉当前定义方向
 */
extern float GetVisionYawError(void);

/**
 * @brief  获取视觉目标状态
 * @return 0 = 丢失目标，1 = 识别到目标
 */
extern uint8_t GetVisionTargetStatus(void);

#endif  /* CHASSIS_TYPE == DART_CHASSIS */

#endif  /* VISION_UART_H */
/*------------------------------ End of File ------------------------------*/
