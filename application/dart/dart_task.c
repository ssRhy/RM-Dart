/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_task.c/h
  * @brief      完成飞镖控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-10-25      CJH            1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

  #include "dart_task.h"

#include "attribute_typedef.h"
#include "cmsis_os.h"
#include "dart_main.h"
#include "dart_chasis.h"
#include "dart_shoot.h"
  
  #ifndef DART_TASK_INIT_TIME
  #define DART_TASK_INIT_TIME 201
  #endif  // DART_TASK_INIT_TIME
  
  #ifndef DART_CONTROL_TIME
  #define DART_CONTROL_TIME 1
  #endif  // DART_CONTROL_TIME
  
  #if INCLUDE_uxTaskGetStackHighWaterMark
  uint32_t dart_high_water;
  #endif
  
  __weak void DartPublish(void);
  __weak void DartInit(void);
  __weak void DartHandleException(void);
  __weak void DartSetMode(void);
  __weak void DartObserver(void);
  __weak void DartReference(void);
  __weak void DartConsole(void);
  __weak void DartSendCmd(void);
  
  /**
   * @brief          飞镖任务
   * @param[in]      pvParameters: 空
   * @retval         none
   */
  void dart_task(void const * pvParameters)
  {
      DartPublish();
      // 等待陀螺仪任务更新陀螺仪数据
      vTaskDelay(DART_TASK_INIT_TIME);
      // 飞镖初始化
      DartInit();
  
      while (1) {
          // 更新状态量
          DartObserver();
          // 处理异常
          DartHandleException();
          // 设置飞镖模式
          DartSetMode();
          // 设置目标量
          DartReference();
          // 计算控制量
          DartConsole();
          // 发送控制量
          DartSendCmd();
  
          // 系统延时
          vTaskDelay(DART_CONTROL_TIME);
  
  #if INCLUDE_uxTaskGetStackHighWaterMark
          dart_high_water = uxTaskGetStackHighWaterMark(NULL);
  #endif
      }
  }
  
  __weak void DartPublish(void)
  {
      // 空函数，由具体实现文件重写
  }
__weak void DartInit(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainInit();
    ChassisInit();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootInit();
#endif
}
__weak void DartHandleException(void)
{
    // 空函数，由具体实现文件重写
}
__weak void DartSetMode(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainSetMode();
    ChassisSetMode();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootSetMode();
#endif
}
__weak void DartObserver(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainObserver();
    ChassisObserver();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootObserver();
#endif
}
__weak void DartReference(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainReference();
    ChassisReference();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootReference();
#endif
}
__weak void DartConsole(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainConsole();
    ChassisConsole();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootConsole();
#endif
}
__weak void DartSendCmd(void)
{
#if (DART_BOARD_TYPE == DART_BOARD_MAIN)
    DartMainSendCmd();
    ChassisSendCmd();
#endif
#if (DART_BOARD_TYPE == DART_BOARD_SHOOT)
    DartShootSendCmd();
#endif
}

