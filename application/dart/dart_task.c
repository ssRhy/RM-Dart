#include "dart_task.h"
#include "attribute_typedef.h"
#include "cmsis_os.h"
#include "dart_chassis.h"
#include "dart_param.h"
#include "dart_shoot.h"
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
  
// #ifndef DART_TASK_INIT_TIME
// #define DART_TASK_INIT_TIME 201
// #endif  // OUTPOST_TASK_INIT_TIME
  
// #ifndef DART_CONTROL_TIME
// #define DART_CONTROL_TIME 1
// #endif  // OUTPOST_CONTROL_TIME
  

void Dart_task(void const * pvParameters)
  {
      DartPublish();
      // 等待陀螺仪任务更新陀螺仪数据
      vTaskDelay(DART_TASK_INIT_TIME);
      // 初始化
     DartInit();
  
    while (1) {
      //底盘部分
          // 更新状态量
          DartObserver();
          // 处理异常
          DartHandleException();
          // 设置模式
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
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartInit(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartHandleException(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartSetMode(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartObserver(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartReference(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
	
__weak void DartSendCmd(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
  
__weak void DartConsole(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
_weak void DartShootObserver(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
_weak void DartShootConsole(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
_weak void DartShootSendCmd(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
_weak void DartShootReference(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartShootConsole(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
__weak void DartShootSendCmd(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
_weak void DartShootHandleException(void)
  {
      /* 
       NOTE : 在其他文件中定义具体内容
      */
  }
