// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "signal_generator.h"
#include "PWM_cmd_pump.h"

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);


    while (1) {
    //    HAL_TIM_Base_Start(&htim1);
    //    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
       int n=GenerateStepWave(30000,0,3);
       PwmCmdPump(3, n);
       

    //    __HAL_TIM_DISABLE(&htim1);//时钟禁用
    //    __HAL_TIM_DISABLE(&htim8);


        vTaskDelay(1);


}
}
