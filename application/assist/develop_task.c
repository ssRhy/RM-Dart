// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
#include "data_exchange.h"
#include "signal_generator.h"
#include "usb_debug.h"
#include "user_lib.h"
#include "stm32f4xx_hal.h"

const Imu_t * imu;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    imu = Subscribe("imu_data");

    while (1) {
        // code here
        ModifyDebugDataPackage(1, imu->yaw, "yaw");
        ModifyDebugDataPackage(2, (HAL_GetTick()/10) % 1000, "data1");

        vTaskDelay(1);
    }
}
