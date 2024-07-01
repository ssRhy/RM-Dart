// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
#include "data_exchange.h"
#include "signal_generator.h"
#include "usb_task.h"
#include "user_lib.h"

const Imu_t * imu;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    imu = Subscribe("imu_data");

    while (1) {
        // code here
        // ModifyDebugDataPackage(0, imu->yaw, "yaw");
        // ModifyDebugDataPackage(1, imu->pitch, "pitch");
        // ModifyDebugDataPackage(2, imu->pitch, "roll");

        vTaskDelay(1);
    }
}
