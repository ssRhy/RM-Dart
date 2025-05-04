#include "ps2_task.h"

#include "cmsis_os.h"

#define PS2_TASK_TIME_MS 10  // ms

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t ps2_high_water;
#endif

void ps2_task(void const * pvParameters)
{
    while (1) {
        // 系统延时
        vTaskDelay(PS2_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        ps2_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
/*------------------------------ End of File ------------------------------*/
