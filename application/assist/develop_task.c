// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "cmsis_os.h"
#include "data_exchange.h"
#include "remote_control.h"
#include "signal_generator.h"
#include "stm32f4xx_hal.h"
#include "usb_debug.h"
#include "user_lib.h"
#include "gimbal.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t develop_high_water;
#endif

const Imu_t * p_IMU;
const RC_ctrl_t * p_VIRTUAL_RC_CTRL;
const CaliBuzzerState_e * p_CALI_BUZZER_STATE;
const ChassisSpeedVector_t * p_CHASSIS_FDB_SPEED;
const RobotCmdData_t * p_ROBOT_CMD_DATA;
const bool * p_USB_OFFLINE;

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    p_IMU = Subscribe(IMU_NAME);
    p_VIRTUAL_RC_CTRL = Subscribe(VIRTUAL_RC_NAME);
    p_CALI_BUZZER_STATE = Subscribe(CALI_BUZZER_STATE_NAME);
    p_CHASSIS_FDB_SPEED = Subscribe(CHASSIS_FDB_SPEED_NAME);
    p_ROBOT_CMD_DATA = Subscribe(ROBOT_CMD_DATA_NAME);
    p_USB_OFFLINE = Subscribe(USB_OFFLINE_NAME);

    while (1) {
        
//        float a = GetGimbalDeltaYawMid();
          vTaskDelay(1);

#if INCLUDE_uxTaskGetStackHighWaterMark
        develop_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
