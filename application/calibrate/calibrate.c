#include "calibrate.h"

#include <stdlib.h>

#include "IMU_task.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"
#include "remote_control.h"

/**
  * @brief          "head"设备校准
  * @param[in][out] cali:指针指向head数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_head_hook(uint32_t * cali, bool_t cmd)
{
    head_cali_t * local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        //        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP)) {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }

    local_cali_t->firmware_version_x = FIRMWARE_VERSION_X;
    local_cali_t->firmware_version_y = FIRMWARE_VERSION_Y;
    local_cali_t->firmware_version_z = FIRMWARE_VERSION_Z;
    local_cali_t->firmware_version_release = FIRMWARE_VERSION_RELEASE;
    local_cali_t->firmware_version_beta = FIRMWARE_VERSION_BETA;

    return 1;
}

/**
  * @brief          云台设备校准
  * @param[in][out] cali:指针指向云台数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_gimbal_hook(uint32_t * cali, bool_t cmd)
{
    //gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        // set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
        //                      local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
        //                      local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        // if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
        //                          &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
        //                          &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        // {
        //     cali_buzzer_off();

        //     return 1;
        // }
        // else
        // {
        //     gimbal_start_buzzer();

        //     return 0;
        // }
    }

    return 0;
}

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
bool_t cali_gyro_hook(uint32_t * cali, bool_t cmd)
{
    imu_cali_t * local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT) {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);

        return 0;
    } else if (cmd == CALI_FUNC_CMD_ON) {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME) {
            count_time = 0;
            cali_buzzer_off();
            gyro_cali_enable_control();
            return 1;
        } else {
            gyro_cali_disable_control();  //disable the remote control to make robot no move
            imu_start_buzzer();

            return 0;
        }
    }

    return 0;
}