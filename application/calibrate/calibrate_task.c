/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       calibrate_task.c/h
  * @brief      校准任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-27-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#define CALIBRATE_CONTROL_TIME 1

#include "calibrate_task.h"

#include <stdlib.h>
#include <string.h>

#include "bsp_flash.h"
#include "calibrate.h"
#include "cmsis_os.h"
#include "remote_control.h"

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT \
    (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3 + CALI_LIST_LENGHT * 4)

/*------------------------------ Variable Definition ------------------------------*/

static const RC_ctrl_t * calibrate_RC;  //remote control point
static head_cali_t head_cali;           //head cali data
static gimbal_cali_t gimbal_cali;       //gimbal cali data
static imu_cali_t accel_cali;           //accel cali data
static imu_cali_t gyro_cali;            //gyro cali data
static imu_cali_t mag_cali;             //mag cali data

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//cali data address
static uint32_t * cali_sensor_buf[CALI_LIST_LENGHT] = {
    // clang-format off
    (uint32_t *)&head_cali,
    (uint32_t *)&gimbal_cali,
    (uint32_t *)&gyro_cali,
    (uint32_t *)&accel_cali,
    (uint32_t *)&mag_cali
    // clang-format on
};
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
    // clang-format off
    sizeof(head_cali_t) / 4,
    sizeof(gimbal_cali_t) / 4, 
    sizeof(imu_cali_t) / 4, 
    sizeof(imu_cali_t) / 4,
    sizeof(imu_cali_t) / 4
    // clang-format on
};
// void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_gimbal_hook, cali_gyro_hook, NULL, NULL};
void * cali_hook_fun[CALI_LIST_LENGHT] = {NULL, NULL, NULL, NULL};

/*------------------------------ Function Declaration ------------------------------*/

/*******************************************************************************/
/* Flash Operation Function                                                    */
/*******************************************************************************/

static void cali_data_read(void);
static void cali_data_write(void);

/*------------------------------ Task Definition ------------------------------*/

/**
  * @brief          校准任务
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void calibrate_task(void const * pvParameters)
{
    while (1) {
        ;
        vTaskDelay(CALIBRATE_CONTROL_TIME);
    }
}

/*------------------------------ Function Definition ------------------------------*/

/*******************************************************************************/
/* Flash Operation Function                                                    */
/*******************************************************************************/

/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        //read the data in flash,
        cali_flash_read(
            FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);

        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        cali_flash_read(
            FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);

        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL) {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

/**
  * @brief          往flash写入校准数据
  * @param[in]      none
  * @retval         none
  */
static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++) {
        //copy the data of device calibration data
        memcpy(
            (void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf,
            cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
        memcpy(
            (void *)(flash_write_buf + offset), (void *)cali_sensor[i].name,
            CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }

    //erase the page
    cali_flash_erase(FLASH_USER_ADDR, 1);
    //write data
    cali_flash_write(
        FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/
void cali_param_init(void) { ; }

int8_t get_control_temperature(void) { return 30.0f; }

void unused_cali_func(void)
{
    // clang-format off
    uint8_t flag = 
        *cali_sensor_size != 0 && 
        *cali_sensor_buf != 0 && 
        calibrate_RC != NULL &&
        *cali_name != 0;
    // clang-format on
    if (flag) {
        cali_data_read();
        cali_data_write();
    }
}

/*------------------------------ End of File ------------------------------*/
