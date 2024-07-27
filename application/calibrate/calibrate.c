// #include "calibrate.h"

// #include <stdlib.h>

// #include "bsp_flash.h"
// #include "remote_control.h"

// static const RC_ctrl_t * calibrate_RC;  //remote control point
// static gimbal_cali_t gimbal_cali;       //gimbal cali data
// static imu_cali_t accel_cali;           //accel cali data
// static imu_cali_t gyro_cali;            //gyro cali data
// static imu_cali_t mag_cali;             //mag cali data

// cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

// //cali data address
// static uint32_t * cali_sensor_buf[CALI_LIST_LENGHT] = {
//     // clang-format off
//     (uint32_t *)&gimbal_cali,
//     (uint32_t *)&gyro_cali,
//     (uint32_t *)&accel_cali,
//     (uint32_t *)&mag_cali
//     // clang-format on
// };
// static uint8_t cali_sensor_size[CALI_LIST_LENGHT] = {
//     // clang-format off
//     sizeof(gimbal_cali_t) / 4, 
//     sizeof(imu_cali_t) / 4, 
//     sizeof(imu_cali_t) / 4,
//     sizeof(imu_cali_t) / 4
//     // clang-format on
// };
// void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_gimbal_hook, cali_gyro_hook, NULL, NULL};


/**
  * @brief          从flash读取校准数据
  * @param[in]      none
  * @retval         none
  */
// static void cali_data_read(void)
// {
//     uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
//     uint8_t i = 0;
//     uint16_t offset = 0;
//     for (i = 0; i < CALI_LIST_LENGHT; i++) {
//         //read the data in flash,
//         cali_flash_read(
//             FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);

//         offset += cali_sensor[i].flash_len * 4;

//         //read the name and cali flag,
//         cali_flash_read(
//             FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);

//         cali_sensor[i].name[0] = flash_read_buf[0];
//         cali_sensor[i].name[1] = flash_read_buf[1];
//         cali_sensor[i].name[2] = flash_read_buf[2];
//         cali_sensor[i].cali_done = flash_read_buf[3];

//         offset += CALI_SENSOR_HEAD_LEGHT * 4;

//         if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL) {
//             cali_sensor[i].cali_cmd = 1;
//         }
//     }
// }
