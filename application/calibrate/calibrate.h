#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "struct_typedef.h"

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_9  // 保存的flash页地址,page 9
#define GYRO_CONST_MAX_TEMP 45.0f            // 最大陀螺仪控制温度
#define CALIED_FLAG 0x55                     // means it has been calibrated
#define GYRO_CALIBRATE_TIME 20000            //gyro calibrate time,陀螺仪校准时间

#define CALI_SENSOR_HEAD_LEGHT 1

// Macro functions

#define imu_start_buzzer() buzzer_on(95, 10000)  // 当imu在校准,蜂鸣器的设置频率和强度
#define gimbal_start_buzzer() buzzer_on(31, 19999)  // 当云台在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_off() buzzer_off()              // buzzer off，关闭蜂鸣器

#define cali_get_mcu_temperature() get_temprate()  // 获取stm32片内温度，计算imu的控制温度

// clang-format off
#define cali_flash_read(address, buf, len) \
    flash_read((address), (buf), (len))  // flash 读取函数
#define cali_flash_write(address, buf, len) \
    flash_write_single_address((address), (buf), (len))  // flash 写入函数
#define cali_flash_erase(address, page_num) \
    flash_erase_address((address), (page_num))  // flash擦除函数
// clang-format on

#define gyro_cali_disable_control() RC_unable()  // 当imu在校准时候,失能遥控器
#define gyro_cali_enable_control() RC_restart(SBUS_RX_BUF_NUM)  // 当imu校准完成,使能遥控器

#define gyro_cali_fun(cali_scale, cali_offset, time_count) \
    INS_cali_gyro((cali_scale), (cali_offset), (time_count))  // 计算陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset) \
    INS_set_cali_gyro((cali_scale), (cali_offset))  // 设置在INS task内的陀螺仪零漂

//cali device name
typedef enum {
    CALI_HEAD = 0,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;

typedef struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t * flash_buf;                               //link to device calibration data
    bool_t (*cali_hook)(uint32_t * point, bool_t cmd);  //cali function
} __attribute__((packed)) cali_sensor_t;

//header device (固件信息)
typedef struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;  // imu control temperature
} __attribute__((packed)) head_cali_t;

//gimbal device (云台设备)
typedef struct
{
    fp32 yaw_offset;
    fp32 pitch_offset;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} __attribute__((packed)) gimbal_cali_t;

//gyro, accel, mag device (陀螺仪,加速度计,磁力计设备)
typedef struct
{
    fp32 offset[3];  //x,y,z
    fp32 scale[3];   //x,y,z
} imu_cali_t;

#endif  // CALIBRATE_TASK_H
