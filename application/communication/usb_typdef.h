
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 20

/*-------------------- Send --------------------*/

// 串口调试数据包
typedef struct DebugData
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x01
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    struct
    {
        uint8_t name[10];
        uint8_t type;
        float data;
    } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];
    uint16_t checksum;
} __attribute__((packed)) DebugSendData_s;

// IMU 数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x02
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        float x_accel;  // m/s^2
        float y_accel;  // m/s^2
        float z_accel;  // m/s^2
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ImuSendData_s;

/*-------------------- Receive --------------------*/
