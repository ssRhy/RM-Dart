
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 10

/*-------------------- Send --------------------*/

typedef struct InfoData
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x00
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    uint16_t type;

    uint16_t checksum;
} __attribute__((packed)) InfoData_s;

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

    uint32_t time_stamp;

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

    uint32_t time_stamp;

    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ImuSendData_s;

// 机器人信息数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x03
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __attribute__((packed)) frame_header;

    uint32_t time_stamp;

    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis:3;
            uint16_t gimbal:3;
            uint16_t shoot:3;
            uint16_t arm:3;
            uint16_t custom_controller:3;
            uint16_t reserve:1;
        } __attribute__((packed)) moudle;

        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis:1;
            uint8_t gimbal:1;
            uint8_t shoot:1;
            uint8_t arm:1;
            uint8_t custom_controller:1;
            uint8_t reserve:3;
        } __attribute__((packed)) state;

        /// @brief 机器人运动状态 12 bytes
        struct
        {
            float vx;  // m/s
            float vy;  // m/s
            float wz;  // rad/s
        } __attribute__((packed)) speed_vector;

    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) RobotInfoSendData_s;
/*-------------------- Receive --------------------*/
