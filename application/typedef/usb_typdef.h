#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "struct_typedef.h"
#include "attribute_typedef.h"
#include "remote_control.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DEBUG_DATA_SEND_ID        ((uint8_t)0x01)
#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define ROBOT_INFO_DATA_SEND_ID   ((uint8_t)0x03)
#define PID_DEBUG_DATA_SEND_ID    ((uint8_t)0x04)
#define ALL_ROBOT_HP_SEND_ID      ((uint8_t)0x05)
#define GAME_STATUS_SEND_ID       ((uint8_t)0x06)
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x07)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define PID_DEBUG_DATA_RECEIVE_ID  ((uint8_t)0x02)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
// clang-format on

/*-------------------- Send --------------------*/

// typedef struct InfoData
// {
//     struct
//     {
//         uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
//         uint8_t len;  // 数据段长度
//         uint8_t id;   // 数据段id = 0x00
//         uint8_t crc;  // 数据帧头的 CRC8 校验
//     } __packed__ frame_header;

//     uint16_t type;

//     uint16_t checksum;
// } __packed__ InfoData_s;

// 串口调试数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x01
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        uint8_t name[10];
        uint8_t type;
        float data;
    } __packed__ packages[DEBUG_PACKAGE_NUM];

    uint16_t checksum;
} __packed__ SendDataDebug_s;

// IMU 数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x02
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

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
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataImu_s;

// 机器人信息数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x03
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis : 3;
            uint16_t gimbal : 3;
            uint16_t shoot : 3;
            uint16_t arm : 3;
            uint16_t custom_controller : 3;
            uint16_t reserve : 1;
        } __packed__ type;

        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis : 1;
            uint8_t gimbal : 1;
            uint8_t shoot : 1;
            uint8_t arm : 1;
            uint8_t custom_controller : 1;
            uint8_t reserve : 3;
        } __packed__ state;

        /// @brief 机器人裁判系统信息 7 bytes
        struct
        {
            uint8_t id;
            uint8_t color;  // 0-red 1-blue 2-unknown
            bool attacked;
            uint16_t hp;
            uint16_t heat;
        } __packed__ referee;

    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRobotInfo_s;

// PID调参数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x04
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataPidDebug_s;

// 全场机器人hp信息数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x05
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        uint16_t red_1_robot_hp;
        uint16_t red_2_robot_hp;
        uint16_t red_3_robot_hp;
        uint16_t red_4_robot_hp;
        uint16_t red_5_robot_hp;
        uint16_t red_7_robot_hp;
        uint16_t red_outpost_hp;
        uint16_t red_base_hp;
        uint16_t blue_1_robot_hp;
        uint16_t blue_2_robot_hp;
        uint16_t blue_3_robot_hp;
        uint16_t blue_4_robot_hp;
        uint16_t blue_5_robot_hp;
        uint16_t blue_7_robot_hp;
        uint16_t blue_outpost_hp;
        uint16_t blue_base_hp;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataAllRobotHp_s;

// 比赛信息数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x06
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        uint8_t game_progress;
        uint16_t stage_remain_time;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataGameStatus_s;

// 机器人运动数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x07
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;

    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x01
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;

        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_lenth;
        } __packed__ chassis;

        struct
        {
            float pitch;
            float yaw;
        } __packed__ gimbal;

        struct
        {
            uint8_t fire;
            uint8_t fric_on;
        } __packed__ shoot;

    } __packed__ data;

    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

// PID调参数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x02
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    struct
    {
        float kp;
        float ki;
        float kd;
        float max_out;
        float max_iout;
    } __packed__ data;

    uint16_t crc;
} __packed__ ReceiveDataPidDebug_s;

// 虚拟遥控器数据包
typedef struct
{
    struct
    {
        uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
        uint8_t len;  // 数据段长度
        uint8_t id;   // 数据段id = 0x03
        uint8_t crc;  // 数据帧头的 CRC8 校验
    } __packed__ frame_header;

    uint32_t time_stamp;

    RC_ctrl_t data;

    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;
#endif  // USB_TYPEDEF_H
