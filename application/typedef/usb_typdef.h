#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DEBUG_DATA_SEND_ID        ((uint8_t)0x01)
#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define EVENT_DATA_SEND_ID        ((uint8_t)0x03)
#define PID_DEBUG_DATA_SEND_ID    ((uint8_t)0x04)
#define ALL_ROBOT_HP_SEND_ID      ((uint8_t)0x05)
#define GAME_STATUS_SEND_ID       ((uint8_t)0x06)
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x07)
#define GROUND_ROBOT_POSITION_SEND_ID ((uint8_t)0x08)
#define RFID_STATUS_SEND_ID       ((uint8_t)0x09)
#define ROBOT_STATUS_SEND_ID      ((uint8_t)0x0A)
#define JOINT_STATE_SEND_ID       ((uint8_t)0x0B)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define PID_DEBUG_DATA_RECEIVE_ID  ((uint8_t)0x02)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
// clang-format on

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// 串口调试数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
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
    FrameHeader_t frame_header;  // 数据段id = 0x02
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

// 事件数据包

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;

    uint8_t supply_station_front;
    uint8_t supply_station_internal;
    uint8_t supply_zone;
    uint8_t center_gain_zone;

    uint8_t small_energy;
    uint8_t big_energy;

    uint8_t circular_highland;
    uint8_t trapezoidal_highland_3;
    uint8_t trapezoidal_highland_4;

    uint8_t base_virtual_shield_remaining;
    
    uint16_t crc;
} __packed__ SendDataEvent_s;


// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
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
    FrameHeader_t frame_header;  // 数据段id = 0x05
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
    FrameHeader_t frame_header;  // 数据段id = 0x06
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
    FrameHeader_t frame_header;  // 数据段id = 0x07
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

// 地面机器人位置数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    
    float hero_x;
    float hero_y;

    float engineer_x;
    float engineer_y;

    float standard_3_x;
    float standard_3_y;

    float standard_4_x;
    float standard_4_y;

    float standard_5_x;
    float standard_5_y;

    uint16_t crc;
} __packed__ SendDataGroundRobotPosition_s;

// RFID状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;

    bool base_gain_point;                       
    bool circular_highland_gain_point;          
    bool enemy_circular_highland_gain_point;
    bool friendly_r3_b3_gain_point;            
    bool enemy_r3_b3_gain_point;                
    bool friendly_r4_b4_gain_point;              
    bool enemy_r4_b4_gain_point;                 
    bool energy_mechanism_gain_point;           
    bool friendly_fly_ramp_front_gain_point;    
    bool friendly_fly_ramp_back_gain_point;     
    bool enemy_fly_ramp_front_gain_point;       
    bool enemy_fly_ramp_back_gain_point;         
    bool friendly_outpost_gain_point;            
    bool friendly_healing_point;                 
    bool friendly_sentry_patrol_area;            
    bool enemy_sentry_patrol_area;               
    bool friendly_big_resource_island;           
    bool enemy_big_resource_island;              
    bool friendly_exchange_area;                 
    bool center_gain_point;        

    uint16_t crc;            
} __packed__ SendDataRfidStatus_s;

// 机器人状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0A
    uint32_t time_stamp;

    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_up;
    uint16_t maximum_hp;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;

    uint16_t shooter_17mm_1_barrel_heat;

    float robot_pos_x;
    float robot_pos_y;
    float robot_pos_angle;

    uint8_t armor_id;
    uint8_t hp_deduction_reason;

    uint16_t projectile_allowance_17mm_1;
    uint16_t remaining_gold_coin;      

    uint16_t crc;
} __packed__ SendDataRobotStatus_s;

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;
    struct
    {
        float pitch;
        float yaw;

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataJointState_s;
/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
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
    FrameHeader_t frame_header;  // 数据段id = 0x02
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
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    RC_ctrl_t data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;
#endif  // USB_TYPEDEF_H
