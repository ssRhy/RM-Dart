/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#ifndef CHASSIS_BALANCE_H
#define CHASSIS_BALANCE_H

#include "robot_param.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "IMU_task.h"
#include "chassis.h"
#include "math.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

// clang-format off
#define JOINT_ERROR_OFFSET   ((uint8_t)1 << 0)  // 关节电机错误偏移量
#define WHEEL_ERROR_OFFSET   ((uint8_t)1 << 1)  // 驱动轮电机错误偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
// clang-format on

/*-------------------- Structural definition --------------------*/

typedef enum {
    CHASSIS_OFF,        // 底盘关闭
    CHASSIS_SAFE,       // 底盘无力，所有控制量置0
    CHASSIS_STAND_UP,   // 底盘起立，从倒地状态到站立状态的中间过程
    CHASSIS_CALIBRATE,  // 底盘校准
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 底盘跟随云台（运动方向为云台坐标系方向，需进行坐标转换）
    CHASSIS_FLOATING,    // 底盘悬空状态
    CHASSIS_CUSHIONING,  // 底盘缓冲状态
    CHASSIS_FREE,        // 底盘不跟随云台
    CHASSIS_AUTO,        // 底盘自动模式
    CHASSIS_DEBUG,       // 调试模式
    CHASSIS_CUSTOM       // 自定义模式
} ChassisMode_e;

typedef struct Leg
{
    struct rod
    {
        float Phi0;    // rad
        float dPhi0;   // rad/s
        float ddPhi0;  // rad/s^2

        float L0;    // m
        float dL0;   // m/s
        float ddL0;  // m/s^2
    } rod;

    struct joint
    {
        float Phi1, Phi4;    // rad
        float dPhi1, dPhi4;  // rad/s
    } joint;

    struct wheel
    {
        float Angle;     // rad
        float Velocity;  // rad/s
    } wheel;

    float J[2][2];  //雅可比矩阵
} Leg_t;

typedef struct Body
{
    float x;
    float x_dot;
    float phi;
    float phi_dot;

    float roll;
    float roll_dot;
    float yaw;
    float yaw_dot;
} Body_t;

//状态向量
typedef struct LegState
{
    float theta;
    float theta_dot;
    float x;
    float x_dot;
    float phi;
    float phi_dot;
} LegState_t;

/**
 * @brief      比例系数结构体
 * @note       比例系数，用于手动优化控制效果
 */
typedef struct
{
    float k[2][6];
    float Tp;
    float T;
    float length;
} Ratio_t;

/**
 * @brief 状态
 */
typedef struct
{
    Body_t body;
    Leg_t leg[2];             // 0-左 1-右
    LegState_t leg_state[2];  // 0-左 1-右
    ChassisSpeedVector_t speed_vector;
} Fdb_t;

/**
 * @brief 期望
 */
typedef struct
{
    Body_t body;
    LegState_t leg_state[2];  // 0-左 1-右
    float rod_L0[2];         // 0-左 1-右
    ChassisSpeedVector_t speed_vector;
} Ref_t;

typedef struct Cmd
{
    struct leg
    {
        struct rod_cmd
        {
            float F;   // N
            float Tp;  // N*m
        } rod;
        struct joint_cmd
        {
            float T[2];  // N*m
            float Pos[2];  // rad
        } joint;
        struct wheel_cmd
        {
            float T;  // N*m
        } wheel;
    } leg[2];  // 0-左 1-右
} Cmd_t;

typedef struct
{
    pid_type_def yaw_angle;
    pid_type_def yaw_velocity;

    pid_type_def vel_add;

#if LOCATION_CONTROL
    pid_type_def roll_angle;

    pid_type_def pitch_angle;
    pid_type_def pitch_vel;
#else

    pid_type_def roll_angle;
    // pid_type_def roll_velocity;

    pid_type_def pitch_angle;
    // pid_type_def pitch_vel;

    pid_type_def leg_length_length[2];
    pid_type_def leg_length_speed[2];

    pid_type_def leg_angle_angle;
#endif

    pid_type_def stand_up;
    pid_type_def wheel_stop[2];
} PID_t;

typedef struct LPF
{
    LowPassFilter_t leg_length_accel_filter[2];
    LowPassFilter_t leg_angle_accel_filter[2];
    LowPassFilter_t support_force_filter[2];
} LPF_t;

/**
 * @brief  底盘数据结构体
 * @note   底盘坐标使用右手系，前进方向为x轴，左方向为y轴，上方向为z轴
 */
typedef struct
{
    const RC_ctrl_t * rc;  // 底盘使用的遥控器指针
    const Imu_t * imu;     // imu数据
    ChassisMode_e mode;    // 底盘模式
    ChassisState_e state;  // 底盘状态
    uint8_t error_code;    // 底盘错误代码

    /*-------------------- Motors --------------------*/
    // 平衡底盘有2个驱动轮电机和4个关节电机
    Motor_s joint_motor[4];
    Motor_s wheel_motor[2];  // 驱动轮电机 0-左轮，1-右轮
    /*-------------------- Values --------------------*/

    Ref_t ref;  // 期望值
    Fdb_t fdb;  // 状态值
    Cmd_t cmd;  // 控制量

    PID_t pid;  // PID控制器
    LPF_t lpf;  // 低通滤波器

    Ratio_t ratio;  // 比例系数

    float dyaw;  // (rad)(feedback)当前位置与云台中值角度差（用于坐标转换）
    uint16_t yaw_mid;  // (ecd)(preset)云台中值角度
} Chassis_s;

typedef struct Calibrate
{
    uint32_t cali_cnt;      //记录遥控器摇杆保持校准姿态的次数（等效于时间）
    float velocity[4];      //关节电机速度
    uint32_t stpo_time[4];  //停止时间
    bool reached[4];        //是否到达限位
    bool calibrated;        //完成校准
} Calibrate_s;

typedef struct GroundTouch
{
    uint32_t touch_time;     //记录触地时间
    float force[2];          //记录腿上的力
    float support_force[2];  //(N)地面的支持力 0-左 1-右

    bool touch;  //是否触地
} GroundTouch_s;

extern void ChassisInit(void);

extern void ChassisHandleException(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif /* CHASSIS_BALANCE */
#endif /* CHASSIS_BALANCE_H */
