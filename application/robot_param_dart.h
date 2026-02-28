/**
****************************(C) COPYRIGHT 2024 Polarbear****************************
* @file       robot_param_dart.h
* @brief      飞镖机器人参数配置文件
* @note       飞镖专用参数配置
* @history
*  Version    Date            Author          Modification
*  V1.0.0     Dec-2024        Assistant       1. 创建飞镖参数配置
*
@verbatim
==============================================================================
飞镖机器人参数配置
==============================================================================
@endverbatim
****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#ifndef ROBOT_PARAM_DART_H
#define ROBOT_PARAM_DART_H

// 飞镖多板配置
#define DART_BOARD_MAIN    1  // 主控板：负责拨弹(feed) + 横移(trans) + 底盘(chassis)
#define DART_BOARD_SHOOT   2  // 射击板：负责射击摩擦轮(shoot)
#define DART_BOARD_TYPE    DART_BOARD_MAIN  // ← 烧录前在此选择目标板

// 底盘类型选择
#define CHASSIS_TYPE DART_CHASSIS  // 选择底盘类型为飞镖

// 飞镖电机参数
#define DART_TRANS_MOTOR_ID 4            // 飞镖电机ID
#define MOTOR_DART_CAN 1           // 飞镖电机CAN总线
#define MOTOR_TRANS_TYPE DJI_M3508  // 飞镖电机类型
#define MOTOR_DART_DIRECTION 1     // 飞镖电机方向
#define MOTOR_DART_REDUCTION 1.0f  // 飞镖电机减速比
#define MOTOR_DART_MODE 0          // 飞镖电机模式

// 任务相关参数
#define DART_TASK_INIT_TIME 200  // 飞镖任务初始化时间(ms)
#define DART_CONTROL_TIME_MS 2   // 飞镖控制周期(ms)

//PID
//飞镖电机速度环PID
#define DART_SPEED_PID_KP (120.0f)
#define DART_SPEED_PID_KI (1.0f)
#define DART_SPEED_PID_KD (1.0f)

#define DART_PID_MAX_OUT (10000.0f)
#define DART_PID_MAX_IOUT (1000.0f)

//飞镖电机角度环PID
#define DART_ANGEL_PID_KP (30.0f)
#define DART_ANGEL_PID_KI (0.05f)
#define DART_ANGEL_PID_KD (0.05f)

#define DART_ANGEL_PID_MAX_OUT (300.0f)
#define DART_ANGEL_PID_MAX_IOUT (30.0f)
//飞镖速度限制
#define DART_SPEED (3.0f)
//飞镖电机标准ID
#define DART_TRANS_STD_ID (0x200)
//飞镖电机CAN总线
#define DART_CAN (1)

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191



//飞镖模式切换时间(ms)
#define CHANGE_TIME (1500)
//停止速度
#define STOP_SPEED (0.0f) 

// Feed motor PID parameters
#define FEED_ANGEL_PID_KP (30.0f)
#define FEED_ANGEL_PID_KI (0.05f)
#define FEED_ANGEL_PID_KD (0.05f)
#define FEED_ANGEL_PID_MAX_OUT (300.0f)
#define FEED_ANGEL_PID_MAX_IOUT (30.0f)
#define FEED_SPEED_PID_KP (120.0f)
#define FEED_SPEED_PID_KI (1.0f)
#define FEED_SPEED_PID_KD (1.0f)
#define FEED_PID_MAX_OUT (10000.0f)
#define FEED_PID_MAX_IOUT (1000.0f)
// Feed motor CAN parameters
#define FEED_STD_ID (0x200)
#define FEED_CAN (1)


// Chassis motor PID parameters
#define CHASSIS_ANGEL_PID_KP (30.0f)
#define CHASSIS_ANGEL_PID_KI (0.05f)
#define CHASSIS_ANGEL_PID_KD (0.05f)
#define CHASSIS_ANGEL_PID_MAX_OUT (300.0f)
#define CHASSIS_ANGEL_PID_MAX_IOUT (30.0f)

#define CHASSIS_SPEED_PID_KP (120.0f)
#define CHASSIS_SPEED_PID_KI (1.0f)
#define CHASSIS_SPEED_PID_KD (1.0f)
#define CHASSIS_PID_MAX_OUT (10000.0f)
#define CHASSIS_PID_MAX_IOUT (1000.0f)

// Chassis motor CAN parameters
#define CHASSIS_STD_ID (0x1FF)
#define CHASSIS_CAN (1)

// ===================== 射击机构参数 =====================
// 摩擦轮电机 ID（CAN 报文位置）
#define SHOOT_MOTOR_L_ID    (1)         // 左摩擦轮电机 ID
#define SHOOT_MOTOR_R_ID    (2)         // 右摩擦轮电机 ID

// 射击电机 CAN 总线及标准帧 ID
#define SHOOT_CAN           (1)
#define SHOOT_STD_ID_1        (0x200)
#define SHOOT_STD_ID_2        (0x1FF)

// 摩擦轮速度设定（rad/s，与 MOTOR_RPM_TO_SPEED 对应）
#define SHOOT_READY_SPEED_FRONT (100.0f)
#define SHOOT_READY_SPEED_MID (70.0f)
#define SHOOT_READY_SPEED_REAR (70.0f)

// 发射持续时间（ms）：摩擦轮维持全速的时长
#define SHOOT_LAUNCH_TIME   (300)

// 摩擦轮速度环 PID
#define SHOOT_FRONT_PID_KP  (150.0f)
#define SHOOT_FRONT_PID_KI  (1.0f)
#define SHOOT_FRONT_PID_KD  (0.0f)

#define SHOOT_MID_PID_KP  (150.0f)
#define SHOOT_MID_PID_KI  (1.0f)
#define SHOOT_MID_PID_KD  (0.0f)

#define SHOOT_REAR_PID_KP  (150.0f)
#define SHOOT_REAR_PID_KI  (1.0f)
#define SHOOT_REAR_PID_KD  (0.0f)

#define SHOOT_FRONT_PID_MAX_OUT (16000.0f)
#define SHOOT_FRONT_PID_MAX_IOUT (2000.0f)

#define SHOOT_MID_PID_MAX_OUT (16000.0f)
#define SHOOT_MID_PID_MAX_IOUT (2000.0f)

#define SHOOT_REAR_PID_MAX_OUT (16000.0f)
#define SHOOT_REAR_PID_MAX_IOUT (2000.0f)



#endif /* ROBOT_PARAM_DART_H */
