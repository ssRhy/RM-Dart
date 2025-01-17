/**
  * @file       robot_param_hero.h
  * @brief      英雄配置文件
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024-12-5       CJH             只包含射击所需参数
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE                // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

/*-------------------- Shoot --------------------*/

#define BULLET_NUM 6  // 定义拨弹盘容纳弹丸个数

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机ID
#define TRIGGER_MOTOR_ID 7
#define FRIC_MOTOR_R_ID 1
#define FRIC_MOTOR_L_ID 2

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机std_id
#define TRIGGER_STD_ID 0x1FF
#define FRIC_STD_ID 0x200

//单环拨弹速度
#define TRIGGER_SPEED (500.0f)
//摩擦轮速度
#define FRIC_SPEED (70.0f)
#define FRIC_SPEED_LIMIT (60.0f)

//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.00001503902725490196f
#define FULL_COUNT 25

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 1.0f
#define BLOCK_TIME 500
#define REVERSE_TIME 250
#define REVERSE_SPEED_LIMIT 13.0f
#define REVERSE_SPEED (-5.0f)  // (rad/s)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度单环
#define TRIGGER_SPEED_PID_KP (200.0f)
#define TRIGGER_SPEED_PID_KI (0.1f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (16000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

// 单发模式 拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (100.0f)
#define TRIGGER_ANGEL_PID_KI (0.5f)
#define TRIGGER_ANGEL_PID_KD (0.0f)

#define TRIGGER_ANGEL_PID_MAX_OUT (700.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (1000.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (500.0f)
#define FIRC_SPEED_PID_KI (0.1f)
#define FRIC_SPEED_PID_KD (0.03f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (10.0f)

#endif /* INCLUDED_ROBOT_PARAM_H */
