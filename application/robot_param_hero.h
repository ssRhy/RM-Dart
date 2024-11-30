/**
  * @file       robot_param_hero.h
  * @brief      为了测试射击写的临时英雄配置文件
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     2024-11-30     CJH/SSR        1. 别急（T-T)
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_NONE                // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                  // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER            // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE  //选择机械臂类型

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//拨弹轮电机PID速度单环
#define TRIGGER_SPEED_PID_KP 1000.0f
#define TRIGGER_SPEED_PID_KI 0.15f
#define TRIGGER_SPEED_PID_KD 0.0f

#define TRIGGER_PID_MAX_OUT 10000.0f
#define TRIGGER_PID_MAX_IOUT 2000.0f

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP 15000.0f
#define FIRC_SPEED_PID_KI 3.0f
#define FRIC_SPEED_PID_KD 0.0f

#define FRIC_PID_MAX_OUT 16000.0f
#define FRIC_PID_MAX_IOUT 7000.0f

//PID parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
