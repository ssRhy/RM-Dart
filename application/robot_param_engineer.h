/**
  * @file       robot_param_engineer.h
  * @brief      这里是工程机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "motor.h"
#include "robot_typedef.h"

#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_ENGINEER_ARM    // 选择机械臂类型
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_ENGINEER  // 选择自定义控制器类型

// 机器人物理参数

/*-------------------- Chassis --------------------*/
//motor type
//physical parameters ---------------------
//upper_limit parameters ---------------------
//lower_limit parameters ---------------------
//PID parameters ---------------------

/*-------------------- Mechanical arm --------------------*/
#define ARM_DJI_CAN 1
#define ARM_DM_CAN 2
//motor type ---------------------

#define JOINT_MOTOR_0_ID 1
#define JOINT_MOTOR_1_ID 1
#define JOINT_MOTOR_2_ID 2
#define JOINT_MOTOR_3_ID 3
#define JOINT_MOTOR_4_ID 6
#define JOINT_MOTOR_5_ID 7

#define JOINT_MOTOR_0_CAN ARM_DJI_CAN
#define JOINT_MOTOR_1_CAN ARM_DM_CAN
#define JOINT_MOTOR_2_CAN ARM_DM_CAN
#define JOINT_MOTOR_3_CAN ARM_DM_CAN
#define JOINT_MOTOR_4_CAN ARM_DJI_CAN
#define JOINT_MOTOR_5_CAN ARM_DJI_CAN

#define JOINT_MOTOR_0_TYPE DJI_M6020
#define JOINT_MOTOR_1_TYPE DM_8009
#define JOINT_MOTOR_2_TYPE DM_8009
#define JOINT_MOTOR_3_TYPE DM_4340
#define JOINT_MOTOR_4_TYPE DJI_M2006
#define JOINT_MOTOR_5_TYPE DJI_M2006

#define JOINT_MOTOR_0_DIRECTION 1
#define JOINT_MOTOR_1_DIRECTION 1
#define JOINT_MOTOR_2_DIRECTION 1
#define JOINT_MOTOR_3_DIRECTION 1
#define JOINT_MOTOR_4_DIRECTION 1
#define JOINT_MOTOR_5_DIRECTION 1

#define JOINT_MOTOR_0_MODE 0
#define JOINT_MOTOR_1_MODE DM_MODE_MIT
#define JOINT_MOTOR_2_MODE DM_MODE_MIT
#define JOINT_MOTOR_3_MODE DM_MODE_MIT
#define JOINT_MOTOR_4_MODE 0
#define JOINT_MOTOR_5_MODE 0
//upper_limit parameters ---------------------
#define MAX_JOINT_0_POSITION 6.283185f  //2*M_PI
#define MAX_JOINT_1_POSITION M_PI
#define MAX_JOINT_2_POSITION M_PI
#define MAX_JOINT_3_POSITION 6.283185f
#define MAX_JOINT_4_POSITION M_PI
//lower_limit parameters ---------------------
#define MIN_JOINT_0_POSITION 0.0f
#define MIN_JOINT_1_POSITION 0.0f
#define MIN_JOINT_2_POSITION 0.0f
#define MIN_JOINT_3_POSITION 0.0f
#define MIN_JOINT_4_POSITION 0.0f
//PID parameters ---------------------
//J0角度环PID参数
#define KP_JOINT_0_ANGLE 0.0f
#define KI_JOINT_0_ANGLE 0.0f
#define KD_JOINT_0_ANGLE 0.0f
#define MAX_IOUT_JOINT_0_ANGLE 0.0f
#define MAX_OUT_JOINT_0_ANGLE 0.0f
//J0速度环PID参数
#define KP_JOINT_0_VELOCITY 0.0f
#define KI_JOINT_0_VELOCITY 0.0f
#define KD_JOINT_0_VELOCITY 0.0f
#define MAX_IOUT_JOINT_0_VELOCITY 0.0f
#define MAX_OUT_JOINT_0_VELOCITY 0.0f
//J4角度环PID参数
#define KP_JOINT_4_ANGLE 0.0f
#define KI_JOINT_4_ANGLE 0.0f
#define KD_JOINT_4_ANGLE 0.0f
#define MAX_IOUT_JOINT_4_ANGLE 0.0f
#define MAX_OUT_JOINT_4_ANGLE 0.0f
//J4速度环PID参数
#define KP_JOINT_4_VELOCITY 0.0f
#define KI_JOINT_4_VELOCITY 0.0f
#define KD_JOINT_4_VELOCITY 0.0f
#define MAX_IOUT_JOINT_4_VELOCITY 0.0f
#define MAX_OUT_JOINT_4_VELOCITY 0.0f
//J5角度环PID参数
#define KP_JOINT_5_ANGLE 0.0f
#define KI_JOINT_5_ANGLE 0.0f
#define KD_JOINT_5_ANGLE 0.0f
#define MAX_IOUT_JOINT_5_ANGLE 0.0f
#define MAX_OUT_JOINT_5_ANGLE 0.0f
//J5速度环PID参数
#define KP_JOINT_5_VELOCITY 0.0f
#define KI_JOINT_5_VELOCITY 0.0f
#define KD_JOINT_5_VELOCITY 0.0f
#define MAX_IOUT_JOINT_5_VELOCITY 0.0f
#define MAX_OUT_JOINT_5_VELOCITY 0.0f
// Init parameters ---------------------
// Other parameters ---------------------

#endif /* INCLUDED_ROBOT_PARAM_H */
