/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_engineer.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.1     Aug-20-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "robot_param.h"

#ifndef MECHANICAL_ARM_ENGINEER_H
#define MECHANICAL_ARM_ENGINEER_H

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include "custom_typedef.h"
#include "data_exchange.h"
#include "mechanical_arm.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "user_lib.h"

#define JOINT_NUM 6  // 关节数量

/*-------------------- Structural definition --------------------*/

typedef enum {
    MECHANICAL_ARM_SAFE,
    MECHANICAL_ARM_CALIBRATE,
    MECHANICAL_ARM_FOLLOW,
    MECHANICAL_ARM_DEBUG,
    MECHANICAL_ARM_CUSTOM,
    MECHANICAL_ARM_INIT,
} MechanicalArmMode_e;

/**
 * @brief  机械臂数据结构体
 */
typedef struct
{
    const RC_ctrl_t * rc;      // 遥控器指针
    MechanicalArmMode_e mode;  // 机械臂模式
    uint8_t error_code;        // 机械臂错误代码
    bool init_completed;    // 机械臂初始化完成标志

    uint32_t last_time;  // (ms)上一次更新时间
    uint32_t duration;   // (ms)任务周期
    /*-------------------- Motors --------------------*/
    Motor_s joint_motor[JOINT_NUM];
    /*-------------------- Values --------------------*/

    struct
    {
        struct
        {
            float angle;     // (rad)位置
            float velocity;  // (rad/s)速度
        } joint[JOINT_NUM];
    } ref;

    struct
    {
        struct
        {
            float angle;     // (rad)位置
            float velocity;  // (rad/s)速度
            float torque;    // (N*m)力矩
            int16_t round;    // 圈数(当前位于第几圈中，初始为0)
        } joint[JOINT_NUM];
    } fdb;

    struct
    {
        float dpos[6];
        uint8_t duration[6];
    } transform;

    struct
    {
        struct
        {
            float pos[6];
            float vj4_pos;
        } max;
        struct
        {
            float pos[6];
            float vj4_pos;
        } min;
    } limit;

    // struct
    // {
    //     struct
    //     {
    //         float angle;     // (rad)位置
    //         float velocity;  // (rad/s)速度
    //         float value;     // 电流值
    //     } joint[JOINT_NUM];
    // } cmd;

    struct
    {// 0为角度环，1为速度环
        pid_type_def j0[2];
        pid_type_def j1[2];
        pid_type_def j2[2];
        pid_type_def j3[2];
        pid_type_def j4[2];
        pid_type_def j5[2];
    } pid;

    struct
    {
        LowPassFilter_t j[6];
    } lpf;

} MechanicalArm_s;

/*-------------------- Function Declaration --------------------*/

extern void MechanicalArmPublish(void);
extern void MechanicalArmInit(void);
extern void MechanicalArmHandleException(void);
extern void MechanicalArmSetMode(void);
extern void MechanicalArmObserver(void);
extern void MechanicalArmReference(void);
extern void MechanicalArmConsole(void);
extern void MechanicalArmSendCmd(void);

#endif
#endif  // MECHANICAL_ARM_ENGINEER_H
/*------------------------------ End of File ------------------------------*/
