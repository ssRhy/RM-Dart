/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       mechanical_arm_engineer.c/h
  * @brief      机械臂功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-20-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_engineer.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include <stdbool.h>

#include "CAN_communication.h"
#include "bsp_delay.h"
#include "custom_controller_connect.h"
#include "math.h"
#include "pid.h"
#include "signal_generator.h"
#include "usb_debug.h"

/*------------------------------ Macro Definition ------------------------------*/

#define ANGLE_PID 0
#define VELOCITY_PID 1

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5

#define DM_DELAY 250  // (us)dm电机发送延时
#define DM_KP_FOLLOW 1
#define DM_KD_FOLLOW 0.5

#define JointMotorInit(index)                                                                    \
    MotorInit(                                                                                   \
        &MECHANICAL_ARM.joint_motor[index], JOINT_MOTOR_##index##_ID, JOINT_MOTOR_##index##_CAN, \
        JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION, 1,                          \
        JOINT_MOTOR_##index##_MODE)

#define JointPidInit(index)                                                                \
    {                                                                                      \
        float j##index##_pid_angle[3] = {                                                  \
            KP_JOINT_##index##_ANGLE, KI_JOINT_##index##_ANGLE, KD_JOINT_##index##_ANGLE}; \
        float j##index##_pid_velocity[3] = {                                               \
            KP_JOINT_##index##_VELOCITY, KI_JOINT_##index##_VELOCITY,                      \
            KD_JOINT_##index##_VELOCITY};                                                  \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[0], PID_POSITION, j##index##_pid_angle,         \
            MAX_OUT_JOINT_##index##_ANGLE, MAX_IOUT_JOINT_##index##_ANGLE);                \
        PID_init(                                                                          \
            &MECHANICAL_ARM.pid.j##index##[1], PID_POSITION, j##index##_pid_velocity,      \
            MAX_OUT_JOINT_##index##_VELOCITY, MAX_IOUT_JOINT_##index##_VELOCITY);          \
    }

/*------------------------------ Variable Definition ------------------------------*/

MechanicalArm_s MECHANICAL_ARM;

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/******************************************************************/

void MechanicalArmPublish(void) {}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      MechanicalArmInit                          */
/* auxiliary function: None                                       */
/******************************************************************/

void MechanicalArmInit(void)
{
    MECHANICAL_ARM.rc = get_remote_control_point();
    // #Motor init ---------------------
    JointMotorInit(0);
    JointMotorInit(1);
    JointMotorInit(2);
    JointMotorInit(3);
    JointMotorInit(4);
    JointMotorInit(5);
    // #PID init ---------------------
    JointPidInit(0);
    JointPidInit(4);
    JointPidInit(5);
    // #Initial value setting ---------------------
    MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
}

/******************************************************************/
/* HandleException                                                */
/******************************************************************/

void MechanicalArmHandleException(void) {}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSetMode                      */
/* auxiliary function:  None                                      */
/******************************************************************/

void MechanicalArmSetMode(void)
{
    if (MECHANICAL_ARM.mode == MECHANICAL_ARM_CALIBRATE) {
        return;
    }

    if (switch_is_up(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_CUSTOM;
    } else if (switch_is_mid(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_DEBUG;
    } else if (switch_is_down(MECHANICAL_ARM.rc->rc.s[MECHANICAL_ARM_MODE_CHANNEL])) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    }
}

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmObserver                     */
/* auxiliary function:  UpdateMotorStatus                         */
/*                      JointStateObserve                         */
/******************************************************************/

static void UpdateMotorStatus(void);
static void JointStateObserve(void);

void MechanicalArmObserver(void)
{
    UpdateMotorStatus();
    JointStateObserve();
}

/**
 * @brief  更新电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    uint8_t i;
    for (i = 0; i < 6; i++) {
        GetMotorMeasure(&MECHANICAL_ARM.joint_motor[i]);
    }
}

/**
 * @brief  关节状态观测
 * @param  none
 */
static void JointStateObserve(void)
{
    uint8_t i;
    for (i = 0; i < 6; i++) {
        MECHANICAL_ARM.fdb.joint[i].angle = MECHANICAL_ARM.joint_motor[i].fdb.pos;
        MECHANICAL_ARM.fdb.joint[i].velocity = MECHANICAL_ARM.joint_motor[i].fdb.vel;
        MECHANICAL_ARM.fdb.joint[i].torque = MECHANICAL_ARM.joint_motor[i].fdb.tor;
    }
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmReference                    */
/******************************************************************/

void MechanicalArmReference(void)
{
    uint8_t i;
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_CUSTOM: {
            MECHANICAL_ARM.ref.joint[J0].angle = MECHANICAL_ARM.rc->rc.ch[4] * RC_TO_ONE * M_PI;
            MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.rc->rc.ch[0] * RC_TO_ONE * M_PI;
            MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.rc->rc.ch[1] * RC_TO_ONE * M_PI;
            MECHANICAL_ARM.ref.joint[J3].angle = 0;
            MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.rc->rc.ch[2] * RC_TO_ONE * M_PI;
            MECHANICAL_ARM.ref.joint[J5].angle = MECHANICAL_ARM.rc->rc.ch[3] * RC_TO_ONE * M_PI;
        } break;
        case MECHANICAL_ARM_DEBUG:
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            for (i = 0; i < 6; i++) {
                MECHANICAL_ARM.ref.joint[i].velocity = 0;
            }
        }
    }
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmConsole                      */
/******************************************************************/

void MechanicalArmConsole(void)
{
    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_CUSTOM: {
            // 优先处理dm电机部分
            // 位置
            MECHANICAL_ARM.joint_motor[J1].set.pos =
                theta_transform(MECHANICAL_ARM.ref.joint[J1].angle, -J1_ANGLE_TRANSFORM, 1, 1);
            MECHANICAL_ARM.joint_motor[J2].set.pos =
                theta_transform(MECHANICAL_ARM.ref.joint[J2].angle, -J2_ANGLE_TRANSFORM, 1, 1);
            MECHANICAL_ARM.joint_motor[J3].set.pos =
                theta_transform(MECHANICAL_ARM.ref.joint[J3].angle, -J3_ANGLE_TRANSFORM, 1, 1);
            // 速度
            MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J3].set.vel = 0;

            // 然后再处理dji电机部分，涉及到pid计算
            // J0
            PID_calc(
                &MECHANICAL_ARM.pid.j0[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J0].angle,
                MECHANICAL_ARM.ref.joint[J0].angle);
            MECHANICAL_ARM.joint_motor[J0].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j0[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J0].velocity,
                MECHANICAL_ARM.pid.j0[ANGLE_PID].out);
            // J4
            PID_calc(
                &MECHANICAL_ARM.pid.j4[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J4].angle,
                MECHANICAL_ARM.ref.joint[J4].angle);
            MECHANICAL_ARM.joint_motor[J4].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j4[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J4].velocity,
                MECHANICAL_ARM.pid.j4[ANGLE_PID].out);
            // J5
            PID_calc(
                &MECHANICAL_ARM.pid.j5[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J5].angle,
                MECHANICAL_ARM.ref.joint[J5].angle);
            MECHANICAL_ARM.joint_motor[J5].set.value = PID_calc(
                &MECHANICAL_ARM.pid.j5[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J5].velocity,
                MECHANICAL_ARM.pid.j5[ANGLE_PID].out);
        } break;
        case MECHANICAL_ARM_DEBUG:
        case MECHANICAL_ARM_FOLLOW:
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_SAFE:
        default: {
            MECHANICAL_ARM.joint_motor[J0].set.value = 0;
            MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J3].set.vel = 0;
            MECHANICAL_ARM.joint_motor[J4].set.value = 0;
            MECHANICAL_ARM.joint_motor[J5].set.value = 0;
        }
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:       MechanicalArmSendCmd                      */
/* auxiliary function:  ArmSendCmdSafe                            */
/*                      ArmSendCmdFollow                          */
/******************************************************************/

void ArmSendCmdSafe(void);
void ArmSendCmdFollow(void);

void MechanicalArmSendCmd(void)
{
    uint8_t cnt;
    for (uint8_t i = 0; i < 4; i++) {
        if (cnt % 2 == 0) {
            delay_us(DM_DELAY);
        }
        if (MECHANICAL_ARM.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
            DmEnable(&MECHANICAL_ARM.joint_motor[i]);
            cnt++;
        }
    }

    delay_us(DM_DELAY);

    switch (MECHANICAL_ARM.mode) {
        case MECHANICAL_ARM_FOLLOW: {
            ArmSendCmdFollow();
        } break;
        case MECHANICAL_ARM_CALIBRATE:
        case MECHANICAL_ARM_DEBUG:
        case MECHANICAL_ARM_CUSTOM:
        case MECHANICAL_ARM_SAFE:
        default: {
            ArmSendCmdSafe();
        }
    }
}

void ArmSendCmdSafe(void)
{
    DmMitStop(&MECHANICAL_ARM.joint_motor[J1]);
    delay_us(DM_DELAY);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J2]);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J3]);
    CanCmdDjiMotor(ARM_DJI_CAN, 0x1FF, 0, 0, 0, 0);  // J0 J4 J5
}

void ArmSendCmdFollow(void)
{
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J1], DM_KP_FOLLOW, DM_KD_FOLLOW);
    delay_us(DM_DELAY);
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J2], DM_KP_FOLLOW, DM_KD_FOLLOW);
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J3], DM_KP_FOLLOW, DM_KD_FOLLOW);
    // clang-format off
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF, 
        MECHANICAL_ARM.joint_motor[J0].set.value,
        MECHANICAL_ARM.joint_motor[J4].set.value, 
        MECHANICAL_ARM.joint_motor[J5].set.value, 0); // J0 J4 J5
    // clang-format on
}

#endif
/*------------------------------ End of File ------------------------------*/
