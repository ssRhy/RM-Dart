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
机械臂相关的一些方向定义
    - 机械臂水平向前时设置J0关节为0位置
    - 机械臂竖直向上时设置J1 J2 J3关节为0位置

    - 定义机械臂水平向前时的J0关节位置为0，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，从左看，顺时针为正方向（J1 J2关节的位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0，从吸盘方向看，逆时针为正方向

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "mechanical_arm_engineer.h"

#if (MECHANICAL_ARM_TYPE == MECHANICAL_ARM_ENGINEER_ARM)
#include <stdbool.h>

#include "CAN_communication.h"
#include "bsp_delay.h"
#include "cmsis_os.h"
#include "custom_controller_connect.h"
#include "detect_task.h"
#include "math.h"
#include "pid.h"
#include "remote_control.h"
#include "signal_generator.h"
#include "usb_debug.h"

/*------------------------------ Macro Definition ------------------------------*/

#define MS_TO_S 0.001f  // ms转s

#define ANGLE_PID 0
#define VELOCITY_PID 1

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5

#define DM_DELAY 250  // (us)dm电机发送延时

#define J0_KP_FOLLOW 0
#define J0_KD_FOLLOW 1.5

#define J1_KP_FOLLOW 1
#define J1_KD_FOLLOW 0.5

#define J2_KP_FOLLOW 1
#define J2_KD_FOLLOW 0.5

#define JointMotorInit(index)                                                                    \
    MotorInit(                                                                                   \
        &MECHANICAL_ARM.joint_motor[index], JOINT_MOTOR_##index##_ID, JOINT_MOTOR_##index##_CAN, \
        JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION,                             \
        JOINT_MOTOR_##index##_REDUCATION_RATIO, JOINT_MOTOR_##index##_MODE)

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
#define MA MECHANICAL_ARM

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
    JointPidInit(1);
    JointPidInit(2);
    JointPidInit(3);
    JointPidInit(4);
    JointPidInit(5);
    // #Initial value setting ---------------------
    MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
    MECHANICAL_ARM.error_code = 0;
    MECHANICAL_ARM.transform.dpos[J0] = J0_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J1] = J1_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J2] = J2_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J3] = J3_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J4] = J4_ANGLE_TRANSFORM;
    MECHANICAL_ARM.transform.dpos[J5] = J5_ANGLE_TRANSFORM;

    MECHANICAL_ARM.transform.duration[J0] = 4;
    MECHANICAL_ARM.transform.duration[J1] = 4;
    MECHANICAL_ARM.transform.duration[J2] = 4;
    MECHANICAL_ARM.transform.duration[J3] = 2;
    MECHANICAL_ARM.transform.duration[J4] = 1;
    MECHANICAL_ARM.transform.duration[J5] = 1;
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

    if (toe_is_error(DBUS_TOE)) {
        MECHANICAL_ARM.mode = MECHANICAL_ARM_SAFE;
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
    MA.duration = xTaskGetTickCount() - MA.last_time;
    MA.last_time = xTaskGetTickCount();

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
#define dangle MECHANICAL_ARM.transform.dpos

    /*-----处理J0 J1 J2 J3 关节的反馈信息（J4 J5作为差速机构要特殊处理）*/
    uint8_t i;
    for (i = 0; i < 4; i++) {
        MA.fdb.joint[i].angle = theta_transform(
                                    MA.joint_motor[i].fdb.pos, dangle[i],
                                    MA.joint_motor[i].direction, MA.transform.duration[i]) /
                                MA.joint_motor[i].reduction_ratio;
        MA.fdb.joint[i].velocity = MA.joint_motor[i].fdb.vel / MA.joint_motor[i].reduction_ratio *
                                   MA.joint_motor[i].direction;
        MA.fdb.joint[i].torque = MA.joint_motor[i].fdb.tor * MA.joint_motor[i].reduction_ratio *
                                 MA.joint_motor[i].direction;
    }
#undef dangle

    /*-----处理J4 J5 关节的反馈信息*/
    MA.fdb.joint[J4].velocity = MA.joint_motor[J4].fdb.vel;
    MA.fdb.joint[J5].velocity = MA.joint_motor[J5].fdb.vel;

    MA.fdb.joint[J4].angle += MA.fdb.joint[J4].velocity * MA.duration * MS_TO_S;
    MA.fdb.joint[J5].angle += MA.fdb.joint[J5].velocity * MA.duration * MS_TO_S;
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
            // MECHANICAL_ARM.ref.joint[J0].angle = MECHANICAL_ARM.rc->rc.ch[4] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J1].angle = MECHANICAL_ARM.rc->rc.ch[0] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J2].angle = MECHANICAL_ARM.rc->rc.ch[1] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J3].angle = 0;
            // MECHANICAL_ARM.ref.joint[J4].angle = MECHANICAL_ARM.rc->rc.ch[2] * RC_TO_ONE * M_PI;
            // MECHANICAL_ARM.ref.joint[J5].angle = MECHANICAL_ARM.rc->rc.ch[3] * RC_TO_ONE * M_PI;
        } break;
        case MECHANICAL_ARM_DEBUG: {
            MECHANICAL_ARM.ref.joint[J0].angle = GetDt7RcCh(DT7_CH_RH) * M_PI / 3;
            MECHANICAL_ARM.ref.joint[J1].angle = 0;
            MECHANICAL_ARM.ref.joint[J2].angle = 0;
            MECHANICAL_ARM.ref.joint[J3].angle = 0;
            MECHANICAL_ARM.ref.joint[J4].angle = 0;
            MECHANICAL_ARM.ref.joint[J5].angle = 0;
        } break;
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
            // // 优先处理dm电机部分
            // // 位置
            // MECHANICAL_ARM.joint_motor[J0].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J0].angle, -J0_ANGLE_TRANSFORM, 1, 1);
            // MECHANICAL_ARM.joint_motor[J1].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J1].angle, -J1_ANGLE_TRANSFORM, 1, 1);
            // MECHANICAL_ARM.joint_motor[J2].set.pos =
            //     theta_transform(MECHANICAL_ARM.ref.joint[J2].angle, -J2_ANGLE_TRANSFORM, 1, 1);
            // // 速度
            // MECHANICAL_ARM.joint_motor[J1].set.vel = 0;
            // MECHANICAL_ARM.joint_motor[J2].set.vel = 0;
            // MECHANICAL_ARM.joint_motor[J3].set.vel = 0;

            // // 然后再处理dji电机部分，涉及到pid计算
            // // J0
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j0[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J0].angle,
            //     MECHANICAL_ARM.ref.joint[J0].angle);
            // MECHANICAL_ARM.joint_motor[J0].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j0[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J0].velocity,
            //     MECHANICAL_ARM.pid.j0[ANGLE_PID].out);
            // // J4
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j4[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J4].angle,
            //     MECHANICAL_ARM.ref.joint[J4].angle);
            // MECHANICAL_ARM.joint_motor[J4].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j4[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J4].velocity,
            //     MECHANICAL_ARM.pid.j4[ANGLE_PID].out);
            // // J5
            // PID_calc(
            //     &MECHANICAL_ARM.pid.j5[ANGLE_PID], MECHANICAL_ARM.fdb.joint[J5].angle,
            //     MECHANICAL_ARM.ref.joint[J5].angle);
            // MECHANICAL_ARM.joint_motor[J5].set.value = PID_calc(
            //     &MECHANICAL_ARM.pid.j5[VELOCITY_PID], MECHANICAL_ARM.fdb.joint[J5].velocity,
            //     MECHANICAL_ARM.pid.j5[ANGLE_PID].out);
        } break;
        case MECHANICAL_ARM_DEBUG: {
            //机械臂基本不会出现过圈问题，不考虑过圈时的最优旋转方向问题。

            // DM电机
            MA.joint_motor[J0].set.vel =
                PID_calc(&MA.pid.j0[0], MA.fdb.joint[J0].angle, MA.ref.joint[J0].angle) *
                MA.joint_motor[J0].direction * MA.joint_motor[J0].reduction_ratio;
            MA.joint_motor[J0].set.tor = 0;

            MA.joint_motor[J1].set.pos = 0;
            MA.joint_motor[J1].set.vel = 0;
            MA.joint_motor[J1].set.tor = 0;

            MA.joint_motor[J2].set.pos = 0;
            MA.joint_motor[J2].set.vel = 0;
            MA.joint_motor[J2].set.tor = 0;
        } break;
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
void ArmSendCmdDebug(void);
void ArmSendCmdFollow(void);

void MechanicalArmSendCmd(void)
{
    uint8_t cnt;
    for (uint8_t i = 0; i < 3; i++) {
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
        case MECHANICAL_ARM_DEBUG: {
            ArmSendCmdDebug();
        } break;
        case MECHANICAL_ARM_CUSTOM:
        case MECHANICAL_ARM_SAFE:
        default: {
            ArmSendCmdSafe();
        }
    }
    ModifyDebugDataPackage(1, MA.fdb.joint[J0].angle, "J0_pos_f");
    ModifyDebugDataPackage(2, MA.ref.joint[J0].angle, "J0_pos_r");
    ModifyDebugDataPackage(3, MA.fdb.joint[J0].velocity, "J0_vel_f");
    ModifyDebugDataPackage(4, MA.pid.j0[0].out * 2.24f, "pid0 out");
}

void ArmSendCmdSafe(void)
{
    DmMitStop(&MECHANICAL_ARM.joint_motor[J0]);
    delay_us(DM_DELAY);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J1]);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J2]);
    CanCmdDjiMotor(ARM_DJI_CAN, 0x1FF, 0, 0, 0, 0);  // J3 J4 J5
}

void ArmSendCmdDebug(void)
{
    DmMitCtrlVelocity(&MECHANICAL_ARM.joint_motor[J0], J0_KD_FOLLOW);
    delay_us(DM_DELAY);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J1]);
    DmMitStop(&MECHANICAL_ARM.joint_motor[J2]);
    CanCmdDjiMotor(ARM_DJI_CAN, 0x1FF, 0, 0, 0, 0);  // J3 J4 J5
}

void ArmSendCmdFollow(void)
{
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J0], J0_KP_FOLLOW, J0_KD_FOLLOW);
    delay_us(DM_DELAY);
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J1], J1_KP_FOLLOW, J1_KD_FOLLOW);
    DmMitCtrl(&MECHANICAL_ARM.joint_motor[J2], J2_KP_FOLLOW, J2_KD_FOLLOW);
    // clang-format off
    CanCmdDjiMotor(
        ARM_DJI_CAN, 0x1FF, 
        MECHANICAL_ARM.joint_motor[J0].set.value,
        MECHANICAL_ARM.joint_motor[J4].set.value, 
        MECHANICAL_ARM.joint_motor[J5].set.value, 0); // J3 J4 J5
    // clang-format on
}

#endif

#undef MA
/*------------------------------ End of File ------------------------------*/
