/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_engineer.c/h
  * @brief      工程机械臂配套自定义控制器功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Aug-22-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "custom_controller_engineer.h"

#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_ENGINEER)

#include "CAN_communication.h"
#include "custom_controller.h"
#include "string.h"

/*------------------------------ Macro Definition ------------------------------*/

#define J0 0
#define J1 1
#define J2 2
#define J3 3
#define J4 4
#define J5 5

#define JointMotorInit(index)                                                                      \
    MotorInit(                                                                                     \
        &CUSTOM_CONTROLLER.joint_motor[index], JOINT_MOTOR_##index##_ID,                           \
        JOINT_MOTOR_##index##_CAN, JOINT_MOTOR_##index##_TYPE, JOINT_MOTOR_##index##_DIRECTION, 1, \
        JOINT_MOTOR_##index##_MODE)

#define JointPidInit(index)                                                             \
    {                                                                                   \
        float j##index##_pid_velocity[3] = {                                            \
            KP_JOINT_##index##_VELOCITY, KI_JOINT_##index##_VELOCITY,                   \
            KD_JOINT_##index##_VELOCITY};                                               \
        PID_init(                                                                       \
            &CUSTOM_CONTROLLER.pid.joint[index], PID_POSITION, j##index##_pid_velocity, \
            MAX_OUT_JOINT_##index##_VELOCITY, MAX_IOUT_JOINT_##index##_VELOCITY);       \
    }
#define JointLpfInit(index) \
    LowPassFilterInit(&CUSTOM_CONTROLLER.lpf.joint[index], J##index##_LPF_ALPHA)

/*------------------------------ Variable Definition ------------------------------*/

CustomController_s CUSTOM_CONTROLLER;

/*------------------------------ Function Definition ------------------------------*/

/******************************************************************/
/* Publish                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerPublish                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerPublish(void) {}

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerInit                       */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerInit(void)
{
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
    // #LPF init ---------------------
    JointLpfInit(0);
    JointLpfInit(1);
    JointLpfInit(2);
    JointLpfInit(3);
    JointLpfInit(4);
    JointLpfInit(5);
    // #Initial value setting ---------------------
    memset(&CUSTOM_CONTROLLER.ref, 0, sizeof(CUSTOM_CONTROLLER.ref));  // 目标量置零
    CUSTOM_CONTROLLER.mode = CUSTOM_CONTROLLER_DRAGGING;
    CUSTOM_CONTROLLER.error_code = 0;
    CUSTOM_CONTROLLER.transform.pos[J0] = J0_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J1] = J1_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J2] = J2_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J3] = J3_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J4] = J4_ANGLE_TRANSFORM;
    CUSTOM_CONTROLLER.transform.pos[J5] = J5_ANGLE_TRANSFORM;
}

/******************************************************************/
/* HandleException                                                */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerHandleException            */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerHandleException(void) {}

/******************************************************************/
/* SetMode                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerSetMode                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerSetMode(void) {}

/******************************************************************/
/* Observer                                                       */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerObserver                   */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerObserver(void)
{
    uint8_t i;
    // 更新电机测量数据
    for (i = 0; i < JOINT_NUM; i++) {
        GetMotorMeasure(&CUSTOM_CONTROLLER.joint_motor[i]);
    }
    // 获取观测值
    float pos;
    for (i = 0; i < JOINT_NUM; i++) {
        pos = theta_transform(
            CUSTOM_CONTROLLER.joint_motor[i].fdb.pos, CUSTOM_CONTROLLER.transform.pos[i], 1, 1);
        CUSTOM_CONTROLLER.fdb.joint[i].dpos = pos - CUSTOM_CONTROLLER.fdb.joint[i].pos;
        CUSTOM_CONTROLLER.fdb.joint[i].pos = pos;
        CUSTOM_CONTROLLER.fdb.joint[i].vel = CUSTOM_CONTROLLER.joint_motor[i].fdb.vel;
    }

    // 更新机械臂控制数据
    cc_control_data.pos[0] = CUSTOM_CONTROLLER.fdb.joint[0].pos;
    cc_control_data.pos[1] = CUSTOM_CONTROLLER.fdb.joint[1].pos;
    cc_control_data.pos[2] = CUSTOM_CONTROLLER.fdb.joint[2].pos;
    cc_control_data.pos[3] = CUSTOM_CONTROLLER.fdb.joint[3].pos;

    cc_control_data.pos[4] += CUSTOM_CONTROLLER.fdb.joint[4].dpos;
    cc_control_data.pos[5] += CUSTOM_CONTROLLER.fdb.joint[4].dpos;

    cc_control_data.pos[4] += CUSTOM_CONTROLLER.fdb.joint[5].dpos;
    cc_control_data.pos[5] -= CUSTOM_CONTROLLER.fdb.joint[5].dpos;
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerReference                  */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerReference(void) {}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerConsole                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerConsole(void)
{
    uint8_t i;
    // 计算控制量
    for (i = 0; i < JOINT_NUM; i++) {
        // CUSTOM_CONTROLLER.joint_motor[i].set.value = PID_calc(
        //     &CUSTOM_CONTROLLER.pid.joint[i], CUSTOM_CONTROLLER.fdb.joint[i].vel,
        //     CUSTOM_CONTROLLER.ref.joint[i].vel);
        CUSTOM_CONTROLLER.joint_motor[i].set.value = 0;
    }
}

/******************************************************************/
/* SendCmd                                                        */
/*----------------------------------------------------------------*/
/* main function:      CustomControllerSendCmd                    */
/* auxiliary function: None                                       */
/******************************************************************/

void CustomControllerSendCmd(void)
{
    // clang-format off
    CanCmdDjiMotor(
        1, DJI_6020_MODE_VOLTAGE_1, 
        CUSTOM_CONTROLLER.joint_motor[0].set.value,
        CUSTOM_CONTROLLER.joint_motor[1].set.value, 
        CUSTOM_CONTROLLER.joint_motor[2].set.value, 0);
    CanCmdDjiMotor(
        2, DJI_3508_MODE_CURRENT_1, 
        CUSTOM_CONTROLLER.joint_motor[3].set.value,
        CUSTOM_CONTROLLER.joint_motor[4].set.value, 
        CUSTOM_CONTROLLER.joint_motor[5].set.value, 0);
    // clang-format on
}

#endif  // CUSTOM_CONTROLLER_TYPE
/*------------------------------ End of File ------------------------------*/
