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
/* main function: MechanicalArmInit                               */
/* auxiliary function: MechanicalArmReset                         */
/******************************************************************/

void MechanicalArmReset(void);

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
    // #Low pass filter init ---------------------
}

void MechanicalArmReset(void)
{
    // #PID reset ---------------------
    // #Low pass filter reset ---------------------
}

/******************************************************************/
/* HandleException                                                */
/******************************************************************/

void MechanicalArmHandleException(void) {}

/******************************************************************/
/* SetMode                                                        */
/******************************************************************/

void MechanicalArmSetMode(void) {}

/******************************************************************/
/* Observer                                                       */
/******************************************************************/

void MechanicalArmObserver(void) {}

/******************************************************************/
/* Reference                                                      */
/******************************************************************/

void MechanicalArmReference(void) {}

/******************************************************************/
/* Console                                                        */
/******************************************************************/

void MechanicalArmConsole(void) {}

/******************************************************************/
/* SendCmd                                                        */
/******************************************************************/

void MechanicalArmSendCmd(void) {}

#endif
/*------------------------------ End of File ------------------------------*/
