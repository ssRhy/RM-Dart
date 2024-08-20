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
#include "mechanical_arm.h"

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
