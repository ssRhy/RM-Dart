/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       custom_controller_engineer.c/h
  * @brief      机械臂功能。
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

#ifndef CUSTOM_CONTROLLER_ENGINEER_H
#define CUSTOM_CONTROLLER_ENGINEER_H
#include "robot_param.h"

#if (CUSTOM_CONTROLLER_TYPE == CUSTOM_CONTROLLER_ENGINEER)
extern void CustomControllerPublish(void);
extern void CustomControllerInit(void);
extern void CustomControllerHandleException(void);
extern void CustomControllerSetMode(void);
extern void CustomControllerObserver(void);
extern void CustomControllerReference(void);
extern void CustomControllerConsole(void);
extern void CustomControllerSendCmd(void);
#endif
#endif  // CUSTOM_CONTROLLER_ENGINEER_H
/*------------------------------ End of File ------------------------------*/
