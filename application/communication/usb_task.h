/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#ifndef __USB_TASK_H
#define __USB_TASK_H
#include "robot_param.h"

extern void usb_task(void const * argument);

extern void ModifyDebugDataPackage(uint8_t index, float data, const char * name);

#endif /* __USB_TASK_H */