#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"

extern void usb_task(void const * argument);

extern void ModifyDebugDataPackage(uint8_t index, float data, const char * name);

#endif /* USB_TASK_H */
