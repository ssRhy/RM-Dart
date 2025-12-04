#include "robot_param.h"

#if (DART_TYPE == DART_CHASSIS)
#ifndef DART_CHASSIS_H
#define DART_CHASSIS_H

#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_communication.h"
#include "math.h"
#include "usb_debug.h"
#include "supervisory_computer_cmd.h"
#include "user_lib.h"
#include "arm_math.h"
#include "referee.h"
#include "detect_task.h"
#include "CAN_cmd_dji.h"
#include "CAN_receive.h"

typedef struct
{
    Motor_s motor[1];  
    uint32_t timer;    
    
    // PID控制器
    pid_type_def pid[1];
    
    // 目标速度
    fp32 speed_ref;
} Dart_s;


extern void DartInit(void);
extern void DartObserver(void);
extern void DartSetMode(void);
extern void DartReference(void);
extern void DartConsole(void);
extern void DartSendCmd(void);

#endif /* DART_CHASSIS */
#endif /* DART_CHASSIS_H */

