#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_DART)
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
    Motor_s chassis_motor[1];  
    uint32_t timer;    
    
    // PID控制器
    pid_type_def pid[1];
    
    // 目标速度
    fp32 speed_ref;
} Dart_down;


extern void ChassisInit(void);

extern void ChassisSetMode(void);

extern void ChassisObserver(void);

extern void ChassisReference(void);

extern void ChassisConsole(void);

extern void ChassisSendCmd(void);

#endif /* DART_CHASSIS */
#endif /* DART_CHASSIS_H */

