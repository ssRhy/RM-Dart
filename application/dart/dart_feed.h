#include "robot_param.h"          //     选择飞镖发射类型
#if (DART_FEED_TYPE == DART_FEED)
#ifndef DART_FEED_H
#define DART_FEED_H

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
    Motor_s feed_motor;  
    uint32_t timer;    
    
    // PID控制器
    pid_type_def pid;
    
    // 目标速度
    fp32 speed_ref;
} Dart_feed;


#endif /* DART_FEED_H */
#endif /* DART_FEED_H */
