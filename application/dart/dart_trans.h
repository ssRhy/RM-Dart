#include "robot_param.h"          //     选择飞镖发射类型
#if (DART_TRANS_TYPE == DART_TRANS)
#ifndef DART_TRANS_H
#define DART_TRANS_H

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
    Motor_s trans_motor;  
    uint32_t timer;    
    
    // PID控制器
    pid_type_def pid;
    
    // 目标速度
    fp32 speed_ref;
} Dart_trans;


#endif /* DART_TRANS_H */
#endif /* DART_TRANS_H */
