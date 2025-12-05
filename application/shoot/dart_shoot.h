#include "robot_param.h"   
#if (SHOOT_TYPE == SHOOT_DART_FRIC)
#ifndef DART_SHOOT_H
#define DART_SHOOT_H

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


typedef struct feedback
{
    fp32 shoot_speed_fdb[6]; // 飞镖发射速度反馈
} Dart_fdb;

typedef struct reference
{
    fp32 shoot_speed_ref[6]; // 飞镖发射速度参考
} Dart_ref;

typedef enum{
    DART_SHOOT_NOT_READY,
    DART_SHOOT_READY,

   
}DartState_e;

typedef enum
{
    DART_SHOOT_STOP,
    DART_SHOOT_FIRE,
    DART_LOAD,
} DartMode_e;

typedef struct
{
    Motor_s shoot_motor[6];  
    uint32_t timer;    
    // PID控制器
    pid_type_def pid[6];

    Dart_fdb FDB;//反馈值feedback

    Dart_ref REF;//参考值reference

    DartState_e state;
    
    DartMode_e mode;
} Dart_up;


extern void ShootInit(void);
extern void ShootObserver(void);
extern void hootSetMode(void);
extern void ShootReference(void);
extern void ShootConsole(void);
extern void ShootSendCmd(void);

#endif /* DART_SHOOT */
#endif /* DART_SHOOT_H */