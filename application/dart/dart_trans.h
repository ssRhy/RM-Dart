/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       dart_trans.c/h
 * @brief      trans 电机独立控制（角度模式）
 * @note       仅驱动 trans 电机，不包含其他判断逻辑
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-3-15      HY            1. 从 dart_main 拆分出 trans 独立控制
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
******************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "robot_param.h"
#include "user_lib.h"

#if (CHASSIS_TYPE == DART_CHASSIS) && (DART_BOARD_TYPE == DART_BOARD_TRANS || DART_BOARD_TYPE == DART_BOARD_MAIN)
#ifndef DART_TRANS_H
#define DART_TRANS_H

#include "motor.h"
#include "pid.h"
#include "CAN_communication.h"
#include "math.h"
#include "arm_math.h"

/* ==================== Trans 模式枚举 ==================== */
typedef enum {
    TRANS_ANGLE = 0,
    TRANS_STOP,
} TransMode_e;

/* ==================== Trans 控制结构体 ==================== */
typedef struct {
    Motor_s      motor;
    fp32         target_angle;
    fp32         ref_speed;
    fp32         fdb_speed;
    fp32         fdb_angle;
    TransMode_e  mode;
    pid_type_def speed_pid;
    pid_type_def angle_pid;
    int16_t      last_ecd;
    int16_t      ecd_count;
} TransControl_s;

/* ==================== 对外接口 ==================== */
extern void TransInit(void);
extern void TransSetAngle(fp32 angle);
extern void TransStop(void);
extern fp32 TransGetAngle(void);
extern void TransObserver(void);
extern void TransConsole(void);
extern void TransSendCmd(void);

#endif  // DART_TRANS_H
#endif  // CHASSIS_TYPE == DART_CHASSIS && (DART_BOARD_TYPE == DART_BOARD_TRANS || DART_BOARD_TYPE == DART_BOARD_MAIN)
