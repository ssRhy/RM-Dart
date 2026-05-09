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

#include "dart_trans.h"
#include "user_lib.h"

#if (CHASSIS_TYPE == DART_CHASSIS) && (DART_BOARD_TYPE == DART_BOARD_TRANS || DART_BOARD_TYPE == DART_BOARD_MAIN)

static TransControl_s TRANS;

/*==================== Init ====================*/

/**
 * @brief          初始化 trans 电机及 PID
 * @param[in]      none
 * @retval         none
 */
void TransInit(void)
{
    MotorInit(&TRANS.motor, 3, 1, DJI_M3508, TRANS_DIRECTION, 1.0f, 0);

    const fp32 pid_angle[3] = {DART_ANGEL_PID_KP, DART_ANGEL_PID_KI, DART_ANGEL_PID_KD};
    const fp32 pid_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD};

    PID_init(&TRANS.angle_pid, PID_POSITION, pid_angle, DART_ANGEL_PID_MAX_OUT, DART_ANGEL_PID_MAX_IOUT);
    PID_init(&TRANS.speed_pid, PID_POSITION, pid_speed, DART_PID_MAX_OUT, DART_PID_MAX_IOUT);

    TRANS.last_ecd = 0;
    TRANS.ecd_count = 0;
    TransSetAngle(50.0f * PI);
}

/*==================== Control ====================*/

/**
 * @brief          设置目标角度（相对移动）
 * @param[in]      angle: 相对当前角度的增量 (rad)，正值为正转，负值为反转
 * @retval         none
 */
void TransSetAngle(fp32 angle)
{
    TRANS.mode = TRANS_ANGLE;
    TRANS.target_angle = TRANS.fdb_angle + angle;  // 相对移动：在当前角度基础上增加增量
}

/**
 * @brief          停止 trans 电机
 * @param[in]      none
 * @retval         none
 */
void TransStop(void)
{
    TRANS.mode = TRANS_STOP;
}

/**
 * @brief          获取 trans 电机当前位置
 * @param[in]      none
 * @retval         当前角度 (rad)
 */
fp32 TransGetAngle(void)
{
    return TRANS.fdb_angle;
}

/*==================== Process ====================*/

/**
 * @brief          更新 trans 电机反馈值
 * @param[in]      none
 * @retval         none
 */
void TransObserver(void)
{
    GetMotorMeasure(&TRANS.motor);
    TRANS.fdb_speed = TRANS.motor.fdb.vel * TRANS.motor.direction;

    if (TRANS.motor.fdb.ecd - TRANS.last_ecd > HALF_ECD_RANGE)
        TRANS.ecd_count--;
    else if (TRANS.motor.fdb.ecd - TRANS.last_ecd < -HALF_ECD_RANGE)
        TRANS.ecd_count++;

    TRANS.fdb_angle = (TRANS.ecd_count * ECD_RANGE + TRANS.motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE * TRANS.motor.direction;
    TRANS.last_ecd = TRANS.motor.fdb.ecd;
}

/**
 * @brief          trans 电机控制计算
 * @param[in]      none
 * @retval         none
 */
void TransConsole(void)
{
    if (TRANS.mode == TRANS_STOP)
    {
        TRANS.ref_speed = STOP_SPEED;
        TRANS.motor.set.curr = PID_calc(&TRANS.speed_pid, TRANS.fdb_speed, TRANS.ref_speed);
    }
    else if (TRANS.mode == TRANS_ANGLE)
    {
        fp32 delta = TRANS.target_angle - TRANS.fdb_angle;
        if (delta > TRANS_DELTA_MAX)
            delta = TRANS_DELTA_MAX;
        else if (delta < -TRANS_DELTA_MAX)
            delta = -TRANS_DELTA_MAX;

        TRANS.ref_speed = PID_calc(&TRANS.angle_pid, 0, delta);
        TRANS.motor.set.curr = PID_calc(&TRANS.speed_pid, TRANS.fdb_speed, TRANS.ref_speed);
    }

    TRANS.motor.set.curr *= TRANS.motor.direction;
}

/**
 * @brief          发送 trans 电机控制指令
 * @param[in]      none
 * @retval         none
 */
void TransSendCmd(void)
{
    CanCmdDjiMotor(DART_CAN, DART_TRANS_STD_ID, 0, 0, TRANS.motor.set.curr, 0);
}

#endif  // CHASSIS_TYPE == DART_CHASSIS && (DART_BOARD_TYPE == DART_BOARD_TRANS || DART_BOARD_TYPE == DART_BOARD_MAIN)
