/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       dart_main.c/h
 * @brief      飞镖主控板机构控制器（trans + feed 合并管理）
 * @note       参照 shoot_fric_trigger 模块风格，将 trans 和 feed 统一在一个结构体中，
 *             实现先 trans 到位、再 feed 动作的时序控制，并合并 CAN 帧发送。
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-2-27       HY            1. 由 dart_trans + dart_feed 重构合并
 *
 @verbatim
 ==============================================================================

 ==============================================================================
 @endverbatim
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "dart_main.h"

#if (CHASSIS_TYPE == DART_CHASSIS) && (DART_BOARD_TYPE == DART_BOARD_MAIN)

#include "robot_param.h"

static DartMain_s DART = {
    .trans_move_flag = 0,
    .trans_last_time = 0,
    .trans_mode      = TRANS_ANGEL,
    .feed_move_flag  = 0,
    .feed_last_time  = 0,
    .feed_mode       = FEED_STOP,
};

static fp32 trans_delta;
static fp32 feed_delta;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化 trans 和 feed 电机及 PID
 * @param[in]      none
 * @retval         none
 */
void DartMainInit(void)
{
    /* ----- Trans ----- */
    MotorInit(&DART.trans_motor, 1, 1, DJI_M3508, 1, 1.0f, 0);

    const fp32 pid_trans_angle[3] = {DART_ANGEL_PID_KP, DART_ANGEL_PID_KI, DART_ANGEL_PID_KD};
    const fp32 pid_trans_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD};

    PID_init(&DART.trans_angle_pid, PID_POSITION, pid_trans_angle, DART_ANGEL_PID_MAX_OUT, DART_ANGEL_PID_MAX_IOUT);
    PID_init(&DART.trans_speed_pid, PID_POSITION, pid_trans_speed, DART_PID_MAX_OUT, DART_PID_MAX_IOUT);

    DART.trans_last_time = osKernelSysTick() - CHANGE_TIME - 1;

    /* ----- Feed ----- */
    MotorInit(&DART.feed_motor, 2, 1, DJI_M2006, -1, 1.0f, 0);

    const fp32 pid_feed_angle[3] = {FEED_ANGEL_PID_KP, FEED_ANGEL_PID_KI, FEED_ANGEL_PID_KD};
    const fp32 pid_feed_speed[3] = {FEED_SPEED_PID_KP, FEED_SPEED_PID_KI, FEED_SPEED_PID_KD};

    PID_init(&DART.feed_angle_pid, PID_POSITION, pid_feed_angle, FEED_ANGEL_PID_MAX_OUT, FEED_ANGEL_PID_MAX_IOUT);
    PID_init(&DART.feed_speed_pid, PID_POSITION, pid_feed_speed, FEED_PID_MAX_OUT, FEED_PID_MAX_IOUT);

    DART.feed_last_time = osKernelSysTick() - CHANGE_TIME - 1;
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @note           时序约束：trans 未到位时强制 feed 停止，参照 shoot 模块摩擦轮检测逻辑
 * @param[in]      none
 * @retval         none
 */
void DartMainSetMode(void)
{
    /* ----- Trans 模式 ----- */
    if (DART.trans_move_flag == 0 && (DART.trans_time - DART.trans_last_time) >= CHANGE_TIME)
    {
        DART.trans_mode      = TRANS_ANGEL;
        DART.trans_last_time = DART.trans_time;
    }
    else if (DART.trans_move_flag == 1)
    {
        DART.trans_mode = TRANS_ANGEL;
    }
    else
    {
        DART.trans_mode = TRANS_STOP;
    }

    /* ----- Feed 模式（时序约束：trans 到位才允许 feed 动作） ----- */
    if (DART.trans_move_flag != 0)
    {
        /* trans 尚未到位，feed 强制停止 */
        DART.feed_mode = FEED_STOP;
        return;
    }

    if (DART.feed_move_flag == 0 && (DART.feed_time - DART.feed_last_time) >= CHANGE_TIME)
    {
        DART.feed_mode      = FEED_ANGEL;
        DART.feed_last_time = DART.feed_time;
    }
    else if (DART.feed_move_flag == 1)
    {
        DART.feed_mode = FEED_ANGEL;
    }
    else
    {
        DART.feed_mode = FEED_STOP;
    }
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新 trans 和 feed 状态量
 * @param[in]      none
 * @retval         none
 */
void DartMainObserver(void)
{
    /* ----- Trans ----- */
    GetMotorMeasure(&DART.trans_motor);
    DART.trans_fdb.speed_fdb = DART.trans_motor.fdb.vel;

    if (DART.trans_motor.fdb.ecd - DART.trans_last_ecd > HALF_ECD_RANGE)
        DART.trans_ecd_count--;
    else if (DART.trans_motor.fdb.ecd - DART.trans_last_ecd < -HALF_ECD_RANGE)
        DART.trans_ecd_count++;

    if (DART.trans_ecd_count == FULL_COUNT)
        DART.trans_ecd_count = -(FULL_COUNT - 1);
    else if (DART.trans_ecd_count == -FULL_COUNT)
        DART.trans_ecd_count = FULL_COUNT - 1;

    DART.trans_fdb.angle_fdb = (DART.trans_ecd_count * ECD_RANGE + DART.trans_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE;
    DART.trans_last_ecd      = DART.trans_motor.fdb.ecd;
    DART.trans_time          = osKernelSysTick();

    /* ----- Feed ----- */
    GetMotorMeasure(&DART.feed_motor);
    DART.feed_fdb.speed_fdb = DART.feed_motor.fdb.vel;

    if (DART.feed_motor.fdb.ecd - DART.feed_last_ecd > HALF_ECD_RANGE)
        DART.feed_ecd_count--;
    else if (DART.feed_motor.fdb.ecd - DART.feed_last_ecd < -HALF_ECD_RANGE)
        DART.feed_ecd_count++;

    if (DART.feed_ecd_count == FULL_COUNT)
        DART.feed_ecd_count = -(FULL_COUNT - 1);
    else if (DART.feed_ecd_count == -FULL_COUNT)
        DART.feed_ecd_count = FULL_COUNT - 1;

    DART.feed_fdb.angle_fdb = (DART.feed_ecd_count * ECD_RANGE + DART.feed_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE;
    DART.feed_last_ecd      = DART.feed_motor.fdb.ecd;
    DART.feed_time          = osKernelSysTick();
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @note           外层 switch：trans 目标；内层 switch：feed 目标（需 trans 到位）
 *                 与 ShootReference 的 state/mode 双层结构完全对应
 * @param[in]      none
 * @retval         none
 */
void DartMainReference(void)
{
    /* ===== 外层：Trans 目标量（类比摩擦轮） ===== */
    switch (DART.trans_mode)
    {
    case TRANS_STOP:
        DART.trans_ref.speed_ref = STOP_SPEED;
        break;

    case TRANS_ANGEL:
        if (DART.trans_move_flag == 0)
        {
            DART.trans_ref.angle_ref = theta_format(DART.trans_fdb.angle_fdb + PI / 2);
        }
        if (theta_format(DART.trans_ref.angle_ref - DART.trans_fdb.angle_fdb) > 0.001f)
            DART.trans_move_flag = 1;
        else
            DART.trans_move_flag = 0;
        break;

    default:
        break;
    }

    /* ===== 内层：Feed 目标量（类比拨弹盘，trans 到位后才执行） ===== */
    switch (DART.feed_mode)
    {
    case FEED_STOP:
        DART.feed_ref.speed_ref = STOP_SPEED;
        break;

    case FEED_ANGEL:
        if (DART.feed_move_flag == 0)
        {
            DART.feed_ref.angle_ref = theta_format(DART.feed_fdb.angle_fdb + PI / 3);
        }
        if (theta_format(DART.feed_ref.angle_ref - DART.feed_fdb.angle_fdb) > 0.001f)
            DART.feed_move_flag = 1;
        else
            DART.feed_move_flag = 0;
        break;

    default:
        break;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartMainConsole(void)
{
    /* ----- Trans ----- */
    if (DART.trans_mode == TRANS_STOP)
    {
        DART.trans_motor.set.curr = PID_calc(&DART.trans_speed_pid,
                                              DART.trans_fdb.speed_fdb,
                                              DART.trans_ref.speed_ref);
    }
    else if (DART.trans_mode == TRANS_ANGEL)
    {
        trans_delta               = theta_format(DART.trans_ref.angle_ref - DART.trans_fdb.angle_fdb);
        DART.trans_ref.speed_ref  = PID_calc(&DART.trans_angle_pid, 0, trans_delta);
        DART.trans_motor.set.curr = PID_calc(&DART.trans_speed_pid,
                                              DART.trans_fdb.speed_fdb,
                                              DART.trans_ref.speed_ref);
    }

    /* ----- Feed ----- */
    if (DART.feed_mode == FEED_STOP)
    {
        DART.feed_motor.set.curr = PID_calc(&DART.feed_speed_pid,
                                             DART.feed_fdb.speed_fdb,
                                             DART.feed_ref.speed_ref);
    }
    else if (DART.feed_mode == FEED_ANGEL)
    {
        feed_delta               = theta_format(DART.feed_ref.angle_ref - DART.feed_fdb.angle_fdb);
        DART.feed_ref.speed_ref  = PID_calc(&DART.feed_angle_pid, 0, feed_delta);
        DART.feed_motor.set.curr = PID_calc(&DART.feed_speed_pid,
                                             DART.feed_fdb.speed_fdb,
                                             DART.feed_ref.speed_ref);
    }
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @note           trans(ID1) 和 feed(ID2) 同属一块主控板，共用 CAN1+0x200，
 *                 必须合并为一帧发送，避免独立发送时后帧覆盖前帧。
 * @param[in]      none
 * @retval         none
 */
void DartMainSendCmd(void)
{
    CanCmdDjiMotor(DART_CAN, DART_TRANS_STD_ID,
                   DART.trans_motor.set.curr,   /* ID1: trans */
                   DART.feed_motor.set.curr,    /* ID2: feed  */
                   0, 0);

    ModifyDebugDataPackage(1, DART.trans_ref.angle_ref, "trans_ref");
    ModifyDebugDataPackage(2, DART.trans_fdb.angle_fdb, "trans_fdb");
    ModifyDebugDataPackage(3, DART.feed_ref.angle_ref,  "feed_ref");
    ModifyDebugDataPackage(4, DART.feed_fdb.angle_fdb,  "feed_fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS && DART_BOARD_TYPE == DART_BOARD_MAIN
