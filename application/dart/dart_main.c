/**
 ****************************(C) COPYRIGHT 2024 Polarbear****************************
 * @file       dart_main.c/h
 * @brief      飞镖主控板机构控制器（chassis + feed + trans 合并管理）
 * @note       时序约束（参照 shoot_fric_trigger 的 state/mode 双层结构）：
 *               第1层：chassis—— 底盘到位是后续动作的前提
 *               第2层：feed—— chassis 到位后执行供弹
 *               第3层：trans—— feed 到位后执行横移
 *             CAN 帧分配：
 *               chassis → CAN1 + 0x1FF（ID4，M6020，独立帧）
 *               feed    → CAN1 + 0x200（ID2，M2006，与 trans 合帧）
 *               trans   → CAN1 + 0x200（ID1，M3508，与 feed 合帧）
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
#include "robot_param.h"

#if (CHASSIS_TYPE == DART_CHASSIS) && (DART_BOARD_TYPE == DART_BOARD_MAIN)


static DartMain_s DART = {
    .chassis_move_flag = 0,
    .chassis_done_flag = 0,
    .chassis_last_time = 0,
    .chassis_mode      = CHASSIS_ANGEL,
    .feed_move_flag    = 0,
    .feed_done_flag    = 0,
    .feed_last_time    = 0,
    .feed_mode         = FEED_STOP,
    .trans_move_flag   = 0,
    .trans_done_flag   = 0,
    .trans_last_time   = 0,
    .trans_mode        = TRANS_STOP,
};

static fp32 chassis_delta;
static fp32 feed_delta;
static fp32 trans_delta;

#define FEED_DELTA_MAX  2.0f

/*-------------------- Init --------------------*/

/**
 * @brief          初始化 chassis、feed、trans 电机及 PID
 * @param[in]      none
 * @retval         none
 */
void DartMainInit(void)
{
    /* ----- Chassis（ID1，M6020） ----- */
    MotorInit(&DART.chassis_motor, 1, 1, DJI_M6020, CHASSIS_DIRECTION, 1.0f, 0);
    const fp32 pid_chassis_angle[3] = {CHASSIS_ANGEL_PID_KP, CHASSIS_ANGEL_PID_KI, CHASSIS_ANGEL_PID_KD};
    const fp32 pid_chassis_speed[3] = {CHASSIS_SPEED_PID_KP, CHASSIS_SPEED_PID_KI, CHASSIS_SPEED_PID_KD};
    PID_init(&DART.chassis_angle_pid, PID_POSITION, pid_chassis_angle, CHASSIS_ANGEL_PID_MAX_OUT, CHASSIS_ANGEL_PID_MAX_IOUT);
    PID_init(&DART.chassis_speed_pid, PID_POSITION, pid_chassis_speed, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);
    DART.chassis_last_time = osKernelSysTick() - CHANGE_TIME - 1;


    /* ----- Feed（ID2，M2006） ----- */
    MotorInit(&DART.feed_motor, 2, 1, DJI_M2006, FEED_DIRECTION, 1.0f, 0);
    const fp32 pid_feed_angle[3] = {FEED_ANGEL_PID_KP, FEED_ANGEL_PID_KI, FEED_ANGEL_PID_KD};
    const fp32 pid_feed_speed[3] = {FEED_SPEED_PID_KP, FEED_SPEED_PID_KI, FEED_SPEED_PID_KD};
    PID_init(&DART.feed_angle_pid, PID_POSITION, pid_feed_angle, FEED_ANGEL_PID_MAX_OUT, FEED_ANGEL_PID_MAX_IOUT);
    PID_init(&DART.feed_speed_pid, PID_POSITION, pid_feed_speed, FEED_PID_MAX_OUT, FEED_PID_MAX_IOUT);
    DART.feed_last_time = osKernelSysTick() - CHANGE_TIME - 1;



    /* ----- Trans（ID3，M3508） ----- */
    MotorInit(&DART.trans_motor, 3, 1, DJI_M3508, TRANS_DIRECTION, 1.0f, 0);
    const fp32 pid_trans_angle[3] = {DART_ANGEL_PID_KP, DART_ANGEL_PID_KI, DART_ANGEL_PID_KD};
    const fp32 pid_trans_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD};
    PID_init(&DART.trans_angle_pid, PID_POSITION, pid_trans_angle, DART_ANGEL_PID_MAX_OUT, DART_ANGEL_PID_MAX_IOUT);
    PID_init(&DART.trans_speed_pid, PID_POSITION, pid_trans_speed, DART_PID_MAX_OUT, DART_PID_MAX_IOUT);
    DART.trans_last_time = osKernelSysTick() - CHANGE_TIME - 1;
}



/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @note           层级关系
 *                   chassis 未到位 → feed 强制停止，trans 强制停止
 *                   chassis 到位但 feed 未到位 → trans 强制停止
 *                   feed 到位后 → trans 正常运行
 * @param[in]      none
 * @retval         none
 */

void DartMainSetMode(void)
{
    static bool last_dart_on = false;
    bool dart_on = GetScCmdDartOn();

    if (dart_on) {
        DART.chassis_mode = CHASSIS_STOP;
        DART.feed_mode    = FEED_STOP;
        DART.trans_mode   = TRANS_STOP;
        last_dart_on = false;
        return;
    }

    if (!last_dart_on && !dart_on) {
        DART.chassis_move_flag     = 1;
        DART.chassis_done_flag     = 0;
        DART.chassis_ref.angle_ref = 0.0f;
        DART.feed_move_flag        = 0;
        DART.feed_done_flag        = 0;
        DART.feed_step             = 0;
        DART.trans_move_flag       = 0;
        DART.trans_done_flag       = 0;
    }
    last_dart_on = !dart_on;//真正上位机的时候这个给逻辑要改
    

    /* ===== 第1层：Chassis 模式（始终运行，无前置条件） ===== */
    if (DART.chassis_done_flag)
    {
        DART.chassis_mode = CHASSIS_STOP;
    }
    else if (DART.chassis_move_flag == 1)
    {
        /* 首次进入：设置目标角度（基于当前反馈 + 增量） */
        if (DART.chassis_ref.angle_ref == 0.0f)
        {
            DART.chassis_ref.angle_ref = theta_format(DART.chassis_fdb.angle_fdb + PI / 3);
        }
        DART.chassis_mode = CHASSIS_ANGEL;
    }
    else
    {
        DART.chassis_done_flag = 1;
        DART.chassis_mode      = CHASSIS_STOP;
    }

    /* ===== 第2层：Feed 模式（chassis 到位才允许） ===== */
    if (DART.chassis_move_flag != 0)
    {
        DART.feed_mode  = FEED_STOP;
        DART.trans_mode = TRANS_STOP;
        return;
    }

    if (DART.feed_done_flag)
    {
        DART.feed_mode = FEED_STOP;
    }
    else if (DART.feed_step == 2 || DART.feed_step == 4)
    {
        DART.feed_mode = FEED_STOP;
    }
    else if (DART.feed_move_flag == 1)
    {
        DART.feed_mode = FEED_ANGEL;
    }
    else if (!DART.chassis_done_flag)
    {
        DART.feed_mode = FEED_STOP;
    }
    else
    {
        DART.feed_ref.angle_ref = DART.feed_fdb.angle_fdb + 11 * PI/2;
        DART.feed_step          = 1;
        DART.feed_move_flag     = 1;
        DART.feed_mode          = FEED_ANGEL;
    }

    /* ===== 第3层：Trans 模式（feed 到位才允许） ===== */
    if (DART.feed_move_flag != 0)
    {
        DART.trans_mode = TRANS_STOP;
        return;
    }

    if (DART.trans_done_flag)
    {
        DART.trans_mode = TRANS_STOP;
    }
    else if (DART.trans_move_flag == 1)
    {
        DART.trans_mode = TRANS_ANGEL;
    }
    else if (!DART.feed_done_flag)
    {
        DART.trans_mode = TRANS_STOP;
    }
    else
    {
        DART.trans_ref.angle_ref = DART.trans_fdb.angle_fdb + 20 * PI ;
        DART.trans_move_flag     = 1;
        DART.trans_mode          = TRANS_ANGEL;
    }
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新 chassis、feed、trans 状态量
 * @param[in]      none
 * @retval         none
 */
void DartMainObserver(void)
{
    /* ----- Chassis ----- */
    GetMotorMeasure(&DART.chassis_motor);
    DART.chassis_fdb.speed_fdb = DART.chassis_motor.fdb.vel * DART.chassis_motor.direction;

    if (DART.chassis_motor.fdb.ecd - DART.chassis_last_ecd > HALF_ECD_RANGE)
        DART.chassis_ecd_count--;
    else if (DART.chassis_motor.fdb.ecd - DART.chassis_last_ecd < -HALF_ECD_RANGE)
        DART.chassis_ecd_count++;

    if (DART.chassis_ecd_count == FULL_COUNT)
        DART.chassis_ecd_count = -(FULL_COUNT - 1);
    else if (DART.chassis_ecd_count == -FULL_COUNT)
        DART.chassis_ecd_count = FULL_COUNT - 1;

    DART.chassis_fdb.angle_fdb = (DART.chassis_ecd_count * ECD_RANGE + DART.chassis_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE * DART.chassis_motor.direction;
    DART.chassis_last_ecd      = DART.chassis_motor.fdb.ecd;
    DART.chassis_time          = osKernelSysTick();

    /* ----- Feed ----- */
    GetMotorMeasure(&DART.feed_motor);
    DART.feed_fdb.speed_fdb = DART.feed_motor.fdb.vel * DART.feed_motor.direction;

    if (DART.feed_motor.fdb.ecd - DART.feed_last_ecd > HALF_ECD_RANGE)
        DART.feed_ecd_count--;
    else if (DART.feed_motor.fdb.ecd - DART.feed_last_ecd < -HALF_ECD_RANGE)
        DART.feed_ecd_count++;

    DART.feed_fdb.angle_fdb = (DART.feed_ecd_count * ECD_RANGE + DART.feed_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE * DART.feed_motor.direction;
    DART.feed_last_ecd      = DART.feed_motor.fdb.ecd;
    DART.feed_time          = osKernelSysTick();

    /* ----- Trans ----- */
    GetMotorMeasure(&DART.trans_motor);
    DART.trans_fdb.speed_fdb = DART.trans_motor.fdb.vel * DART.trans_motor.direction;

    if (DART.trans_motor.fdb.ecd - DART.trans_last_ecd > HALF_ECD_RANGE)
        DART.trans_ecd_count--;
    else if (DART.trans_motor.fdb.ecd - DART.trans_last_ecd < -HALF_ECD_RANGE)
        DART.trans_ecd_count++;

    DART.trans_fdb.angle_fdb = (DART.trans_ecd_count * ECD_RANGE + DART.trans_motor.fdb.ecd) * MOTOR_ECD_TO_ANGLE * DART.trans_motor.direction;
    DART.trans_last_ecd      = DART.trans_motor.fdb.ecd;
    DART.trans_time          = osKernelSysTick();
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @note           第1层 chassis 始终计算目标；
 *                 第2层 feed 仅在 chassis 到位后计算；
 *                 第3层 trans 仅在 feed 到位后计算。
 * @param[in]      none
 * @retval         none
 */
void DartMainReference(void)
{
    /* ===== 第1层：Chassis 目标量 ===== */
    switch (DART.chassis_mode)
    {
    case CHASSIS_STOP:
        DART.chassis_ref.speed_ref = STOP_SPEED;
        break;

    case CHASSIS_ANGEL:
        if (fabsf(theta_format(DART.chassis_ref.angle_ref - DART.chassis_fdb.angle_fdb)) > ARRIVE_THRESHOLD)
            DART.chassis_move_flag = 1;
        else
            DART.chassis_move_flag = 0;
        break;

    default:
        break;
    }

    /* ===== 第2层：Feed 目标量（chassis 到位后才更新） ===== */
    switch (DART.feed_mode)
    {
    case FEED_STOP:
        DART.feed_ref.speed_ref = STOP_SPEED;
        if (DART.feed_step == 2 &&
            osKernelSysTick() - DART.feed_delay_start >= 3000)
        {
            DART.feed_ref.angle_ref = DART.feed_fdb.angle_fdb + 9 * PI/2;
            DART.feed_step          = 3;
            DART.feed_move_flag     = 1;
            DART.feed_mode          = FEED_ANGEL;
        }
        else if (DART.feed_step == 4 &&
                 osKernelSysTick() - DART.feed_delay_start >= 3000)
        {
            DART.feed_ref.angle_ref = DART.feed_fdb.angle_fdb - 10 * PI;
            DART.feed_step          = 5;
            DART.feed_move_flag     = 1;
            DART.feed_mode          = FEED_ANGEL;
        }
        break;

    case FEED_ANGEL:
        if (fabsf(DART.feed_ref.angle_ref - DART.feed_fdb.angle_fdb) > ARRIVE_THRESHOLD)
        {
            DART.feed_move_flag = 1;
        }
        else
        {
            if (DART.feed_step == 1)
            {
                DART.feed_step        = 2;
                DART.feed_delay_start = osKernelSysTick();
                DART.feed_mode        = FEED_STOP;
                DART.feed_move_flag   = 1;
            }
            else if (DART.feed_step == 3)
            {
                DART.feed_step        = 4;
                DART.feed_delay_start = osKernelSysTick();
                DART.feed_mode        = FEED_STOP;
                DART.feed_move_flag   = 1;
            }
            else
            {
                DART.feed_move_flag  = 0;
                DART.feed_done_flag  = 1;
            }
        }
        break;

    default:
        break;
    }

    /* ===== 第3层：Trans 目标量（feed 到位后才更新） ===== */
    switch (DART.trans_mode)
    {
    case TRANS_STOP:
        DART.trans_ref.speed_ref = STOP_SPEED;
        break;

    case TRANS_ANGEL:
        if (fabsf(DART.trans_ref.angle_ref - DART.trans_fdb.angle_fdb) > ARRIVE_THRESHOLD)
        {
            DART.trans_move_flag = 1;
        }
        else
        {
            DART.trans_move_flag = 0;
            DART.trans_done_flag = 1;
        }
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
    /* ----- Chassis ----- */
    if (DART.chassis_mode == CHASSIS_STOP)
    {
        DART.chassis_motor.set.curr = PID_calc(&DART.chassis_speed_pid,DART.chassis_fdb.speed_fdb,DART.chassis_ref.speed_ref);
    }
    else if (DART.chassis_mode == CHASSIS_ANGEL)
    {
        chassis_delta               = theta_format(DART.chassis_ref.angle_ref - DART.chassis_fdb.angle_fdb);
        DART.chassis_ref.speed_ref  = PID_calc(&DART.chassis_angle_pid, 0, chassis_delta);
        DART.chassis_motor.set.curr = PID_calc(&DART.chassis_speed_pid,DART.chassis_fdb.speed_fdb,DART.chassis_ref.speed_ref);
    }
    DART.chassis_motor.set.curr *= DART.chassis_motor.direction;

    /* ----- Feed ----- */
    if (DART.feed_mode == FEED_STOP)
    {
        DART.feed_motor.set.curr = PID_calc(&DART.feed_speed_pid,DART.feed_fdb.speed_fdb,DART.feed_ref.speed_ref);
    }
    else if (DART.feed_mode == FEED_ANGEL)
    {
        feed_delta               = DART.feed_ref.angle_ref - DART.feed_fdb.angle_fdb;
        if (feed_delta > FEED_DELTA_MAX)
            feed_delta = FEED_DELTA_MAX;
        else if (feed_delta < -FEED_DELTA_MAX)
            feed_delta = -FEED_DELTA_MAX;
        DART.feed_ref.speed_ref  = PID_calc(&DART.feed_angle_pid, 0, feed_delta);
        DART.feed_motor.set.curr = PID_calc(&DART.feed_speed_pid,DART.feed_fdb.speed_fdb,DART.feed_ref.speed_ref);
    }
    DART.feed_motor.set.curr *= DART.feed_motor.direction;

    /* ----- Trans ----- */
    if (DART.trans_mode == TRANS_STOP)
    {
        DART.trans_motor.set.curr = PID_calc(&DART.trans_speed_pid,DART.trans_fdb.speed_fdb,DART.trans_ref.speed_ref);
    }
    else if (DART.trans_mode == TRANS_ANGEL)
    {
        trans_delta               = DART.trans_ref.angle_ref - DART.trans_fdb.angle_fdb;
        if (trans_delta > FEED_DELTA_MAX)
            trans_delta = FEED_DELTA_MAX;
        else if (trans_delta < -FEED_DELTA_MAX)
            trans_delta = -FEED_DELTA_MAX;
        DART.trans_ref.speed_ref  = PID_calc(&DART.trans_angle_pid, 0, trans_delta);
        DART.trans_motor.set.curr = PID_calc(&DART.trans_speed_pid,DART.trans_fdb.speed_fdb,DART.trans_ref.speed_ref);
    }
    DART.trans_motor.set.curr *= DART.trans_motor.direction;
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @note           CAN 帧分配：
 *                   chassis(ID4) → CAN1 + 0x1FF（独立帧，M6020）
 *                   trans(ID1) + feed(ID2) → CAN1 + 0x200（合并帧，同一主控板）
 * @param[in]      none
 * @retval         none
 */
void DartMainSendCmd(void)
{

    CanCmdDjiMotor(CHASSIS_CAN,CHASSIS_STD_ID,DART.chassis_motor.set.curr,0,0,0); 
    CanCmdDjiMotor(DART_CAN,DART_TRANS_STD_ID,0,DART.feed_motor.set.curr, DART.trans_motor.set.curr, 0);

    ModifyDebugDataPackage(1, DART.chassis_ref.angle_ref, "chas_ref");
    ModifyDebugDataPackage(2, DART.chassis_fdb.angle_fdb, "chas_fdb");
    ModifyDebugDataPackage(3, DART.feed_ref.angle_ref,    "feed_ref");
    ModifyDebugDataPackage(4, DART.feed_fdb.angle_fdb,    "feed_fdb");
    ModifyDebugDataPackage(5, DART.trans_ref.angle_ref,   "trans_ref");
    ModifyDebugDataPackage(6, DART.trans_fdb.angle_fdb,   "trans_fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS && DART_BOARD_TYPE == DART_BOARD_MAIN
