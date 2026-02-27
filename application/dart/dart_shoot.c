/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_shoot.c/h
  * @brief      飞镖射击机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  *             射击机构由两个摩擦轮电机组成，采用速度闭环控制
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-1-24        HY           1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "dart_shoot.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "robot_param.h"

static Shoot_s shoot = {
    .shoot_flag = 0,
    .last_time  = 0,
};



/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void DartShootInit(void)
{
    // 左摩擦轮：CAN1，电机ID 3，DJI M3508，正方向，减速比 1:1
    MotorInit(&shoot.shoot_motor_L, SHOOT_MOTOR_L_ID, SHOOT_CAN, DJI_M3508, 1, 1.0f, 0);
    // 右摩擦轮：CAN1，电机ID 4，DJI M3508，反方向（对转），减速比 1:1
    MotorInit(&shoot.shoot_motor_R, SHOOT_MOTOR_R_ID, SHOOT_CAN, DJI_M3508, -1, 1.0f, 0);

    const fp32 pid_speed_l[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};
    const fp32 pid_speed_r[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD};

    PID_init(&shoot.shoot_speed_pid_L, PID_POSITION, pid_speed_l, SHOOT_PID_MAX_OUT, SHOOT_PID_MAX_IOUT);
    PID_init(&shoot.shoot_speed_pid_R, PID_POSITION, pid_speed_r, SHOOT_PID_MAX_OUT, SHOOT_PID_MAX_IOUT);
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void DartShootSetMode(void)
{
    shoot.time = osKernelSysTick();

    if (shoot.shoot_flag == 0)
    {
        // 无发射指令时保持待机转速
        shoot.mode = SHOOT_READY;
    }
    else
    {
        // 收到发射指令，切换为发射转速
        shoot.mode = SHOOT_LAUNCH;

        // 发射持续时间结束后自动复位标志
        if ((shoot.time - shoot.last_time) >= SHOOT_LAUNCH_TIME)
        {
            shoot.shoot_flag = 0;
            shoot.last_time  = shoot.time;
        }
    }
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void DartShootObserver(void)
{
    GetMotorMeasure(&shoot.shoot_motor_L);
    GetMotorMeasure(&shoot.shoot_motor_R);

    shoot.FDB.shoot_speed_fdb_L   = shoot.shoot_motor_L.fdb.vel;
    shoot.FDB.shoot_speed_fdb_R = shoot.shoot_motor_R.fdb.vel;

}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartShootReference(void)
{
    if (shoot.mode == SHOOT_STOP)
    {
        shoot.REF.shoot_speed_ref_L = STOP_SPEED;
        shoot.REF.shoot_speed_ref_R = STOP_SPEED;
    }
    else if (shoot.mode == SHOOT_READY)
    {
        // 待机转速：摩擦轮低速预热，左右对称反转
        shoot.REF.shoot_speed_ref_L =  SHOOT_READY_SPEED;
        shoot.REF.shoot_speed_ref_R = -SHOOT_READY_SPEED;
    }
    else if (shoot.mode == SHOOT_LAUNCH)
    {
        // 发射转速：摩擦轮全速，左右对称反转
        shoot.REF.shoot_speed_ref_L =  SHOOT_LAUNCH_SPEED;
        shoot.REF.shoot_speed_ref_R = -SHOOT_LAUNCH_SPEED;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootConsole(void)
{
    shoot.shoot_motor_L.set.curr = PID_calc(&shoot.shoot_speed_pid_L, shoot.FDB.shoot_speed_fdb_L, shoot.REF.shoot_speed_ref_L);
    shoot.shoot_motor_R.set.curr = PID_calc(&shoot.shoot_speed_pid_R, shoot.FDB.shoot_speed_fdb_R, shoot.REF.shoot_speed_ref_R);
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootSendCmd(void)
{
    // CanCmdDjiMotor(SHOOT_CAN, SHOOT_STD_ID,shoot.shoot_motor_L.set.curr,shoot.shoot_motor_R.set.curr,0, 0);
    CanCmdDjiMotor(SHOOT_CAN, SHOOT_STD_ID,50,0,0, 0);
    // ModifyDebugDataPackage(5, shoot.REF.shoot_speed_ref_L, "shoot_l_ref");
    // ModifyDebugDataPackage(6, shoot.FDB.shoot_speed_fdb_L, "shoot_l_fdb");
    // ModifyDebugDataPackage(7, shoot.REF.shoot_speed_ref_R, "shoot_r_ref");
    // ModifyDebugDataPackage(8, shoot.FDB.shoot_speed_fdb_R, "shoot_r_fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
