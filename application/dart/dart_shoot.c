/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_shoot.c/h
  * @brief      电机飞镖机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-1-24        HY           1. 完成基本框架
  *  V1.1.0     2025-1-24        HY          1. 完成电机控制任务
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
  .move_flag = 0,
  .last_time = 0,
};
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void DartShootInit(void) 
{ 
    MotorInit(&shoot.shoot_motor_L,1, 1,DJI_M3508, 1, 1.0f, 0);

    const fp32 pid_speed[3] = {SHOOT_SPEED_PID_KP, SHOOT_SPEED_PID_KI, SHOOT_SPEED_PID_KD}; 

    PID_init(&shoot.motor_speed_pid, PID_POSITION, pid_speed, SHOOT_PID_MAX_OUT, SHOOT_PID_MAX_IOUT);   
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void DartShootSetMode(void)
{
    shoot.mode = SHOOT_MOTOR_STOP;
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void DartShootObserver(void) 
{
    GetMotorMeasure(&shoot.shoot_motor_L);//获取电机反馈值

    shoot.motor_fdb.motor_speed_fdb = shoot.shoot_motor_L.fdb.vel;//电机速度反馈赋值
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartShootReference(void) 
{
    shoot.motor_ref.motor_speed_ref = SHOOT_READY_SPEED;
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootConsole(void) 
{
    shoot.shoot_motor_L.set.curr = PID_calc(&shoot.motor_speed_pid, shoot.motor_fdb.motor_speed_fdb, shoot.motor_ref.motor_speed_ref);
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootSendCmd(void) 
{
    CanCmdDjiMotor(SHOOT_CAN,SHOOT_STD_ID, shoot.shoot_motor_L.set.curr, 0, 0, 0);
    //CanCmdDjiMotor(DART_CAN,DART_TRANS_STD_ID, 0, 0, 0, 0);

    ModifyDebugDataPackage(1, shoot.motor_ref.motor_speed_ref, "ref");
    ModifyDebugDataPackage(2, shoot.motor_fdb.motor_speed_fdb, "fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
