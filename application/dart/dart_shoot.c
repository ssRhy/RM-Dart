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
    MotorInit(&shoot.shoot_motor_front[0],1, 1,DJI_M3508, 1, 1.0f, 0);
    MotorInit(&shoot.shoot_motor_front[1],2, 1,DJI_M3508, 1, 1.0f, 0);
    MotorInit(&shoot.shoot_motor_mid[0],3, 1,DJI_M3508, 1, 1.0f, 0);
    MotorInit(&shoot.shoot_motor_mid[1],4, 1,DJI_M3508, 1, 1.0f, 0);
    MotorInit(&shoot.shoot_motor_rear[0],5, 1,DJI_M3508, 1, 1.0f, 0);
    MotorInit(&shoot.shoot_motor_rear[1],6, 1,DJI_M3508, 1, 1.0f, 0);
    const fp32 pid_speed_front[3] = {SHOOT_FRONT_PID_KP, SHOOT_FRONT_PID_KI, SHOOT_FRONT_PID_KD}; 
    const fp32 pid_speed_mid[3] = {SHOOT_MID_PID_KP, SHOOT_MID_PID_KI, SHOOT_MID_PID_KD}; 
    const fp32 pid_speed_rear[3] = {SHOOT_REAR_PID_KP, SHOOT_REAR_PID_KI, SHOOT_REAR_PID_KD}; 

    PID_init(&shoot.motor_speed_pid_front, PID_POSITION, pid_speed_front, SHOOT_FRONT_PID_MAX_OUT, SHOOT_FRONT_PID_MAX_IOUT); 
    PID_init(&shoot.motor_speed_pid_mid, PID_POSITION, pid_speed_mid, SHOOT_MID_PID_MAX_OUT, SHOOT_MID_PID_MAX_IOUT);   
    PID_init(&shoot.motor_speed_pid_rear, PID_POSITION, pid_speed_rear, SHOOT_REAR_PID_MAX_OUT, SHOOT_REAR_PID_MAX_IOUT);   
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
    GetMotorMeasure(&shoot.shoot_motor_front[0]);//获取电机反馈值
    GetMotorMeasure(&shoot.shoot_motor_front[1]);//获取电机反馈值

    GetMotorMeasure(&shoot.shoot_motor_mid[0]);//获取电机反馈值
    GetMotorMeasure(&shoot.shoot_motor_mid[1]);//获取电机反馈值

    GetMotorMeasure(&shoot.shoot_motor_rear[0]);//获取电机反馈值
    GetMotorMeasure(&shoot.shoot_motor_rear[1]);//获取电机反馈值

    

    shoot.motor_fdb.motor_speed_fdb_front = shoot.shoot_motor_front[0].fdb.vel;//电机速度反馈赋值
    shoot.motor_fdb.motor_speed_fdb_front = shoot.shoot_motor_front[1].fdb.vel;
    shoot.motor_fdb.motor_speed_fdb_mid = shoot.shoot_motor_mid[0].fdb.vel;//电机速度反馈赋值
    shoot.motor_fdb.motor_speed_fdb_mid = shoot.shoot_motor_mid[1].fdb.vel;
    shoot.motor_fdb.motor_speed_fdb_rear = shoot.shoot_motor_rear[0].fdb.vel;//电机速度反馈赋值
    shoot.motor_fdb.motor_speed_fdb_rear = shoot.shoot_motor_rear[1].fdb.vel;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartShootReference(void) 
{
    shoot.motor_ref.motor_speed_ref_front = SHOOT_READY_SPEED_FRONT;
    shoot.motor_ref.motor_speed_ref_mid = SHOOT_READY_SPEED_MID;
    shoot.motor_ref.motor_speed_ref_rear = SHOOT_READY_SPEED_REAR;
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootConsole(void) 
{
    shoot.shoot_motor_front[0].set.curr = PID_calc(&shoot.motor_speed_pid_front, shoot.motor_fdb.motor_speed_fdb_front, shoot.motor_ref.motor_speed_ref_front);
    shoot.shoot_motor_front[1].set.curr = PID_calc(&shoot.motor_speed_pid_front, shoot.motor_fdb.motor_speed_fdb_front, shoot.motor_ref.motor_speed_ref_front);
    shoot.shoot_motor_mid[0].set.curr = PID_calc(&shoot.motor_speed_pid_mid, shoot.motor_fdb.motor_speed_fdb_mid, shoot.motor_ref.motor_speed_ref_mid);
    shoot.shoot_motor_mid[1].set.curr = PID_calc(&shoot.motor_speed_pid_mid, shoot.motor_fdb.motor_speed_fdb_mid, shoot.motor_ref.motor_speed_ref_mid);
    shoot.shoot_motor_rear[0].set.curr = PID_calc(&shoot.motor_speed_pid_rear, shoot.motor_fdb.motor_speed_fdb_rear, shoot.motor_ref.motor_speed_ref_rear);
    shoot.shoot_motor_rear[1].set.curr = PID_calc(&shoot.motor_speed_pid_rear, shoot.motor_fdb.motor_speed_fdb_rear, shoot.motor_ref.motor_speed_ref_rear);
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartShootSendCmd(void) 
{
    CanCmdDjiMotor(SHOOT_CAN,SHOOT_STD_ID_1, -shoot.shoot_motor_front[0].set.curr, shoot.shoot_motor_front[1].set.curr, -shoot.shoot_motor_mid[0].set.curr, shoot.shoot_motor_mid[1].set.curr);
    CanCmdDjiMotor(SHOOT_CAN, SHOOT_STD_ID_2, -shoot.shoot_motor_rear[0].set.curr, shoot.shoot_motor_rear[1].set.curr, 0, 0);
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
