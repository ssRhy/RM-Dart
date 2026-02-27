/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_chasis.c/h
  * @brief      飞镖底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-10-25      CJH            1. 完成基本框架
  *  V1.1.0     2025-11-08      CJH            1. 完成电机控制任务
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "dart_chasis.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "robot_param.h"
#include "CAN_cmd_dji.h"
#include "CAN_receive.h"

static Chassis_s chassis = {
  .move_flag = 0,
  .last_time = 0,
};

static fp32 chassis_delta;
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void) 
{ 
    MotorInit(&chassis.chassis_motor,4, 1,DJI_M6020, -1, 1.0f, 0);

    const fp32 pid_angel[3] = {CHASSIS_ANGEL_PID_KP, CHASSIS_ANGEL_PID_KI, CHASSIS_ANGEL_PID_KD}; 
    const fp32 pid_speed[3] = {CHASSIS_SPEED_PID_KP, CHASSIS_SPEED_PID_KI, CHASSIS_SPEED_PID_KD}; 

    PID_init(&chassis.motor_angle_pid, PID_POSITION, pid_angel, CHASSIS_ANGEL_PID_MAX_OUT, CHASSIS_ANGEL_PID_MAX_IOUT);   
    PID_init(&chassis.motor_speed_pid, PID_POSITION, pid_speed, CHASSIS_PID_MAX_OUT, CHASSIS_PID_MAX_IOUT);   
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if (chassis.move_flag == 0 && (chassis.time - chassis.last_time) >= CHANGE_TIME)
    {
        chassis.mode = CHASSIS_MOTOR_ANGEL;
        chassis.last_time = chassis.time;
    }
    else
    {
        chassis.mode = CHASSIS_MOTOR_STOP;
    }

    if (chassis.move_flag == 1)
    {
        chassis.mode = CHASSIS_MOTOR_ANGEL;
    }
    
    
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) 
{
    GetMotorMeasure(&chassis.chassis_motor);//获取电机反馈值

    chassis.motor_fdb.motor_speed_fdb = chassis.chassis_motor.fdb.vel;//电机速度反馈赋值

    if (chassis.chassis_motor.fdb.ecd - chassis.last_ecd > HALF_ECD_RANGE)
    {
        chassis.ecd_count--;
    }
    else if (chassis.chassis_motor.fdb.ecd - chassis.last_ecd < -HALF_ECD_RANGE)
    {
        
        chassis.ecd_count++;
    }

    if (chassis.ecd_count == FULL_COUNT)
    {
        chassis.ecd_count = -(FULL_COUNT - 1);
    }
    else if (chassis.ecd_count == -FULL_COUNT)
    {
        chassis.ecd_count = FULL_COUNT-1;
    }
    //计算输出轴角度
    chassis.motor_fdb.motor_angle_fdb = (chassis.ecd_count * ECD_RANGE + chassis.chassis_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
    chassis.last_ecd = chassis.chassis_motor.fdb.ecd;

    chassis.time = osKernelSysTick();//获取当前时间
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void) 
{
    if(chassis.mode == CHASSIS_MOTOR_STOP)
    {
        chassis.motor_ref.motor_speed_ref = STOP_SPEED;//设置电机速度期望
    }
    else if(chassis.mode == CHASSIS_MOTOR_ANGEL)
    {
        if (chassis.move_flag == 0)
        {
            chassis.motor_ref.motor_angle_ref = theta_format(chassis.motor_fdb.motor_angle_fdb + PI/3);
        }

        if (theta_format(chassis.motor_ref.motor_angle_ref - chassis.motor_fdb.motor_angle_fdb) > 0.001f)
        {
            chassis.move_flag = 1;
        }
        else
        {
            chassis.move_flag = 0;
        }
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void) 
{
    if(chassis.mode == CHASSIS_MOTOR_STOP)
    {
        chassis.chassis_motor.set.curr = PID_calc(&chassis.motor_speed_pid, chassis.motor_fdb.motor_speed_fdb , chassis.motor_ref.motor_speed_ref);//计算电机控制量
    }
    else if(chassis.mode == CHASSIS_MOTOR_ANGEL)
    {
        chassis_delta = theta_format(chassis.motor_ref.motor_angle_ref - chassis.motor_fdb.motor_angle_fdb);

        chassis.motor_ref.motor_speed_ref = PID_calc(&chassis.motor_angle_pid,0,chassis_delta);
        chassis.chassis_motor.set.curr = PID_calc(&chassis.motor_speed_pid,chassis.motor_fdb.motor_speed_fdb, chassis.motor_ref.motor_speed_ref);
    }
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void) 
{
    //CanCmdDjiMotor(CHASSIS_CAN,CHASSIS_STD_ID, chassis.chassis_motor.set.curr, 0, 0, 0);
    //CanCmdDjiMotor(CHASSIS_CAN,CHASSIS_STD_ID, 0, 0, 0, 0);

    ModifyDebugDataPackage(1, chassis.motor_ref.motor_angle_ref, "ref");
    ModifyDebugDataPackage(2, chassis.motor_fdb.motor_angle_fdb, "fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
