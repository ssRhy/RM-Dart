/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_trans.c/h
  * @brief      飞镖机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-01-25      Assistant       1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "dart_trans.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "robot_param.h"


static Dart_s motor_dart = {
  .move_flag = 0,
  .last_time = 0,
};

fp32 delta;
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void DartInit(void) 
{ 
    MotorInit(&motor_dart.dart_motor, DART_TRANS_MOTOR_ID, MOTOR_DART_CAN, MOTOR_TRANS_TYPE, MOTOR_DART_DIRECTION, MOTOR_DART_REDUCTION, MOTOR_DART_MODE);

    const fp32 pid_angel[3] = {DART_ANGEL_PID_KP, DART_ANGEL_PID_KI, DART_ANGEL_PID_KD}; 
    const fp32 pid_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD}; 

    PID_init(&motor_dart.motor_angle_pid, PID_POSITION, pid_angel, DART_ANGEL_PID_MAX_OUT, DART_ANGEL_PID_MAX_IOUT);   
    PID_init(&motor_dart.motor_speed_pid, PID_POSITION, pid_speed, DART_PID_MAX_OUT, DART_PID_MAX_IOUT);   
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void DartSetMode(void)
{
    if (motor_dart.move_flag == 0 && (motor_dart.time - motor_dart.last_time) >= CHANGE_TIME)
    {
        motor_dart.mode = MOTOR_ANGEL;
        motor_dart.last_time = motor_dart.time;
    }
    else
    {
        motor_dart.mode = MOTOR_STOP;
    }

    if (motor_dart.move_flag == 1)
    {
        motor_dart.mode = MOTOR_ANGEL;
    }
    
    
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void DartObserver(void) 
{
    GetMotorMeasure(&motor_dart.dart_motor);//获取电机反馈值

    motor_dart.motor_fdb.motor_speed_fdb = motor_dart.dart_motor.fdb.vel;//电机速度反馈赋值

    if (motor_dart.dart_motor.fdb.ecd - motor_dart.last_ecd > HALF_ECD_RANGE)
    {
        motor_dart.ecd_count--;
    }
    else if (motor_dart.dart_motor.fdb.ecd - motor_dart.last_ecd < -HALF_ECD_RANGE)
    {
        
        motor_dart.ecd_count++;
    }

    if (motor_dart.ecd_count == FULL_COUNT)
    {
        motor_dart.ecd_count = -(FULL_COUNT - 1);
    }
    else if (motor_dart.ecd_count == -FULL_COUNT)
    {
        motor_dart.ecd_count = FULL_COUNT-1;
    }
    //计算输出轴角度
    motor_dart.motor_fdb.motor_angle_fdb = (motor_dart.ecd_count * ECD_RANGE + motor_dart.dart_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
    motor_dart.last_ecd = motor_dart.dart_motor.fdb.ecd;

    motor_dart.time = osKernelSysTick();//获取当前时间
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartReference(void) 
{
    if(motor_dart.mode == MOTOR_STOP)
    {
        motor_dart.motor_ref.motor_speed_ref = STOP_SPEED;//设置电机速度期望
    }
    else if(motor_dart.mode == MOTOR_ANGEL)
    {
        if (motor_dart.move_flag == 0)
        {
            motor_dart.motor_ref.motor_angle_ref = theta_format(motor_dart.motor_fdb.motor_angle_fdb + PI/3);
        }

        if (theta_format(motor_dart.motor_ref.motor_angle_ref - motor_dart.motor_fdb.motor_angle_fdb) > 0.001f)
        {
            motor_dart.move_flag = 1;
        }
        else
        {
            motor_dart.move_flag = 0;
        }
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartConsole(void) 
{
    if(motor_dart.mode == MOTOR_STOP)
    {
        motor_dart.dart_motor.set.curr = PID_calc(&motor_dart.motor_speed_pid, motor_dart.motor_fdb.motor_speed_fdb , motor_dart.motor_ref.motor_speed_ref);//计算电机控制量
    }
    else if(motor_dart.mode == MOTOR_ANGEL)
    {
        delta = theta_format(motor_dart.motor_ref.motor_angle_ref - motor_dart.motor_fdb.motor_angle_fdb);

        motor_dart.motor_ref.motor_speed_ref = PID_calc(&motor_dart.motor_angle_pid,0,delta);
        motor_dart.dart_motor.set.curr = PID_calc(&motor_dart.motor_speed_pid,motor_dart.motor_fdb.motor_speed_fdb, motor_dart.motor_ref.motor_speed_ref);
    }
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartSendCmd(void) 
{
    CanCmdDjiMotor(MOTOR_DART_CAN, DART_TRANS_STD_ID , motor_dart.dart_motor.set.curr, 0, 0, 0);
}

#endif  // CHASSIS_TYPE == DART_CHASSIS


