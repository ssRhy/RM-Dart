/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_trans.c/h
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

#include "dart_trans.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "robot_param.h"

static Dart_s dart = {
  .move_flag = 0,
  .last_time = 0,
};

static fp32 trans_delta;
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void DartTransInit(void) 
{ 
    MotorInit(&dart.dart_motor,1, 1,DJI_M3508, 1, 1.0f, 0);

    const fp32 pid_angel[3] = {DART_ANGEL_PID_KP, DART_ANGEL_PID_KI, DART_ANGEL_PID_KD}; 
    const fp32 pid_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD}; 

    PID_init(&dart.motor_angle_pid, PID_POSITION, pid_angel, DART_ANGEL_PID_MAX_OUT, DART_ANGEL_PID_MAX_IOUT);   
    PID_init(&dart.motor_speed_pid, PID_POSITION, pid_speed, DART_PID_MAX_OUT, DART_PID_MAX_IOUT);   
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void DartTransSetMode(void)
{
    if (dart.move_flag == 0 && (dart.time - dart.last_time) >= CHANGE_TIME)
    {
        dart.mode = MOTOR_ANGEL;
        dart.last_time = dart.time;
    }
    else
    {
        dart.mode = MOTOR_STOP;
    }

    if (dart.move_flag == 1)
    {
        dart.mode = MOTOR_ANGEL;
    }
    
    
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void DartTransObserver(void) 
{
    GetMotorMeasure(&dart.dart_motor);//获取电机反馈值

    dart.motor_fdb.motor_speed_fdb = dart.dart_motor.fdb.vel;//电机速度反馈赋值

    if (dart.dart_motor.fdb.ecd - dart.last_ecd > HALF_ECD_RANGE)
    {
        dart.ecd_count--;
    }
    else if (dart.dart_motor.fdb.ecd - dart.last_ecd < -HALF_ECD_RANGE)
    {
        
        dart.ecd_count++;
    }

    if (dart.ecd_count == FULL_COUNT)
    {
        dart.ecd_count = -(FULL_COUNT - 1);
    }
    else if (dart.ecd_count == -FULL_COUNT)
    {
        dart.ecd_count = FULL_COUNT-1;
    }
    //计算输出轴角度
    dart.motor_fdb.motor_angle_fdb = (dart.ecd_count * ECD_RANGE + dart.dart_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
    dart.last_ecd = dart.dart_motor.fdb.ecd;

    dart.time = osKernelSysTick();//获取当前时间
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartTransReference(void) 
{
    if(dart.mode == MOTOR_STOP)
    {
        dart.motor_ref.motor_speed_ref = STOP_SPEED;//设置电机速度期望
    }
    else if(dart.mode == MOTOR_ANGEL)
    {
        if (dart.move_flag == 0)
        {
            dart.motor_ref.motor_angle_ref = theta_format(dart.motor_fdb.motor_angle_fdb + PI/2);
        }

        if (theta_format(dart.motor_ref.motor_angle_ref - dart.motor_fdb.motor_angle_fdb) > 0.001f)
        {
            dart.move_flag = 1;
        }
        else
        {
            dart.move_flag = 0;
        }
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartTransConsole(void) 
{
    if(dart.mode == MOTOR_STOP)
    {
        dart.dart_motor.set.curr = PID_calc(&dart.motor_speed_pid, dart.motor_fdb.motor_speed_fdb , dart.motor_ref.motor_speed_ref);//计算电机控制量
    }
    else if(dart.mode == MOTOR_ANGEL)
    {
        trans_delta = theta_format(dart.motor_ref.motor_angle_ref - dart.motor_fdb.motor_angle_fdb);

        dart.motor_ref.motor_speed_ref = PID_calc(&dart.motor_angle_pid,0,trans_delta);
        dart.dart_motor.set.curr = PID_calc(&dart.motor_speed_pid,dart.motor_fdb.motor_speed_fdb, dart.motor_ref.motor_speed_ref);
    }
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartTransSendCmd(void) 
{
    CanCmdDjiMotor(DART_CAN,DART_TRANS_STD_ID, dart.dart_motor.set.curr, 0, 0, 0);

    ModifyDebugDataPackage(1, dart.motor_ref.motor_angle_ref, "ref");
    ModifyDebugDataPackage(2, dart.motor_fdb.motor_angle_fdb, "fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
