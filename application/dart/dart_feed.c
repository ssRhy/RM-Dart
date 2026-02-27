/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       dart_trans.c/h
  * @brief      电机飞镖机构控制器。
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

#include "dart_feed.h"

#if (CHASSIS_TYPE == DART_CHASSIS)

#include "robot_param.h"

static Feed_s feed = {
  .move_flag = 0,
  .last_time = 0,
};

static fp32 feed_delta;
/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void DartFeedInit(void) 
{ 
    MotorInit(&feed.feed_motor,3, 1,DJI_M2006, -1, 1.0f, 0);

    const fp32 pid_angel[3] = {FEED_ANGEL_PID_KP, FEED_ANGEL_PID_KI, FEED_ANGEL_PID_KD}; 
    const fp32 pid_speed[3] = {FEED_SPEED_PID_KP, FEED_SPEED_PID_KI, FEED_SPEED_PID_KD}; 

    PID_init(&feed.motor_angle_pid, PID_POSITION, pid_angel, FEED_ANGEL_PID_MAX_OUT, FEED_ANGEL_PID_MAX_IOUT);   
    PID_init(&feed.motor_speed_pid, PID_POSITION, pid_speed, FEED_PID_MAX_OUT, FEED_PID_MAX_IOUT);   
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void DartFeedSetMode(void)
{
    if (feed.move_flag == 0 && (feed.time - feed.last_time) >= CHANGE_TIME)
    {
        feed.mode = FEED_MOTOR_ANGEL;
        feed.last_time = feed.time;
    }
    else
    {
        feed.mode = FEED_MOTOR_STOP;
    }

    if (feed.move_flag == 1)
    {
        feed.mode = FEED_MOTOR_ANGEL;
    }
    
    
}

/*-------------------- Observer --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void DartFeedObserver(void) 
{
    GetMotorMeasure(&feed.feed_motor);//获取电机反馈值

    feed.motor_fdb.motor_speed_fdb = feed.feed_motor.fdb.vel;//电机速度反馈赋值

    if (feed.feed_motor.fdb.ecd - feed.last_ecd > HALF_ECD_RANGE)
    {
        feed.ecd_count--;
    }
    else if (feed.feed_motor.fdb.ecd - feed.last_ecd < -HALF_ECD_RANGE)
    {
        
        feed.ecd_count++;
    }

    if (feed.ecd_count == FULL_COUNT)
    {
        feed.ecd_count = -(FULL_COUNT - 1);
    }
    else if (feed.ecd_count == -FULL_COUNT)
    {
        feed.ecd_count = FULL_COUNT-1;
    }
    //计算输出轴角度
    feed.motor_fdb.motor_angle_fdb = (feed.ecd_count * ECD_RANGE + feed.feed_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
    feed.last_ecd = feed.feed_motor.fdb.ecd;

    feed.time = osKernelSysTick();//获取当前时间
}

/*-------------------- Reference --------------------*/

/**
 * @brief          设置目标量
 * @param[in]      none
 * @retval         none
 */
void DartFeedReference(void) 
{
    if(feed.mode == FEED_MOTOR_STOP)
    {
        feed.motor_ref.motor_speed_ref = STOP_SPEED;//设置电机速度期望
    }
    else if(feed.mode == FEED_MOTOR_ANGEL)
    {
        if (feed.move_flag == 0)
        {
            feed.motor_ref.motor_angle_ref = theta_format(feed.motor_fdb.motor_angle_fdb + PI/3);
        }

        if (theta_format(feed.motor_ref.motor_angle_ref - feed.motor_fdb.motor_angle_fdb) > 0.001f)
        {
            feed.move_flag = 1;
        }
        else
        {
            feed.move_flag = 0;
        }
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void DartFeedConsole(void) 
{
    if(feed.mode == FEED_MOTOR_STOP)
    {
        feed.feed_motor.set.curr = PID_calc(&feed.motor_speed_pid, feed.motor_fdb.motor_speed_fdb , feed.motor_ref.motor_speed_ref);//计算电机控制量
    }
    else if(feed.mode == FEED_MOTOR_ANGEL)
    {
        feed_delta = theta_format(feed.motor_ref.motor_angle_ref - feed.motor_fdb.motor_angle_fdb);

        feed.motor_ref.motor_speed_ref = PID_calc(&feed.motor_angle_pid,0,feed_delta);
        feed.feed_motor.set.curr = PID_calc(&feed.motor_speed_pid,feed.motor_fdb.motor_speed_fdb, feed.motor_ref.motor_speed_ref);
    }
}

/*-------------------- Send cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void DartFeedSendCmd(void) 
{
    // CanCmdDjiMotor(FEED_CAN,FEED_STD_ID, 0, feed.feed_motor.set.curr, 0, 0);
    //CanCmdDjiMotor(FEED_CAN,FEED_STD_ID, 0, 0, 0, 0);

    ModifyDebugDataPackage(1, feed.motor_ref.motor_angle_ref, "ref");
    ModifyDebugDataPackage(2, feed.motor_fdb.motor_angle_fdb, "fdb");
}

#endif  // CHASSIS_TYPE == DART_CHASSIS
