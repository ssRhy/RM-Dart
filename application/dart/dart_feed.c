#include "dart_feed.h"

#if (DART_FEED_TYPE == DART_FEED)

Dart_feed dart;

void DartInit(void)
{
    MotorInit(&dart.feed_motor,DART_FEED_MOTOR_ID, MOTOR_DART_CAN, DART_FEED_TYPE, MOTOR_DART_DIRECTION, MOTOR_DART_REDUCTION, MOTOR_DART_MODE);//飞镖供弹电机初始化

    dart.timer = 0; // 计时器重置
    
    // PID参数初始化
    const fp32 pid_speed[3] = {DART_FEED_SPEED_PID_KP, DART_FEED_SPEED_PID_KI, DART_FEED_SPEED_PID_KD}; // 速度环PID参数
    const fp32 pid_angel[3] = {DART_FEED_ANGEL_PID_KP, DART_FEED_ANGEL_PID_KI, DART_FEED_ANGEL_PID_KD}; // 角度环PID参数

    PID_init(&dart.feed_speed_pid, PID_POSITION, pid_speed, DART_FEED_SPEED_PID_MAX_OUT, DART_FEED_SPEED_PID_MAX_IOUT); // 初始化速度PID
    PID_init(&dart.feed_angel_pid, PID_POSITION, pid_angel, DART_FEED_ANGEL_PID_MAX_OUT, DART_FEED_ANGEL_PID_MAX_IOUT); // 初始化角度PID
}

void DartObserver(void)
{
    GetMotorMeasure(&dart.feed_motor);
    
    // 更新反馈值
    dart.feed_speed_fdb = dart.feed_motor.fdb.vel;  // 速度反馈
    dart.feed_angel_fdb = dart.feed_motor.fdb.vel;  // 角度反馈
}

void DartReference(void)
{
    // 设置目标速度
    dart.feed_speed_ref = DART_FEED_SPEED;
}

void DartConsole(void)
{
    fp32 delta;
    
    // 角度环控制
    delta = theta_format(dart.feed_angel_ref - dart.feed_angel_fdb);
    dart.feed_speed_ref = PID_calc(&dart.feed_angel_pid, 0, delta);
    
    // 速度环控制
    dart.feed_motor.set.curr = PID_calc(&dart.feed_speed_pid, dart.feed_speed_fdb, dart.feed_speed_ref);
}

void DartSendCmd(void)
{
     CanCmdDjiMotor(MOTOR_DART_CAN, DART_FEED_STD_ID , dart.feed_motor.set.curr, 0, 0, 0);
    
    dart.timer++;
}

#endif