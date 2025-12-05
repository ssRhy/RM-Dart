#include "dart_feed.h"

#if (DART_FEED_TYPE == DART_FEED)

Dart_feed dart;

void DartInit(void)
{
    MotorInit(&dart.feed_motor,DART_FEED_MOTOR_ID, MOTOR_DART_CAN, DART_FEED_TYPE, MOTOR_DART_DIRECTION, MOTOR_DART_REDUCTION, MOTOR_DART_MODE);//飞镖供弹电机初始化

    dart.timer = 0; // 计时器重置
    
    // PID参数初始化
    const fp32 pid_speed[3] = {DART_FEED_SPEED_PID_KP, DART_FEED_SPEED_PID_KI, DART_FEED_SPEED_PID_KD}; // 速度环PID参数

    PID_init(&dart.pid, PID_POSITION, pid_speed, DART_FEED_PID_MAX_OUT,DART_FEED_PID_MAX_IOUT);
}

void DartObserver(void)
{
    GetMotorMeasure(&dart.feed_motor);

}

void DartReference(void)
{
    // 设置目标速度
    dart.speed_ref = DART_FEED_SPEED;
}

void DartConsole(void)
{
    // 使用PID计算电流输出
    dart.feed_motor.set.curr = PID_calc(&dart.pid, dart.feed_motor.fdb.vel, dart.speed_ref);

}

void DartSendCmd(void)
{
     CanCmdDjiMotor(MOTOR_DART_CAN, DART_FEED_STD_ID , dart.feed_motor.set.curr, 0, 0, 0);
    
    dart.timer++;
}




#endif