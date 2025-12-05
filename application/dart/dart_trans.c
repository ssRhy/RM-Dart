#include "dart_trans.h"

#if (DART_TRANS_TYPE == DART_TRANS)

Dart_trans dart;

void DartInit(void)
{
    MotorInit(&dart.trans_motor,DART_TRANS_MOTOR_ID, MOTOR_DART_CAN, DART_TRANS_MOTOR_TYPE, MOTOR_DART_DIRECTION, MOTOR_DART_REDUCTION, MOTOR_DART_MODE);//飞镖供弹电机初始化

    dart.timer = 0; // 计时器重置
    
    // PID参数初始化
    const fp32 pid_speed[3] = {DART_TRANS_SPEED_PID_KP, DART_TRANS_SPEED_PID_KI, DART_TRANS_SPEED_PID_KD}; // 速度环PID参数

    PID_init(&dart.pid, PID_POSITION, pid_speed, DART_TRANS_PID_MAX_OUT,DART_TRANS_PID_MAX_IOUT);
}

void DartObserver(void)
{
    GetMotorMeasure(&dart.trans_motor);

}

void DartReference(void)
{
    // 设置目标速度
    dart.speed_ref = DART_TRANS_SPEED;
}

void DartConsole(void)
{
    // 使用PID计算电流输出
    dart.trans_motor.set.curr = PID_calc(&dart.pid, dart.trans_motor.fdb.vel, dart.speed_ref);

}

void DartSendCmd(void)
{
     CanCmdDjiMotor(MOTOR_DART_CAN, DART_TRANS_STD_ID , dart.trans_motor.set.curr, 0, 0, 0);
    
    dart.timer++;
}


#endif