#include "dart_chassis.h"
#include "dart_param.h"
#if (DART_TYPE == DART_CHASSIS)

Dart_s dart;

void DartInit(void)
{
    MotorInit(&dart.motor[0],DART_CHASSIS_ID, MOTOR_DART_CAN, DART_CHASSIS_TYPE,MOTOR_DART_DIRECTION,MOTOR_DART_REDUCTION, MOTOR_DART_MODE);//底盘摩擦轮初始化

    dart.timer = 0; // 计时器重置
    
    // PID参数初始化
    const fp32 pid_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD}; // 速度环PID参数

    PID_init(&dart.pid[0], PID_POSITION, pid_speed, DART_PID_MAX_OUT,DART_PID_MAX_IOUT);
}

void DartObserver(void)
{
    GetMotorMeasure(&dart.motor[0]);
    
}

void DartSetMode(void)
{
   //none
}

void DartReference(void)
{
    // 设置目标速度
    dart.speed_ref = DART_SPEED;
}

void DartConsole(void)
{
    // 使用PID计算电流输出
    dart.motor[0].set.curr = PID_calc(&dart.pid[0], dart.motor[0].fdb.vel, dart.speed_ref);
}

void DartSendCmd(void)
{
     CanCmdDjiMotor(MOTOR_DART_CAN, DART_STD_ID , dart.motor[0].set.curr, 0, 0, 0);
    
    dart.timer++;
}

#endif

