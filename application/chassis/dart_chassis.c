#include "dart_chassis.h"

#if (CHASSIS_TYPE == CHASSIS_DART)

Dart_down dart;

void ChassisInit(void)
{
    MotorInit(&dart.chassis_motor[0],DART_CHASSIS_MOTOR_ID, MOTOR_DART_CAN, DART_CHASSIS_MOTOR_TYPE,MOTOR_DART_DIRECTION,MOTOR_DART_REDUCTION, MOTOR_DART_MODE);//底盘摩擦轮初始化

    dart.timer = 0; // 计时器重置
    
    // PID参数初始化
    const fp32 pid_speed[3] = {DART_SPEED_PID_KP, DART_SPEED_PID_KI, DART_SPEED_PID_KD}; // 速度环PID参数

    PID_init(&dart.pid[0], PID_POSITION, pid_speed, DART_PID_MAX_OUT,DART_PID_MAX_IOUT);
}

void ChassisObserver(void)
{
    GetMotorMeasure(&dart.chassis_motor[0]);
    
}

void ChassisSetMode(void)
{
   //none
}

void ChassisReference(void)
{
    // 设置目标速度
    dart.speed_ref = DART_SPEED;
}

void ChassisConsole(void)
{
    // 使用PID计算电流输出
    dart.chassis_motor[0].set.curr = PID_calc(&dart.pid[0], dart.chassis_motor[0].fdb.vel, dart.speed_ref);
}

void ChassisSendCmd(void)
{
     CanCmdDjiMotor(MOTOR_DART_CAN, DART_CHASSIS_STD_ID , dart.chassis_motor[0].set.curr, 0, 0, 0);
    
    dart.timer++;
}

#endif

