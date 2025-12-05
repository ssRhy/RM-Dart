#include "dart_shoot.h"
#include "pid.h"
#if (SHOOT_TYPE == SHOOT_DART_FRIC)

Dart_up dart;

void ShootInit(void)
{
    //获取遥控器指针
    //DART.rc = get_remote_control_point(); 

    //飞镖发射电机初始化
    MotorInit(&dart.shoot_motor[0], DART_SHOOT_MOTOR_0_ID, DART_SHOOT_MOTOR_LEFT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体
    MotorInit(&dart.shoot_motor[1], DART_SHOOT_MOTOR_1_ID, DART_SHOOT_MOTOR_LEFT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体
    MotorInit(&dart.shoot_motor[2], DART_SHOOT_MOTOR_2_ID, DART_SHOOT_MOTOR_LEFT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体
    MotorInit(&dart.shoot_motor[3], DART_SHOOT_MOTOR_3_ID, DART_SHOOT_MOTOR_RIGHT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体
    MotorInit(&dart.shoot_motor[4], DART_SHOOT_MOTOR_4_ID, DART_SHOOT_MOTOR_RIGHT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体
    MotorInit(&dart.shoot_motor[5], DART_SHOOT_MOTOR_5_ID, DART_SHOOT_MOTOR_RIGHT_CAN, DART_SHOOT_MOTOR_TYPE, 1, 1.0f, 0);//初始化飞镖发射电机结构体

    const fp32 pid_shoot[3] = {DART_SHOOT_PID_KP, DART_SHOOT_PID_KI, DART_SHOOT_PID_KD};//飞镖发射速度环

    PID_init(&dart.pid[0], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid
    PID_init(&dart.pid[1], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid
    PID_init(&dart.pid[2], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid
    PID_init(&dart.pid[3], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid
    PID_init(&dart.pid[4], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid
    PID_init(&dart.pid[5], PID_POSITION, pid_shoot, DART_SHOOT_PID_MAX_OUT, DART_SHOOT_PID_MAX_IOUT);//飞镖发射初始化pid

}


void ShootSetMode(void)
{

}


//更新状态量
void ShootObserver(void)
{
    GetMotorMeasure(&dart.shoot_motor[0]);
    GetMotorMeasure(&dart.shoot_motor[1]); 
    GetMotorMeasure(&dart.shoot_motor[2]);
    GetMotorMeasure(&dart.shoot_motor[3]); 
    GetMotorMeasure(&dart.shoot_motor[4]);
    GetMotorMeasure(&dart.shoot_motor[5]);

    dart.FDB.shoot_speed_fdb[0] = dart.shoot_motor[0].fdb.vel;
    dart.FDB.shoot_speed_fdb[1] = dart.shoot_motor[1].fdb.vel;
    dart.FDB.shoot_speed_fdb[2] = dart.shoot_motor[2].fdb.vel;
    dart.FDB.shoot_speed_fdb[3] = dart.shoot_motor[3].fdb.vel;
    dart.FDB.shoot_speed_fdb[4] = dart.shoot_motor[4].fdb.vel;
    dart.FDB.shoot_speed_fdb[5] = dart.shoot_motor[5].fdb.vel;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void)
{
    //是否准备
    switch(dart.state)
    {
        case DART_SHOOT_NOT_READY:
        for(int i=0;i<6;i++)
        {
            dart.REF.shoot_speed_ref[i]=0.0f;
        }
        break;

        case DART_SHOOT_READY:
        for(int i=0;i<6;i++)
        {
            dart.REF.shoot_speed_ref[i]=DART_SPEED;
        }
        break;
        
        default:
        break;
    }
    
    //模式选择
    switch(dart.mode)
    {
        case DART_SHOOT_STOP:
        for(int i=0;i<6;i++)
        {
            dart.REF.shoot_speed_ref[i]=0.0f;
        }
        break;
        
        case DART_LOAD:
        for(int i=0;i<6;i++)
        {
            //供弹
        }

        case DART_SHOOT_FIRE:
        for(int i=0;i<6;i++)
        {     
            //设置飞镖发射目标速度
            dart.REF.shoot_speed_ref[i] = DART_SPEED;
        }

        default:
        break;
    }
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */

void ShootConsole(void)
{

    if(dart.mode == DART_SHOOT_STOP)
    {
        for(int i=0;i<6;i++)
        {
           dart.shoot_motor[i].set.curr=PID_calc(&dart.pid[i], dart.FDB.shoot_speed_fdb[i], dart.REF.shoot_speed_ref[i]);
    }
    }

    else if(dart.mode == DART_SHOOT_FIRE)
    {
        for(int i=0;i<6;i++)
        {
           dart.shoot_motor[i].set.curr=PID_calc(&dart.pid[i], dart.FDB.shoot_speed_fdb[i], dart.REF.shoot_speed_ref[i]);
        }
    }
    
    else if(dart.mode == DART_LOAD)
    {
       //供弹控制
    }
}


/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ShootSendCmd(void)
{
    for(int i=0;i<3;i++)
    {
        CanCmdDjiMotor(MOTOR_DART_CAN, SHOOT_STD_ID_0 , dart.shoot_motor[0].set.curr,dart.shoot_motor[1].set.curr,dart.shoot_motor[2].set.curr,0);
    }
    for(int i=3;i<6;i++)
    {
        CanCmdDjiMotor(MOTOR_DART_CAN, SHOOT_STD_ID_1 , 0,dart.shoot_motor[4].set.curr,dart.shoot_motor[5].set.curr,dart.shoot_motor[3].set.curr);
    }
}

#endif  //  DART_SHOOT_TYPE == DART_SHOOT