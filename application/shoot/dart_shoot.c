#include "dart_shoot.h"
#include "pid.h"
#if (SHOOT_TYPE == SHOOT_DART_FRIC)

Dart_shoot dart;
fp32 delta;
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

    const fp32 pid_speed_shoot[3] = {DART_SHOOT_SPEED_PID_KP, DART_SHOOT_SPEED_PID_KI, DART_SHOOT_SPEED_PID_KD}; //飞镖发射速度环
    const fp32 pid_angel_shoot[3] = {DART_SHOOT_ANGEL_PID_KP, DART_SHOOT_ANGEL_PID_KI, DART_SHOOT_ANGEL_PID_KD}; //飞镖发射角度环

    for(int i = 0; i < 6; i++) {
        PID_init(&dart.speed_pid[i], PID_POSITION, pid_speed_shoot, DART_SHOOT_SPEED_PID_MAX_OUT, DART_SHOOT_SPEED_PID_MAX_IOUT); //飞镖发射初始化speedpid
    }

    
    for(int i = 0; i < 6; i++) {
        PID_init(&dart.angel_pid[i], PID_POSITION, pid_angel_shoot, DART_SHOOT_ANGEL_PID_MAX_OUT, DART_SHOOT_ANGEL_PID_MAX_IOUT); //飞镖发射初始化angelpid
    }
}


void ShootSetMode(void)
{

}


//更新状态量
void ShootObserver(void)
{
    for(int i=0;i<5;i++)
    { 
        GetMotorMeasure(&dart.shoot_motor[i]);
    }

    for(int i=0;i<5;i++)
    {
    
        dart.FDB.shoot_speed_fdb[i] = dart.shoot_motor[i].fdb.vel;
    }

    for(int i=0;i<5;i++)
    {
        dart.FDB.shoot_angel_fdb[i] = dart.shoot_motor[i].fdb.vel;
    }
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
           dart.shoot_motor[i].set.curr=PID_calc(&dart.speed_pid[i], dart.FDB.shoot_speed_fdb[i], dart.REF.shoot_speed_ref[i]);
        }
    }

    else if(dart.mode == DART_SHOOT_FIRE)
    {
        for(int i=0;i<5;i++)
        {
         
           delta= theta_format(dart.REF.shoot_angel_ref[i] - dart.FDB.shoot_angel_fdb[i]);
           dart.REF.shoot_speed_ref[i]=PID_calc(&dart.angel_pid[i],0,delta);
           dart.shoot_motor[i].set.curr=PID_calc(&dart.speed_pid[i], dart.FDB.shoot_speed_fdb[i], dart.REF.shoot_speed_ref[i]);

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
        CanCmdDjiMotor(MOTOR_DART_CAN, DART_CHASSIS_STD_ID , dart.shoot_motor[0].set.curr,dart.shoot_motor[1].set.curr,dart.shoot_motor[2].set.curr,0);
    }
    for(int i=3;i<6;i++)
    {
        CanCmdDjiMotor(MOTOR_DART_CAN, DART_CHASSIS_STD_ID , 0,dart.shoot_motor[4].set.curr,dart.shoot_motor[5].set.curr,dart.shoot_motor[3].set.curr);
    }
}

#endif  //  DART_SHOOT_TYPE == DART_SHOOT


