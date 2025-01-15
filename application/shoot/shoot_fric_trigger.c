/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "shoot_fric_trigger.h"

#include "CAN_communication.h"
// 导入usb通信相关的库
#include "usb_debug.h"
#include "user_lib.h"
#include "arm_math.h"

#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

static Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0
};

fp32 delta;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ShootInit(void) 
{ 
  //获取遥控器指针
  SHOOT.rc = get_remote_control_point(); 

  //电机初始化
  MotorInit(&SHOOT.trigger_motor,TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f, 0);//初始化拨弹盘电机结构体
  MotorInit(&SHOOT.fric_motor[0],FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化R摩擦轮电机结构体
  MotorInit(&SHOOT.fric_motor[1],FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化L摩擦轮电机结构体

  //pid初始化
  const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环
  const fp32 pid_speed_trigger[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};//拨弹盘速度环
  
  const fp32 pid_fric[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};//摩擦轮速度环

  PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT);
  PID_init(&SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid
  
  PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
  PID_init(&SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);//摩擦轮初始化pid
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ShootSetMode(void)
{
  /*键鼠遥控器控制方式初版----------------------------*/
      if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))//上档防止误触
    {
        SHOOT.mode = LOAD_STOP;
        SHOOT.state = FRIC_NOT_READY;
    } 

    else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        if(SHOOT.rc->key.v & KEY_PRESSED_OFFSET_Q)//Q启动摩擦轮
        {
          SHOOT.fric_flag = 1;
        }
        else if(SHOOT.rc->key.v & KEY_PRESSED_OFFSET_E)//E关闭摩擦轮
        {
          SHOOT.fric_flag = 0;
        }
        
        if (SHOOT.fric_flag)
        {
            SHOOT.state = FRIC_READY;
        }
        else
        {
            SHOOT.state = FRIC_NOT_READY;
        }


        if (SHOOT.rc->mouse.press_l)
        {
          SHOOT.shoot_flag = 1;
        }
        
        if(SHOOT.move_flag == 0 && SHOOT.shoot_flag != SHOOT.rc->mouse.press_l)
        {
            SHOOT.mode = LAOD_BULLET;
            SHOOT.shoot_flag = 0;
        }
        else if (SHOOT.rc->mouse.press_r)
        {
          SHOOT.mode = LOAD_BURSTFIRE;
        }
        else if(!SHOOT.rc->mouse.press_r && !SHOOT.move_flag)
        {
          SHOOT.mode = LOAD_STOP;
        }

        if (SHOOT.move_flag)
        {
          SHOOT.mode = LAOD_BULLET;
        }
        
        
    } 

    else if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
    {
        SHOOT.state = FRIC_READY;
        SHOOT.mode = LOAD_BURSTFIRE;
    }


    //根据s0判断 如果云台状态是 无力状态，就关闭射击（安全档）
    if ((switch_is_down(SHOOT.rc->rc.s[0])))
    {
        SHOOT.mode = LOAD_STOP;
        SHOOT.state = FRIC_NOT_READY;
    }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ShootObserver(void) 
{
  GetMotorMeasure(&SHOOT.trigger_motor);
  GetMotorMeasure(&SHOOT.fric_motor[0]);
  GetMotorMeasure(&SHOOT.fric_motor[1]);


}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void) 
{
  //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  if(FULL_COUNT%2 == 0)
  {
    if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
    {
        SHOOT.ecd_count--;
    }
    else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
    {
        
        SHOOT.ecd_count++;
    }

    if (SHOOT.ecd_count >= FULL_COUNT)
    {
        SHOOT.ecd_count = SHOOT.ecd_count-2*FULL_COUNT+1;
    }
    else if (SHOOT.ecd_count <= -FULL_COUNT)
    {
        SHOOT.ecd_count = SHOOT.ecd_count+2*FULL_COUNT-1;
    }
  }
  //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 51圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
  else
  {
    if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
    {
        SHOOT.ecd_count--;
    }
    else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
    {
        
        SHOOT.ecd_count++;
    }

    if (SHOOT.ecd_count >= FULL_COUNT)
    {
        SHOOT.ecd_count -= FULL_COUNT*2;
    }
    else if (SHOOT.ecd_count <= -FULL_COUNT)
    {
        SHOOT.ecd_count += FULL_COUNT*2;
    }
  }

    //计算输出轴角度
  SHOOT.trigger_angel = (SHOOT.ecd_count * ECD_RANGE + SHOOT.trigger_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

    //记录上一个ecd值
  SHOOT.last_ecd = SHOOT.trigger_motor.fdb.ecd;

    switch (SHOOT.state)
    {
    case FRIC_NOT_READY:
    SHOOT.fric_motor[0].set.vel=0.0f;
    SHOOT.fric_motor[1].set.vel=0.0f;
    break;

    case FRIC_READY:
    SHOOT.fric_motor[0].set.vel=FRIC_SPEED;
    SHOOT.fric_motor[1].set.vel=-(FRIC_SPEED);
    break;
    
    default:
    break;
    }

    switch (SHOOT.mode)
    {
    case LOAD_STOP:
    SHOOT.trigger_motor.set.vel=0.0f;
    break;
    
    case LAOD_BULLET:
    if (SHOOT.move_flag == 0)
    {
      SHOOT.trigger_motor.set.pos = rad_format(SHOOT.trigger_angel + PI/BULLET_NUM);
    }

    if (rad_format(SHOOT.trigger_motor.set.pos - SHOOT.trigger_angel) > 0.01f)
    {
      SHOOT.move_flag = 1;
    }
    else
    {
      SHOOT.move_flag = 0;
    }

    break;

    case LOAD_BURSTFIRE:
    SHOOT.trigger_motor.set.vel=TRIGGER_SPEED;
    break;

    default:
      break;
    }

  //防堵转
  if (SHOOT.mode == LOAD_BURSTFIRE||SHOOT.mode == LAOD_BULLET)
  {
  if(SHOOT.block_time >= BLOCK_TIME)
  {
      SHOOT.mode = LOAD_BLOCK;
  }

  if(SHOOT.trigger_motor.fdb.vel<BLOCK_TRIGGER_SPEED&&SHOOT.block_time<BLOCK_TIME)
  {
        SHOOT.block_time++;
        SHOOT.reverse_time = 0;
  }
  else if(SHOOT.block_time== BLOCK_TIME&& SHOOT.reverse_time< REVERSE_TIME)
  {
        SHOOT.reverse_time++;  
  }
  else
  {
        SHOOT.block_time = 0;
  }
      
  }

  if (SHOOT.mode == LOAD_BLOCK)
  {
    SHOOT.trigger_motor.set.vel = REVERSE_SPEED;
  }
  
//摩擦轮停止时拨弹盘停止
  if (SHOOT.mode == LOAD_BURSTFIRE)
  {
    if (SHOOT.fric_motor[0].fdb.vel < FRIC_SPEED_LIMIT)
    {
      SHOOT.trigger_motor.set.vel = 0;
    } 
  }
  if (SHOOT.mode == LAOD_BULLET)
  {
    if (SHOOT.fric_motor[0].fdb.vel < FRIC_SPEED_LIMIT)
    {
      SHOOT.trigger_motor.set.pos = SHOOT.trigger_angel;
    } 
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
  SHOOT.fric_motor[0].set.curr=PID_calc(&SHOOT.fric_pid[0], SHOOT.fric_motor[0].fdb.vel,SHOOT.fric_motor[0].set.vel);
  SHOOT.fric_motor[1].set.curr=PID_calc(&SHOOT.fric_pid[1], SHOOT.fric_motor[1].fdb.vel,SHOOT.fric_motor[1].set.vel);
  
    if (SHOOT.mode == LOAD_STOP)
    {
        SHOOT.trigger_motor.set.curr =PID_calc(&SHOOT.trigger_speed_pid, SHOOT.trigger_motor.fdb.vel, SHOOT.trigger_motor.set.vel);
    }
    else if (SHOOT.mode == LOAD_BURSTFIRE)
    {
        SHOOT.trigger_motor.set.curr =PID_calc(&SHOOT.trigger_speed_pid, SHOOT.trigger_motor.fdb.vel, SHOOT.trigger_motor.set.vel);
    }
    else if (SHOOT.mode == LAOD_BULLET)
    {
        delta = rad_format(SHOOT.trigger_motor.set.pos - SHOOT.trigger_angel);

        SHOOT.trigger_motor.set.vel = PID_calc(&SHOOT.trigger_angel_pid,0,delta);
        SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid,SHOOT.trigger_motor.fdb.vel,SHOOT.trigger_motor.set.vel);
    }
    else if (SHOOT.mode == LOAD_BLOCK) 
    {
        SHOOT.trigger_motor.set.curr =PID_calc(&SHOOT.trigger_speed_pid, SHOOT.trigger_motor.fdb.vel, SHOOT.trigger_motor.set.vel);
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
  CanCmdDjiMotor(FRIC_MOTOR_R_CAN, FRIC_STD_ID , SHOOT.fric_motor[1].set.curr,SHOOT.fric_motor[0].set.curr,0, 0);
  CanCmdDjiMotor(TRIGGER_MOTOR_CAN, TRIGGER_STD_ID ,0 ,0 ,SHOOT.trigger_motor.set.curr, 0);

  ModifyDebugDataPackage(1,SHOOT.state,"state"); 
  ModifyDebugDataPackage(2,SHOOT.mode,"mode");
  // ModifyDebugDataPackage(2,SHOOT.fric_motor[0].set.vel,"fric0_set");
  // ModifyDebugDataPackage(1,SHOOT.fric_motor[0].fdb.vel,"fric0_fdb");
  // ModifyDebugDataPackage(4,SHOOT.fric_motor[1].set.vel,"fric1_set");
  // ModifyDebugDataPackage(2,SHOOT.fric_motor[1].fdb.vel,"fric1_fdb");
  // ModifyDebugDataPackage(6,SHOOT.trigger_motor.set.vel,"trigger_set");
  // ModifyDebugDataPackage(7,SHOOT.trigger_motor.fdb.vel,"trigger_fdb");
  
  
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
