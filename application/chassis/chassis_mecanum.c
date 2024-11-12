


#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_MECANUM_WHEEL)
#include "chassis_mecanum.h"
#include "CAN_receive.h"
#include "chassis.h"
#include "usb_task.h"
#include "motor.h" 
#include "detect_task.h"
#include "gimbal_yaw_pitch_direct.h"

Motor_s __Motor;
Chassis_s CHASSIS;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    /*-------------------- 初始化PID --------------------*/
    //速度环pid
    float chassis_speed_pid[3] = {KP_CHASSIS_WHEEL_SPEED, KI_CHASSIS_WHEEL_SPEED, KD_CHASSIS_WHEEL_SPEED};
    //yaw轴跟踪pid
    float chassis_yaw_pid[3] = {KP_CHASSIS_GIMBAL_FOLLOW_ANGLE, KI_CHASSIS_GIMBAL_FOLLOW_ANGLE, KD_CHASSIS_GIMBAL_FOLLOW_ANGLE};;
    
    //获取底盘电机数据指针，初始化PID 
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        //底盘不跟随云台下的pid初始化
        PID_init(&CHASSIS.motor_speed_pid[i], PID_POSITION, chassis_speed_pid, MAX_OUT_CHASSIS_WHEEL_SPEED,MAX_IOUT_CHASSIS_WHEEL_SPEED);
    }
    //底盘跟随云台pid初始化
        PID_init(&CHASSIS.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, 
        MAX_OUT_CHASSIS_GIMBAL_FOLLOW_ANGLE,MAX_IOUT_CHASSIS_GIMBAL_FOLLOW_ANGLE);
    
    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.wheel_motor[0], 1, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[1], 2, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[2], 3, 1, DJI_M3508, 1, 1, 1);
    MotorInit(&CHASSIS.wheel_motor[3], 4, 1, DJI_M3508, 1, 1, 1);
}

/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_SPIN;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_FREE;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_ZERO_FORCE;
        //CHASSIS.mode = CHASSIS_FREE;
    }
    
    uint8_t i;
    //具体模式设定
    switch (CHASSIS.mode) {
        case CHASSIS_ZERO_FORCE: { 
            for (i = 0; i < 4; i++) {
                //CHASSIS.wheel_motor[i].set.curr = 0.0f;
                CHASSIS.wz_set = 0.0f;
                CHASSIS.vx_set = 0.0f;
                CHASSIS.vy_set = 0.0f;
                
            }
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW:{
            //云台跟随模式
        {
            fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
            //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
            //arm_sin是stm34f4中特有的三角函数转换函数，运算速度快
            //sin_yaw = arm_sin_f32(/*填入云台与底盘相对角度（注意正负）*/);
            //cos_yaw = arm_cos_f32(/*填入云台与底盘相对角度（注意正负）*/);

            //下面是云台坐标系转换为底盘坐标系的方程
            CHASSIS.vx_set = cos_yaw * CHASSIS.vx_set + sin_yaw * CHASSIS.vy_set;
            CHASSIS.vy_set = -sin_yaw * CHASSIS.vx_set + cos_yaw * CHASSIS.vy_set;//注意正负

        }
        
        }
            break;
        case CHASSIS_STOP:
            break;
        case CHASSIS_FREE:{//底盘不跟随云台
            CHASSIS.wz_set = NORMAL_MIN_CHASSIS_SPEED_WX;//暂时让这个模式下小陀螺不生效
        }
            break;
        case CHASSIS_SPIN:{//小陀螺模式
            CHASSIS.wz_set = NORMAL_MAX_CHASSIS_SPEED_WX;
        }
            break;
        case CHASSIS_AUTO:
            break;
        case CHASSIS_OPEN: {
            uint16_t current;
            current = CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL] * 3000 * RC_TO_ONE;
            for (i = 0; i < 4; i++) {
                CHASSIS.wheel_motor[0].set.curr = current;
            }
            break;
        }
        default:
            break;
    }
}


/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void) {
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }
    //GetMotorMeasure(/*跟新云台与底盘相对角度*/);
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void) {

   /*int16_t vx_channel, vy_channel;
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_Y_CHANNEL], rc_y, CHASSIS_RC_DEADLINE);
    //rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);

    vx_channel = rc_x * CHASSIS_VX_RC_SEN;
    vy_channel = rc_y * CHASSIS_VY_RC_SEN;

    for (i = 0; i < 4; i++)
   {
    CHASSIS.wheel_motor[i].set.vel = CHASSIS_VX_RC_SEN * CHASSIS.rc->rc.ch[3];
   }*/

    //给定摇杆值
    CHASSIS.vx_set = CHASSIS_VX_RC_SEN * CHASSIS.rc->rc.ch[3];
    CHASSIS.vy_set = CHASSIS_VY_RC_SEN * CHASSIS.rc->rc.ch[2];

    
    //键盘控制
    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_W)
    {
        CHASSIS.vx_set = NORMAL_MAX_CHASSIS_SPEED_X;
    }
    else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_S)
    {
        CHASSIS.vx_set = -NORMAL_MAX_CHASSIS_SPEED_X;
    }

    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_A)
    {
        CHASSIS.vy_set = NORMAL_MAX_CHASSIS_SPEED_Y;
    }
    else if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_D)
    {
        CHASSIS.vy_set = -NORMAL_MAX_CHASSIS_SPEED_Y;
    }
    /*int spinflag = 0; //标定小陀螺状态
    if (CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_R)//按R键启用小陀螺
    {
        spinflag = 1;
    }
    else if(CHASSIS.rc->key.v & KEY_PRESSED_OFFSET_F)//按F键关闭小陀螺
    {
        spinflag = 0;
    }
    if (spinflag){
        CHASSIS.wz_set = NORMAL_MAX_CHASSIS_SPEED_WX;
    }*/
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    uint8_t i;
    
    //麦轮解算
    CHASSIS.wheel_motor[0].set.vel = -CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[1].set.vel =  CHASSIS.vx_set + CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[2].set.vel =  CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;
    CHASSIS.wheel_motor[3].set.vel = -CHASSIS.vx_set - CHASSIS.vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * CHASSIS.wz_set;

    //pid速度计算            
    for (i = 0; i < 4; i++)
    {
        CHASSIS.wheel_motor[i].set.curr =
        PID_calc(&CHASSIS.motor_speed_pid[i], CHASSIS.wheel_motor[i].fdb.vel, CHASSIS.wheel_motor[i].set.vel);
    }

}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */

void ChassisSendCmd(void)
{
    if (toe_is_error(DBUS_TOE))
  {
    CanCmdDjiMotor(1,0x200,0,0,0,0);             
  }
  else
  {
    CanCmdDjiMotor(1, 0x200, 
    CHASSIS.wheel_motor[0].set.curr, CHASSIS.wheel_motor[1].set.curr,
    CHASSIS.wheel_motor[2].set.curr, CHASSIS.wheel_motor[3].set.curr);

  }

    
}

#endif //CHASSIS_MECANUM_WHEEL

