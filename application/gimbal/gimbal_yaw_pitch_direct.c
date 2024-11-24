/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.1.0     2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *  V1.1.1     2024-11-11    Harry_Wong        1.为云台随动添加了yaw轴偏转角度的API
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/

#include "gimbal_yaw_pitch_direct.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#include "CAN_receive.h"
#include "AHRS_middleware.h"
Gimbal_s gimbal_direct;
PID_t gimbal_direct_pid;


/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------angle_solution--------------------*/

/**
 * @brief          解算imu和电机角度的差值（用于更新云台的限位范围）,确认新的电机中值对应的imu值
 * @param[in]      none
 * @retval         none
 */

void Angle_solution(void)
{
  float motor_feedback=gimbal_direct.pitch.fdb.pos,imu_feedback=gimbal_direct.imu->pitch,motor_mid=GIMBAL_DIRECT_PITCH_MID,imu_mid=0.0;
  float motor_delta=GIMBAL_DIRECT_PITCH_DIRECTION*(motor_feedback-motor_mid),imu_delta=imu_feedback-imu_mid;
  gimbal_direct.angle_zero_for_imu=imu_delta-motor_delta;
}




/*-------------------------The end of internal functions--------------------------------------*/

/*----------------GetGimbalDeltaYawMid--------------------*/

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float
 */

float GetGimbalDeltaYawMid(void)
{
  return loop_fp32_constrain(gimbal_direct.yaw.fdb.pos-GIMBAL_DIRECT_YAW_MID,-PI,PI);
}


/*----------------Gimbal_direct_init_judge--------------------*/

/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */

bool Gimbal_direct_init_judge (void)
{
  if (( ((gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos) && (gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos<0.003f && (-0.003f)<gimbal_direct.reference.pitch-gimbal_direct.pitch.fdb.pos) ) && gimbal_direct.init_timer<=1000 )|| gimbal_direct.last_mode==GIMBAL_ZERO_FORCE)
  {
    return true;
  }
  else
  {
    return false;
  }
}


/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void) 
{
  //step1 获取所有所需变量指针
   gimbal_direct.rc = get_remote_control_point(); 
   gimbal_direct.imu= Subscribe("imu_data");
   //step2 置零所有值
   gimbal_direct.reference.pitch=0;
   gimbal_direct.reference.yaw=0;

   gimbal_direct.feedback.pitch=0;
   gimbal_direct.feedback.yaw=0;

   gimbal_direct.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;

   gimbal_direct.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;
   //step3 PID数据清零，设置PID参数
   const static fp32 gimbal_yaw_angle[3]={KP_GIMBAL_YAW_ANGLE,KI_GIMBAL_YAW_ANGLE,KD_GIMBAL_YAW_ANGLE};
   const static fp32 gimbal_yaw_velocity[3]={KP_GIMBAL_YAW_VELOCITY,KI_GIMBAL_YAW_VELOCITY,KD_GIMBAL_YAW_VELOCITY};
   
		const static fp32 gimbal_pitch_angle[3]={KP_GIMBAL_PITCH_ANGLE,KI_GIMBAL_PITCH_ANGLE,KD_GIMBAL_PITCH_ANGLE};
   const static fp32 gimbal_pitch_velocity[3]={KP_GIMBAL_PITCH_VELOCITY,KI_GIMBAL_PITCH_VELOCITY,KD_GIMBAL_PITCH_VELOCITY};

   PID_init(&gimbal_direct_pid.yaw_angle,PID_POSITION,gimbal_yaw_angle,MAX_OUT_GIMBAL_YAW_ANGLE,MAX_IOUT_GIMBAL_YAW_ANGLE);
   PID_init(&gimbal_direct_pid.yaw_velocity,PID_POSITION,gimbal_yaw_velocity,MAX_OUT_GIMBAL_YAW_VELOCITY,MAX_IOUT_GIMBAL_YAW_VELOCITY);

   PID_init(&gimbal_direct_pid.pitch_angle,PID_POSITION,gimbal_pitch_angle,MAX_OUT_GIMBAL_PITCH_ANGLE,MAX_IOUT_GIMBAL_PITCH_ANGLE);
   PID_init(&gimbal_direct_pid.pitch_velocity,PID_POSITION,gimbal_pitch_velocity,MAX_OUT_GIMBAL_PITCH_VELOCITY,MAX_IOUT_GIMBAL_PITCH_VELOCITY);

   //step4 初始化电机
   MotorInit(&gimbal_direct.yaw,GIMBAL_DIRECT_YAW_ID,GIMBAL_DIRECT_YAW_CAN,GIMBAL_DIRECT_YAW_MOTOR_TYPE,GIMBAL_DIRECT_YAW_DIRECTION,GIMBAL_DIRECT_YAW_REDUCTION_RATIO,GIMBAL_DIRECT_YAW_MODE);
   MotorInit(&gimbal_direct.pitch,GIMBAL_DIRECT_PITCH_ID,GIMBAL_DIRECT_PITCH_CAN,GIMBAL_DIRECT_PITCH_MOTOR_TYPE,GIMBAL_DIRECT_PITCH_DIRECTION,GIMBAL_DIRECT_PITCH_REDUCTION_RATIO,GIMBAL_DIRECT_PITCH_MODE);

   //step5 初始化云台初始化校准相关变量
   gimbal_direct.init_start_time=0;
   gimbal_direct.init_timer=0;

   //step6 设置初始模式
   gimbal_direct.mode=GIMBAL_ZERO_FORCE;

}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
  //下档无力
  if ((switch_is_down(gimbal_direct.rc->rc.s[0])) || toe_is_error(DBUS_TOE)) //安全档优先级最高
  {
    gimbal_direct.mode=GIMBAL_ZERO_FORCE;
  }
  //初始校准模式
  else if (gimbal_direct.mode==GIMBAL_ZERO_FORCE || gimbal_direct.mode==GIMBAL_INIT)  
  {
    gimbal_direct.mode=GIMBAL_INIT;
    if (Gimbal_direct_init_judge())//判断是否需要跳出循环
    {
      gimbal_direct.mode=GIMBAL_IMU;
    }
  }
  //上，中档陀螺仪控制
  else 
  {
    gimbal_direct.mode=GIMBAL_IMU;
  }
}
/*-------------------- Observe --------------------*/
 
/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
  GetMotorMeasure(&gimbal_direct.yaw);
  GetMotorMeasure(&gimbal_direct.pitch);

  gimbal_direct.feedback.pitch=gimbal_direct.imu->pitch;
  gimbal_direct.feedback.yaw=gimbal_direct.imu->yaw;

  Angle_solution();


  if (gimbal_direct.mode==GIMBAL_INIT) //初始化校准模式时钟更新
  {
    if (gimbal_direct.last_mode==GIMBAL_ZERO_FORCE)
    {
      gimbal_direct.init_timer=xTaskGetTickCount();
    }

    gimbal_direct.init_timer=xTaskGetTickCount()-gimbal_direct.init_start_time;
  }

  gimbal_direct.last_mode=gimbal_direct.mode;


}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
  if (gimbal_direct.mode==GIMBAL_INIT)
  {
    gimbal_direct.reference.pitch=GIMBAL_DIRECT_PITCH_MID-0.2f;
    gimbal_direct.reference.yaw=GIMBAL_DIRECT_YAW_MID;
  }
  if (gimbal_direct.mode==GIMBAL_IMU)
  {
    if (gimbal_direct.last_mode==GIMBAL_INIT)
    {
      gimbal_direct.reference.pitch=gimbal_direct.feedback.pitch;
      gimbal_direct.reference.yaw=gimbal_direct.feedback.yaw;
      gimbal_direct.init_timer=0;
    }
    else 
    {
      // warning :不建议键鼠跟遥控器同时使用！
      //读取鼠标的移动（还未测试过鼠标）
	    //暂时先屏蔽一下鼠标功能
      //gimbal_direct.reference.pitch=fp32_constrain(gimbal_direct.reference.pitch+gimbal_direct.rc->mouse.y*MOUSE_SENSITIVITY,GIMBAL_LOWER_LIMIT_PITCH,GIMBAL_UPPER_LIMIT_PITCH);
      //gimbal_direct.reference.yaw  =loop_fp32_constrain(gimbal_direct.reference.yaw+gimbal_direct.rc->mouse.x*MOUSE_SENSITIVITY,-PI,PI);

      //读取摇杆的数据
      gimbal_direct.reference.pitch= fp32_constrain(gimbal_direct.reference.pitch-(float)gimbal_direct.rc->rc.ch[1]/REMOTE_CONTROLLER_SENSITIVITY,GIMBAL_LOWER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu,GIMBAL_UPPER_LIMIT_PITCH+gimbal_direct.angle_zero_for_imu);
      gimbal_direct.reference.yaw = loop_fp32_constrain(gimbal_direct.reference.yaw-(float)gimbal_direct.rc->rc.ch[0]/REMOTE_CONTROLLER_SENSITIVITY,-PI,PI);
    }
    
  }
  

  
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal_direct.mode == GIMBAL_ZERO_FORCE)
  {
    gimbal_direct.pitch.set.curr=0;
    gimbal_direct.yaw.set.curr=0;
  }
  else if (gimbal_direct.mode == GIMBAL_IMU)
  {
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.feedback.pitch,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.curr=PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.imu->pitch_vel,gimbal_direct.pitch.set.vel);

    fp32 delta_yaw=loop_fp32_constrain(gimbal_direct.reference.yaw-gimbal_direct.feedback.yaw,-PI,PI);
    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle,0,delta_yaw);
    gimbal_direct.yaw.set.curr=PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.imu->yaw_vel,gimbal_direct.yaw.set.vel);
  }
  else if (gimbal_direct.mode == GIMBAL_INIT)
  {
    gimbal_direct.pitch.set.vel=PID_calc(&gimbal_direct_pid.pitch_angle,gimbal_direct.pitch.fdb.pos,gimbal_direct.reference.pitch);
    gimbal_direct.pitch.set.curr=PID_calc(&gimbal_direct_pid.pitch_velocity,gimbal_direct.pitch.fdb.vel,gimbal_direct.pitch.set.vel);

    fp32 delta_yaw=loop_fp32_constrain(gimbal_direct.reference.yaw-gimbal_direct.yaw.fdb.pos,-PI,PI);
    gimbal_direct.yaw.set.vel=PID_calc(&gimbal_direct_pid.yaw_angle,0,delta_yaw);
    gimbal_direct.yaw.set.curr=PID_calc(&gimbal_direct_pid.yaw_velocity,gimbal_direct.yaw.fdb.vel,gimbal_direct.yaw.set.vel);
  }
 
}
  

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void) 
{
    CanCmdDjiMotor(2,0x1FF,gimbal_direct.yaw.set.curr,gimbal_direct.pitch.set.curr,0,0);

    ModifyDebugDataPackage(1,gimbal_direct.init_timer,"init_T");
    ModifyDebugDataPackage(2,gimbal_direct.mode==GIMBAL_INIT,"init");
    ModifyDebugDataPackage(3,gimbal_direct.mode==GIMBAL_IMU,"imu");
    ModifyDebugDataPackage(4,gimbal_direct.mode==GIMBAL_ZERO_FORCE,"safe");

}



#endif  // GIMBAL_YAW_PITCH
