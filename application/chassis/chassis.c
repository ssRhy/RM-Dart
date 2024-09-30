/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis.c/h
  * @brief      底盘控制任务所需要的变量和函数
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "chassis.h"

#if CHASSIS_TYPE != CHASSIS_NONE

#include <math.h>
#include <stdlib.h>

#include "chassis_balance.h"
#include "chassis_mecanum.h"
#include "chassis_omni.h"
#include "chassis_steering.h"

// ChassisApi_t chassis = {
//     .SetCali = SetCali,
//     .CmdCali = CmdCali,
//     .GetStatus = GetStatus,
//     .GetDuration = GetDuration,
//     .GetSpeedVx = GetSpeedVx,
//     .GetSpeedVy = GetSpeedVy,
//     .GetSpeedWz = GetSpeedWz,
// };

ChassisApi_t chassis = {
    .SetCali = NULL,
    .CmdCali = NULL,
    .GetStatus = NULL,
    .GetDuration = NULL,
    .GetSpeedVx = NULL,
    .GetSpeedVy = NULL,
    .GetSpeedWz = NULL,
};

/**
 * @brief          将速度向量从云台坐标系下转换到底盘坐标系下
 * @param          speed_vector_set 需要变换的速度向量
 * @param[in]      dyaw (rad)云台到底盘旋转的角度
 */
void GimbalSpeedVectorToChassisSpeedVector(ChassisSpeedVector_t * speed_vector_set, float dyaw)
{
    float vx_chassis = speed_vector_set->vx * cosf(dyaw) + speed_vector_set->vy * sinf(dyaw);
    float vy_chassis = -speed_vector_set->vx * sinf(dyaw) + speed_vector_set->vy * cosf(dyaw);
    speed_vector_set->vx = vx_chassis;
    speed_vector_set->vy = vy_chassis;
}
#endif  // CHASSIS_TYPE
/*------------------------------ End of File ------------------------------*/
