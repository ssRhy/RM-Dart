/**
 ******************************************************************************
 * @file    IMU_solve.c/h
 * @brief   6轴陀螺仪解算部分，通过EKF算法滤除噪声，得到姿态角。
 *          解算得到 姿态角、角速度、加速度
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2025-04-05      Penguin         1. done
 ******************************************************************************
 * @attention
 * 1st order LPF transfer function:
 *     1
 *  ———————
 *  as + 1
 ******************************************************************************
 @verbatim
 ==============================================================================
 详细原理参考王工的文章：
    四元数EKF姿态更新算法 https://zhuanlan.zhihu.com/p/454155643

 ==============================================================================
 @endverbatim
*/

#ifndef __IMU_SOLVE_H
#define __IMU_SOLVE_H

extern float gVec[3];

void IMU_QuaternionEKF_Init(float process_noise1, float process_noise2, float measure_noise, float lambda);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void gEstimateKF_Init(float process_noise, float measure_noise);
void gEstimateKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

extern float GetEkfYaw(void);
extern float GetEkfPitch(void);
extern float GetEkfRoll(void);

extern float GetEkfAccel(int i);

#endif /* __IMU_SOLVE_H */
/*------------------------------ End of File ------------------------------*/
