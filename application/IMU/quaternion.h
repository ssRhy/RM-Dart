/**
 ******************************************************************************
 * @file    quaternion.c/h
 * @brief   四元数处理
 * @note    仅在IMU解算任务中使用，不对其他模块外提供接口
 *          禁止在其他模块中导入该文件
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

 ==============================================================================
 @endverbatim
*/

#ifndef __QUATERNION_H
#define __QUATERNION_H
// 四元数结构体
typedef struct
{
    float w, x, y, z;
} Quaternion;

// 旋转矩阵类型（3x3数组）
typedef float RotationMatrix[3][3];

Quaternion matrix_to_quaternion(RotationMatrix mat);
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

#endif  // __QUATERNION_H
/*------------------------------ End of File ------------------------------*/
