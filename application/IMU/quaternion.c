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

#include "quaternion.h"
#include <stdio.h>
#include <math.h>

// 将旋转矩阵转换为四元数
Quaternion matrix_to_quaternion(RotationMatrix mat) {
    Quaternion q;
    float trace = mat[0][0] + mat[1][1] + mat[2][2];
    
    if (trace > 0) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        q.w = 0.25f / s;
        q.x = (mat[2][1] - mat[1][2]) * s;
        q.y = (mat[0][2] - mat[2][0]) * s;
        q.z = (mat[1][0] - mat[0][1]) * s;
    } else {
        if (mat[0][0] > mat[1][1] && mat[0][0] > mat[2][2]) {
            float s = 2.0f * sqrtf(1.0f + mat[0][0] - mat[1][1] - mat[2][2]);
            q.w = (mat[2][1] - mat[1][2]) / s;
            q.x = 0.25f * s;
            q.y = (mat[0][1] + mat[1][0]) / s;
            q.z = (mat[0][2] + mat[2][0]) / s;
        } else if (mat[1][1] > mat[2][2]) {
            float s = 2.0f * sqrtf(1.0f + mat[1][1] - mat[0][0] - mat[2][2]);
            q.w = (mat[0][2] - mat[2][0]) / s;
            q.x = (mat[0][1] + mat[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (mat[1][2] + mat[2][1]) / s;
        } else {
            float s = 2.0f * sqrtf(1.0f + mat[2][2] - mat[0][0] - mat[1][1]);
            q.w = (mat[1][0] - mat[0][1]) / s;
            q.x = (mat[0][2] + mat[2][0]) / s;
            q.y = (mat[1][2] + mat[2][1]) / s;
            q.z = 0.25f * s;
        }
    }
    
    // 归一化保证单位四元数
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

// 四元数乘法 (q1 * q2)
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2) {
    Quaternion result;
    result.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    result.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    result.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    result.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return result;
}

// 应用旋转矩阵到四元数姿态
Quaternion apply_rotation(Quaternion original, RotationMatrix matrix) {
    // 将旋转矩阵转换为四元数
    Quaternion Rq = matrix_to_quaternion(matrix);
    
    // 执行四元数乘法
    Quaternion new_pose = quaternion_multiply(Rq, original);
    
    // 可选：再次归一化
    float norm = sqrtf(new_pose.w*new_pose.w + new_pose.x*new_pose.x + 
                      new_pose.y*new_pose.y + new_pose.z*new_pose.z);
    new_pose.w /= norm;
    new_pose.x /= norm;
    new_pose.y /= norm;
    new_pose.z /= norm;
    
    return new_pose;
}

/*------------------------------ End of File ------------------------------*/
