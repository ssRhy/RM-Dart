#include "chassis_balance_extras.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "math.h"

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK(float l, float k[2][6]) {}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] l0_phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float l0_phi0[2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, phi2, XC, YC;
    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);

    l0_phi0[0] = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    l0_phi0[1] = atan2(YC, (XC - LEG_L5 / 2));
}

/**
 * @brief 通过L0和Phi0的值获取关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 */
void GetPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2]) {}

/**
 * @brief 获取dPhi0
 * @param[in]  phi_1 
 * @param[in]  phi_4 
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
float GetdPhi0(float phi_1, float phi_4, float d_phi1, float d_phi4) {}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  phi1 
 * @param[in]  phi4 
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float phi1, float phi4, float T[2]) {}

#endif /* CHASSIS_BALANCE */
