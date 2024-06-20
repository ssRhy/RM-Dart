#include "chassis_balance_extras.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "math.h"

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK(float l, float k[2][6])
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;
    k[0][0] = -192.0029 * t3 + 238.6046 * t2 - 121.5351 * t1 + 2.5930;
    k[0][1] = 3.0259 * t3 + 0.7416 * t2 - 7.9140 * t1 + 0.3474;
    k[0][2] = -39.2205 * t3 + 40.4062 * t2 - 14.6241 * t1 + 0.5277;
    k[0][3] = -55.1536 * t3 + 57.1891 * t2 - 21.2831 * t1 + 0.7201;
    k[0][4] = 30.5199 * t3 - 6.1050 * t2 - 14.7455 * t1 + 7.9561;
    k[0][5] = 8.5111 * t3 - 5.6617 * t2 - 0.0838 * t1 + 0.9196;
    k[1][0] = 583.8820 * t3 - 510.3673 * t2 + 121.3082 * t1 + 9.8385;
    k[1][1] = 53.0943 * t3 - 53.3170 * t2 + 17.0853 * t1 + 0.0306;
    k[1][2] = -0.9070 * t3 + 18.1948 * t2 - 17.6611 * t1 + 5.7359;
    k[1][3] = 2.1872 * t3 + 22.0835 * t2 - 23.7428 * t1 + 8.0919;
    k[1][4] = 501.4394 * t3 - 529.0736 * t2 + 198.8558 * t1 - 11.2287;
    k[1][5] = 54.4287 * t3 - 59.6999 * t2 + 23.7611 * t1 - 1.9947;
}

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
 * @brief 获取dPhi0
 * @param[in]  phi_1 
 * @param[in]  phi_4 
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdPhi0AnddL0(float J[2][2], float d_phi1, float d_phi4, float dPhi0_dL0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dPhi0_dL0[0] = d_phi0;
    dPhi0_dL0[1] = d_l0;
}

/**
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
    float phi2, phi3;
    float L0, phi0;
    float j11, j12, j21, j22;

    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    phi0 = atan2(YC, XC - LEG_L5 / 2);

    j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));

    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  phi1 
 * @param[in]  phi4 
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] + JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] + JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2]) {}

#endif /* CHASSIS_BALANCE */
