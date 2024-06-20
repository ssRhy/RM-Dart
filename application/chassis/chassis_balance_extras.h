#ifndef CHASSIS_BALANCE_EXTRAS_H
#define CHASSIS_BALANCE_EXTRAS_H

#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)

extern void GetK(float l, float k[2][6]);

extern void GetL0AndPhi0(float phi1, float phi4, float l0_phi0[2]);

extern void GetPhi1AndPhi4(float phi0,float l0,float phi1_phi4[2]);

extern float GetdPhi0(float phi_1, float phi_4, float d_phi1, float d_phi4);

extern void CalcVmc(float F0, float Tp, float phi1, float phi4, float T[2]);

#endif  /* CHASSIS_BALANCE */
#endif  // CHASSIS_BALANCE_EXTRAS_H
