#ifndef CHASSIS_BALANCE_EXTRAS_H
#define CHASSIS_BALANCE_EXTRAS_H

#include "robot_param.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)

extern void GetK(float l, float k[2][6]);

extern void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2]);

extern void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2]);

extern void GetLegForce(float J[2][2], float T1, float T2, float F[2]);

extern void CalcJacobian(float phi1, float phi4, float J[2][2]);

extern void CalcVmc(float F0, float Tp, float J[2][2], float T[2]);

extern void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2]);

#endif  /* CHASSIS_BALANCE */
#endif  // CHASSIS_BALANCE_EXTRAS_H
