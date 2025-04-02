#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "remote_control.h"
#include "stdbool.h"
#include "struct_typedef.h"

extern void Usart1Init(void);

extern void Uart2TaskLoop(void);

// API

extern bool GetUartOffline(void);
extern bool GetUartRcToeError(void);
extern const RC_ctrl_t * GetUartRcPoint(void);
extern float GetUartGimbalYawMotorPos(void);
extern bool GetUartGimbalInitJudge(void);
extern uint32_t GetUartTimeStampForTest(void);

#endif  // __COMMUNICATION_H
/*------------------------------ End of File ------------------------------*/
