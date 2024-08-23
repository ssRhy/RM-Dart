
#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include "struct_typedef.h"

#define CUSTOM_CONTROLLER_TASK_INIT_TIME 100
#define CUSTOM_CONTROLLER_CONTROL_TIME 1

typedef struct 
{
    void (*Blank)(void);
}CustomControllerApi_t;

typedef struct 
{
    float pos[7];
    uint16_t reserved;
}CCControlData_t;

extern CustomControllerApi_t custom_controller;

#endif /* CUSTOM_CONTROLLER_H */
/*------------------------------ End of File ------------------------------*/
