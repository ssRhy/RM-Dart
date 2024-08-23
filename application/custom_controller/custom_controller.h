
#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#define CUSTOM_CONTROLLER_TASK_INIT_TIME 100
#define CUSTOM_CONTROLLER_CONTROL_TIME 1

typedef struct 
{
    void (*Blank)(void);
}CustomControllerApi_t;

extern CustomControllerApi_t custom_controller;

#endif /* CUSTOM_CONTROLLER_H */
/*------------------------------ End of File ------------------------------*/
