
#ifndef DART_PARAM_H
#define DART_PARAM_H


// 飞镖电机参数
#define DART_CHASSIS_ID 1            // 飞镖底盘电机ID
#define MOTOR_DART_CAN 1             //飞镖电机CAN总线
#define DART_CHASSIS_TYPE DJI_M6020  // 底盘飞镖电机类型
#define MOTOR_DART_DIRECTION 1     // 底盘飞镖电机方向
#define MOTOR_DART_REDUCTION 1.0f  // 底盘飞镖电机减速比
#define MOTOR_DART_MODE 0          // 底盘飞镖电机模式

// 任务相关参数
#define DART_TASK_INIT_TIME 200     // 飞镖任务初始化时间(ms)
#define DART_CONTROL_TIME_MS 2      // 飞镖控制周期(ms)
#define DART_CONTROL_TIME 1
//PID
//飞镖电机PID
#define DART_SPEED_PID_KP (200.0f)
#define DART_SPEED_PID_KI (1.0f)
#define DART_SPEED_PID_KD (1.0f)

#define DART_PID_MAX_OUT (2000.0f)
#define DART_PID_MAX_IOUT (200.0f)

//飞镖速度限制
#define DART_SPEED (0.8f)
//飞镖电机标准ID
#define DART_STD_ID (0x1FF)


#endif /* DART_PARAM_H */
