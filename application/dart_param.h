
#ifndef DART_PARAM_H
#define DART_PARAM_H


// 飞镖电机总参数
#define MOTOR_DART_CAN 1             //飞镖电机CAN总线
#define MOTOR_DART_DIRECTION 1     // 底盘飞镖电机方向
#define MOTOR_DART_REDUCTION 1.0f  // 底盘飞镖电机减速比
#define MOTOR_DART_MODE 0          // 底盘飞镖电机模式

// 任务相关参数
#define DART_TASK_INIT_TIME 200     // 飞镖任务初始化时间(ms)
#define DART_CONTROL_TIME_MS 2      // 飞镖控制周期(ms)
#define DART_CONTROL_TIME 1

#define CHASSIS_TYPE CHASSIS_DART                   //选择飞镖底盘类型
#define SHOOT_TYPE SHOOT_DART_FRIC              //     选择飞镖发射类型
#define DART_FEED_TYPE DART_FEED               //     选择飞镖供弹类型
#define DART_TRANS_TYPE DART_TRANS               //     选择飞镖传输类型

#define DART_CHASSIS_MOTOR_TYPE DJI_M6020  // 底盘飞镖电机类型
#define DART_FEED_MOTOR_TYPE DJI_M3508    // 供弹飞镖电机类型
#define DART_TRANS_MOTOR_TYPE DJI_M2006    // 传输飞镖电机类型
#define DART_SHOOT_MOTOR_TYPE DJI_M3508   // 发射飞镖电机类型

#define DART_FEED_MOTOR_ID 1                 // 飞镖供弹电机ID
#define DART_TRANS_MOTOR_ID 2               // 飞镖传送电机ID
#define DART_CHASSIS_MOTOR_ID 1            // 飞镖底盘电机ID

//飞镖电机标准ID
#define DART_CHASSIS_STD_ID ((uint8_t)0x1FF) //底盘电机ID
#define DART_FEED_STD_ID    ((uint8_t)0x200)//供弹电机ID
#define DART_TRANS_STD_ID   ((uint8_t)0x1FF)//传送电机ID

//飞镖速度限制
#define DART_SPEED (0.8f)//底盘飞镖速度
#define DART_FEED_SPEED (1.0f)//供弹飞镖速度
#define DART_TRANS_SPEED (1.0f)//传送飞镖速度


//PID
//飞镖底盘电机PID
#define DART_SPEED_PID_KP (200.0f)
#define DART_SPEED_PID_KI (1.0f)
#define DART_SPEED_PID_KD (1.0f)
#define DART_PID_MAX_OUT (2000.0f)
#define DART_PID_MAX_IOUT (200.0f)

//供弹电机PID
#define DART_FEED_SPEED_PID_KP (200.0f)
#define DART_FEED_SPEED_PID_KI (1.0f)
#define DART_FEED_SPEED_PID_KD (1.0f)
#define DART_FEED_PID_MAX_OUT (2000.0f)
#define DART_FEED_PID_MAX_IOUT (200.0f)
//传送电机PID
#define DART_TRANS_SPEED_PID_KP (200.0f)
#define DART_TRANS_SPEED_PID_KI (1.0f)
#define DART_TRANS_SPEED_PID_KD (1.0f)
#define DART_TRANS_PID_MAX_OUT (2000.0f)
#define DART_TRANS_PID_MAX_IOUT (200.0f)

//---------------------------------------------特殊发射机构-------------------------------------//
//摩擦轮飞镖发射机构参数
#define DART_SHOOT_MOTOR_0_ID 1
#define DART_SHOOT_MOTOR_1_ID 2
#define DART_SHOOT_MOTOR_2_ID 3
#define DART_SHOOT_MOTOR_3_ID 4
#define DART_SHOOT_MOTOR_4_ID 5 
#define DART_SHOOT_MOTOR_5_ID 6

#define DART_SHOOT_MOTOR_LEFT_CAN 1
#define DART_SHOOT_MOTOR_RIGHT_CAN 1

#define DART_SHOOT_PID_KP 300.0f
#define DART_SHOOT_PID_KI 0.0f
#define DART_SHOOT_PID_KD 5.0f
#define DART_SHOOT_PID_MAX_OUT 2000.0f
#define DART_SHOOT_PID_MAX_IOUT 200.0f

  

// #ifndef DART_TYPE
// #define DART_TYPE DART_NONE
// #endif

// #ifndef DART_SHOOT_TYPE
// #define DART_SHOOT_TYPE DART_NONE
// #endif
// #ifndef DART_FEED_TYPE
// #define DART_FEED_TYPE DART_NONE
// #endif
// #ifndef DART_TRANS_TYPE
// #define DART_TRANS_TYPE DART_NONE
// #endif

#endif /* DART_PARAM_H */
