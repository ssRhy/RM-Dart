/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       chassis_balance.c/h
  * @brief      平衡底盘控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.0.2     Sep-16-2024     Penguin         1. 添加速度观测器并测试效果
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  
  @todo:
    2.添加状态清零，当运行过程中出现异常时可以手动将底盘状态清零
    3.在浮空时通过动量守恒维持底盘的平衡，并调整合适的触地姿态

  ****************************(C) COPYRIGHT 2024 Polarbear****************************
*/
#include "chassis_balance.h"
#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "CAN_communication.h"
#include "bsp_delay.h"
#include "chassis.h"
#include "chassis_balance_extras.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "detect_task.h"
#include "kalman_filter.h"
#include "macro_typedef.h"
#include "signal_generator.h"
#include "stdbool.h"
#include "string.h"
#include "usb_debug.h"
#include "user_lib.h"

#define CALIBRATE_STOP_VELOCITY 0.05f  // rad/s
#define CALIBRATE_STOP_TIME 200        // ms
#define CALIBRATE_VELOCITY 2.0f        // rad/s

#define VEL_PROCESS_NOISE 25   // 速度过程噪声
#define VEL_MEASURE_NOISE 800  // 速度测量噪声
// 同时估计加速度和速度时对加速度的噪声
// 更好的方法是设置为动态,当有冲击时/加加速度大时更相信轮速
#define ACC_PROCESS_NOISE 2000  // 加速度过程噪声
#define ACC_MEASURE_NOISE 0.01  // 加速度测量噪声

#define RC_OFF_HOOK_VALUE_HOLE 650

static Calibrate_s CALIBRATE = {
    .cali_cnt = 0,
    .velocity = {0.0f, 0.0f, 0.0f, 0.0f},
    .stpo_time = {0, 0, 0, 0},
    .reached = {false, false, false, false},
    .calibrated = false,
};

static GroundTouch_s GROUND_TOUCH = {
    .touch_time = 0,
    .touch = false,
};

static Observer_t OBSERVER;

Chassis_s CHASSIS = {
    .mode = CHASSIS_OFF,
    .error_code = 0,
    .yaw_mid = 0,

    .ratio =
        {
            // clang-format off
            .k = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}, 
                  {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
            // clang-format on
            .Tp = TP_RATIO,
            .T = T_RATIO,
            .length = 1.0f,
        },
    .dyaw = 0.0f,
};

/*-------------------- Publish --------------------*/

void ChassisPublish(void) { Publish(&CHASSIS.fdb.speed_vector, CHASSIS_FDB_SPEED_NAME); }

/******************************************************************/
/* Init                                                           */
/*----------------------------------------------------------------*/
/* main function:      ChassisInit                                */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ChassisInit(void)
{
    CHASSIS.rc = get_remote_control_point();  // 获取遥控器指针
    CHASSIS.imu = Subscribe(IMU_NAME);        // 获取IMU数据指针
    /*-------------------- 初始化底盘电机 --------------------*/
    MotorInit(&CHASSIS.joint_motor[0], 1, JOINT_CAN, DM_8009, J0_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[1], 2, JOINT_CAN, DM_8009, J1_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[2], 3, JOINT_CAN, DM_8009, J2_DIRECTION, 1, DM_MODE_MIT);
    MotorInit(&CHASSIS.joint_motor[3], 4, JOINT_CAN, DM_8009, J3_DIRECTION, 1, DM_MODE_MIT);

    MotorInit(&CHASSIS.wheel_motor[0], 1, WHEEL_CAN, MF_9025, W0_DIRECTION, 1, 0);
    MotorInit(&CHASSIS.wheel_motor[1], 2, WHEEL_CAN, MF_9025, W1_DIRECTION, 1, 0);

    /*-------------------- 值归零 --------------------*/
    memset(&CHASSIS.fdb, 0, sizeof(CHASSIS.fdb));
    memset(&CHASSIS.ref, 0, sizeof(CHASSIS.ref));

    /*-------------------- 初始化底盘PID --------------------*/
    // yaw轴跟踪pid
    float yaw_angle_pid[3] = {KP_CHASSIS_YAW_ANGLE, KI_CHASSIS_YAW_ANGLE, KD_CHASSIS_YAW_ANGLE};
    float yaw_velocity_pid[3] = {
        KP_CHASSIS_YAW_VELOCITY, KI_CHASSIS_YAW_VELOCITY, KD_CHASSIS_YAW_VELOCITY};
    PID_init(
        &CHASSIS.pid.yaw_angle, PID_POSITION, yaw_angle_pid, MAX_OUT_CHASSIS_YAW_ANGLE,
        MAX_IOUT_CHASSIS_YAW_ANGLE);
    PID_init(
        &CHASSIS.pid.yaw_velocity, PID_POSITION, yaw_velocity_pid, MAX_OUT_CHASSIS_YAW_VELOCITY,
        MAX_IOUT_CHASSIS_YAW_VELOCITY);

    // 速度增量pid
    float vel_add_pid[3] = {KP_CHASSIS_VEL_ADD, KI_CHASSIS_VEL_ADD, KD_CHASSIS_VEL_ADD};
    PID_init(
        &CHASSIS.pid.vel_add, PID_POSITION, vel_add_pid, MAX_OUT_CHASSIS_VEL_ADD,
        MAX_IOUT_CHASSIS_VEL_ADD);

#if LOCATION_CONTROL

    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    float pitch_angle_pid[3] = {
        KP_CHASSIS_PITCH_ANGLE, KI_CHASSIS_PITCH_ANGLE, KD_CHASSIS_PITCH_ANGLE};
    float pitch_vel_pid[3] = {
        KP_CHASSIS_PITCH_VELOCITY, KI_CHASSIS_PITCH_VELOCITY, KD_CHASSIS_PITCH_VELOCITY};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);

    PID_init(
        &CHASSIS.pid.pitch_angle, PID_POSITION, pitch_angle_pid, MAX_OUT_CHASSIS_PITCH_ANGLE,
        MAX_IOUT_CHASSIS_PITCH_ANGLE);
    PID_init(
        &CHASSIS.pid.pitch_vel, PID_POSITION, pitch_vel_pid, MAX_OUT_CHASSIS_PITCH_VELOCITY,
        MAX_IOUT_CHASSIS_PITCH_VELOCITY);
#else
    float roll_angle_pid[3] = {KP_CHASSIS_ROLL_ANGLE, KI_CHASSIS_ROLL_ANGLE, KD_CHASSIS_ROLL_ANGLE};
    // float roll_velocity_pid[3] = {
    //     KP_CHASSIS_ROLL_VELOCITY, KI_CHASSIS_ROLL_VELOCITY, KD_CHASSIS_ROLL_VELOCITY};

    float leg_length_length_pid[3] = {
        KP_CHASSIS_LEG_LENGTH_LENGTH, KI_CHASSIS_LEG_LENGTH_LENGTH, KD_CHASSIS_LEG_LENGTH_LENGTH};

    float leg_angle_angle_pid[3] = {
        KP_CHASSIS_LEG_ANGLE_ANGLE, KI_CHASSIS_LEG_ANGLE_ANGLE, KD_CHASSIS_LEG_ANGLE_ANGLE};

    PID_init(
        &CHASSIS.pid.roll_angle, PID_POSITION, roll_angle_pid, MAX_OUT_CHASSIS_ROLL_ANGLE,
        MAX_IOUT_CHASSIS_ROLL_ANGLE);

    // PID_init(
    //     &CHASSIS.pid.roll_velocity, PID_POSITION, roll_velocity_pid, MAX_OUT_CHASSIS_ROLL_VELOCITY,
    //     MAX_IOUT_CHASSIS_ROLL_VELOCITY);

    PID_init(
        &CHASSIS.pid.leg_length_length[0], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);

    PID_init(
        &CHASSIS.pid.leg_length_length[1], PID_POSITION, leg_length_length_pid,
        MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH, MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH);

    PID_init(
        &CHASSIS.pid.leg_angle_angle, PID_POSITION, leg_angle_angle_pid,
        MAX_OUT_CHASSIS_LEG_ANGLE_ANGLE, MAX_IOUT_CHASSIS_LEG_ANGLE_ANGLE);
#endif

    float stand_up_pid[3] = {KP_CHASSIS_STAND_UP, KI_CHASSIS_STAND_UP, KD_CHASSIS_STAND_UP};
    PID_init(
        &CHASSIS.pid.stand_up, PID_POSITION, stand_up_pid, MAX_OUT_CHASSIS_STAND_UP,
        MAX_IOUT_CHASSIS_STAND_UP);

    float wheel_stop_pid[3] = {KP_CHASSIS_WHEEL_STOP, KI_CHASSIS_WHEEL_STOP, KD_CHASSIS_WHEEL_STOP};
    PID_init(
        &CHASSIS.pid.wheel_stop[0], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);
    PID_init(
        &CHASSIS.pid.wheel_stop[1], PID_POSITION, wheel_stop_pid, MAX_OUT_CHASSIS_WHEEL_STOP,
        MAX_IOUT_CHASSIS_WHEEL_STOP);

    // 初始化低通滤波器
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[0], LEG_DDL0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_l0_accel_filter[1], LEG_DDL0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[0], LEG_DDPHI0_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_phi0_accel_filter[1], LEG_DDPHI0_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[0], LEG_DDTHETA_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.leg_theta_accel_filter[1], LEG_DDTHETA_LPF_ALPHA);

    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[0], LEG_SUPPORT_FORCE_LPF_ALPHA);
    LowPassFilterInit(&CHASSIS.lpf.support_force_filter[1], LEG_SUPPORT_FORCE_LPF_ALPHA);

    // 初始化机体速度观测器
    // 使用kf同时估计速度和加速度
    Kalman_Filter_Init(&OBSERVER.body.v_kf, 2, 0, 2);
    float F[4] = {1, 0.005, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(OBSERVER.body.v_kf.F_data, F, sizeof(F));
    memcpy(OBSERVER.body.v_kf.P_data, P, sizeof(P));
    memcpy(OBSERVER.body.v_kf.Q_data, Q, sizeof(Q));
    memcpy(OBSERVER.body.v_kf.R_data, R, sizeof(R));
    memcpy(OBSERVER.body.v_kf.H_data, H, sizeof(H));
}

/******************************************************************/
/* Handle exception                                               */
/*----------------------------------------------------------------*/
/* main function:      ChassisHandleException                     */
/* auxiliary function: GroundTouchDectect                         */
/******************************************************************/

static void GroundTouchDectect(void);

/**
 * @brief          异常处理
 * @param[in]      none
 * @retval         none
 */
void ChassisHandleException(void)
{
    if (toe_is_error(DBUS_TOE)) {
        CHASSIS.error_code |= DBUS_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~DBUS_ERROR_OFFSET;
    }

    if (CHASSIS.imu == NULL) {
        CHASSIS.error_code |= IMU_ERROR_OFFSET;
    } else {
        CHASSIS.error_code &= ~IMU_ERROR_OFFSET;
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (fabs(CHASSIS.joint_motor[i].fdb.tor) > MAX_TORQUE_PROTECT) {
            CHASSIS.error_code |= JOINT_ERROR_OFFSET;
            break;
        }
    }

    if ((CHASSIS.mode == CHASSIS_OFF || CHASSIS.mode == CHASSIS_SAFE) &&
        fabs(CHASSIS.pid.stand_up.max_out) != 0.0f) {
        PID_clear(&CHASSIS.pid.stand_up);
    }

    GroundTouchDectect();  // 离地检测
}

/**
 * @brief 离地检测
 * @param  
 */
static void GroundTouchDectect(void)
{
    for (uint8_t i = 0; i < 2; i++) {
    }

    uint32_t now = HAL_GetTick();
    if (now - GROUND_TOUCH.touch_time < MAX_TOUCH_INTERVAL) {
        //若上次触地时间距离现在不超过 MAX_TOUCH_INTERVAL ms，则认为当前瞬间接地，避免弹跳导致误判
        GROUND_TOUCH.touch = true;
    } else {
        GROUND_TOUCH.touch = false;
    }
}

/******************************************************************/
/* Set mode                                                       */
/*----------------------------------------------------------------*/
/* main function:      ChassisSetMode                             */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ChassisSetMode(void)
{
    if (CHASSIS.error_code & DBUS_ERROR_OFFSET) {  // 遥控器出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & IMU_ERROR_OFFSET) {  // IMU出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.error_code & JOINT_ERROR_OFFSET) {  // 关节电机出错时的状态处理
        CHASSIS.mode = CHASSIS_SAFE;
        return;
    }

    if (CHASSIS.mode == CHASSIS_CALIBRATE && (!CALIBRATE.calibrated)) {  //校准完成后才退出校准
        return;
    }

    if (CALIBRATE.toggle) {  // 切入底盘校准
        CALIBRATE.toggle = false;
        CHASSIS.mode = CHASSIS_CALIBRATE;
        CALIBRATE.calibrated = false;

        uint32_t now = HAL_GetTick();
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.reached[i] = false;
            CALIBRATE.stpo_time[i] = now;
        }

        return;
    }

    if (switch_is_up(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        // CHASSIS.mode = CHASSIS_FREE;
        CHASSIS.mode = CHASSIS_CUSTOM;
    } else if (switch_is_mid(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        CHASSIS.mode = CHASSIS_DEBUG;
    } else if (switch_is_down(CHASSIS.rc->rc.s[CHASSIS_MODE_CHANNEL])) {
        // 在安全模式时，遥控器摇杆打成左下，右上进入脱困模式
        if (CHASSIS.rc->rc.ch[0] > RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[1] > RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[2] < -RC_OFF_HOOK_VALUE_HOLE &&
            CHASSIS.rc->rc.ch[3] < -RC_OFF_HOOK_VALUE_HOLE) {
            CHASSIS.mode = CHASSIS_OFF_HOOK;
        } else {
            CHASSIS.mode = CHASSIS_SAFE;
        }
    }
}

/******************************************************************/
/* Observe                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisObserver                            */
/* auxiliary function: UpdateBodyStatus                           */
/*                     UpdateLegStatus                            */
/*                     UpdateMotorStatus                          */
/*                     UpdateCalibrateStatus                      */
/*                     BodyMotionObserve                          */
/******************************************************************/

#define ZERO_POS_THRESHOLD 0.001f

static void UpdateBodyStatus(void);
static void UpdateLegStatus(void);
static void UpdateMotorStatus(void);
static void UpdateCalibrateStatus(void);

static void BodyMotionObserve(void);

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ChassisObserver(void)
{
    CHASSIS.duration = xTaskGetTickCount() - CHASSIS.last_time;
    CHASSIS.last_time = xTaskGetTickCount();

    UpdateMotorStatus();
    UpdateLegStatus();
    UpdateBodyStatus();
    UpdateCalibrateStatus();

    BodyMotionObserve();

    ModifyDebugDataPackage(2, CHASSIS.cmd.leg[0].rod.F, "Fl");
    ModifyDebugDataPackage(3, CHASSIS.cmd.leg[1].rod.F, "Fr");
    ModifyDebugDataPackage(4, CHASSIS.fdb.leg[0].rod.L0, "L0l");
    ModifyDebugDataPackage(5, CHASSIS.fdb.leg[1].rod.L0, "L0r");
}

/**
 * @brief  更新底盘电机数据
 * @param  none
 */
static void UpdateMotorStatus(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        GetMotorMeasure(&CHASSIS.joint_motor[i]);
    }

    for (uint8_t i = 0; i < 2; i++) {
        GetMotorMeasure(&CHASSIS.wheel_motor[i]);
    }
}

static void UpdateBodyStatus(void)
{
    CHASSIS.fdb.body.phi = CHASSIS.imu->pitch;
    CHASSIS.fdb.body.phi_dot = CHASSIS.imu->pitch_vel;

    CHASSIS.fdb.body.roll = -CHASSIS.imu->roll;
    CHASSIS.fdb.body.roll_dot = -CHASSIS.imu->roll_vel;

    CHASSIS.fdb.body.yaw = CHASSIS.imu->yaw;
    CHASSIS.fdb.body.yaw_dot = CHASSIS.imu->yaw_vel;

    CHASSIS.fdb.body.x_dot =
        WHEEL_RADIUS * (CHASSIS.fdb.leg[0].wheel.Velocity + CHASSIS.fdb.leg[1].wheel.Velocity) / 2;

    if (fabs(CHASSIS.ref.speed_vector.vx) < WHEEL_DEADZONE && fabs(CHASSIS.fdb.body.x_dot) < 0.8f) {
        // 当目标速度为0，且速度小于阈值时，计算反馈距离
        CHASSIS.fdb.body.x += CHASSIS.fdb.body.x_dot * CHASSIS_CONTROL_TIME_S;
    } else {
        //CHASSIS.fdb.body.x = 0;
    }
}

/**
 * @brief  更新腿部状态
 * @param  none
 */
static void UpdateLegStatus(void)
{
    uint8_t i = 0;
    // =====更新关节姿态=====
    CHASSIS.fdb.leg[0].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[0].fdb.pos, J0_ANGLE_OFFSET, J0_DIRECTION, 1);
    CHASSIS.fdb.leg[0].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[1].fdb.pos, J1_ANGLE_OFFSET, J1_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi1 =
        theta_transform(CHASSIS.joint_motor[2].fdb.pos, J2_ANGLE_OFFSET, J2_DIRECTION, 1);
    CHASSIS.fdb.leg[1].joint.Phi4 =
        theta_transform(CHASSIS.joint_motor[3].fdb.pos, J3_ANGLE_OFFSET, J3_DIRECTION, 1);

    CHASSIS.fdb.leg[0].joint.dPhi1 = CHASSIS.joint_motor[0].fdb.vel * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.dPhi4 = CHASSIS.joint_motor[1].fdb.vel * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi1 = CHASSIS.joint_motor[2].fdb.vel * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.dPhi4 = CHASSIS.joint_motor[3].fdb.vel * (J3_DIRECTION);

    CHASSIS.fdb.leg[0].joint.T1 = CHASSIS.joint_motor[0].fdb.tor * (J0_DIRECTION);
    CHASSIS.fdb.leg[0].joint.T2 = CHASSIS.joint_motor[1].fdb.tor * (J1_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T1 = CHASSIS.joint_motor[2].fdb.tor * (J2_DIRECTION);
    CHASSIS.fdb.leg[1].joint.T2 = CHASSIS.joint_motor[3].fdb.tor * (J3_DIRECTION);

    // =====更新驱动轮姿态=====
    CHASSIS.fdb.leg[0].wheel.Velocity = CHASSIS.wheel_motor[0].fdb.vel * (W0_DIRECTION);
    CHASSIS.fdb.leg[1].wheel.Velocity = CHASSIS.wheel_motor[1].fdb.vel * (W1_DIRECTION);

    // =====更新摆杆姿态=====
    float L0_Phi0[2];
    float dL0_dPhi0[2];
    for (i = 0; i < 2; i++) {
        float last_dL0 = CHASSIS.fdb.leg[i].rod.dL0;
        float last_dPhi0 = CHASSIS.fdb.leg[i].rod.dTheta;

        // 更新位置信息
        GetL0AndPhi0(CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, L0_Phi0);
        CHASSIS.fdb.leg[i].rod.L0 = L0_Phi0[0];
        CHASSIS.fdb.leg[i].rod.Phi0 = L0_Phi0[1];
        CHASSIS.fdb.leg[i].rod.Theta = M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.imu->pitch;

        // 计算雅可比矩阵
        CalcJacobian(
            CHASSIS.fdb.leg[i].joint.Phi1, CHASSIS.fdb.leg[i].joint.Phi4, CHASSIS.fdb.leg[i].J);

        // 更新速度信息
        GetdL0AnddPhi0(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.dPhi1, CHASSIS.fdb.leg[i].joint.dPhi4,
            dL0_dPhi0);
        CHASSIS.fdb.leg[i].rod.dL0 = dL0_dPhi0[0];
        CHASSIS.fdb.leg[i].rod.dPhi0 = dL0_dPhi0[1];
        CHASSIS.fdb.leg[i].rod.dTheta = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.imu->pitch_vel;

        // 更新加速度信息
        float accel = (CHASSIS.fdb.leg[i].rod.dL0 - last_dL0) / CHASSIS.duration;
        CHASSIS.fdb.leg[i].rod.ddL0 = LowPassFilterCalc(&CHASSIS.lpf.leg_l0_accel_filter[i], accel);

        accel = (CHASSIS.fdb.leg[i].rod.dPhi0 - last_dPhi0) / CHASSIS.duration;
        CHASSIS.fdb.leg[i].rod.ddPhi0 =
            LowPassFilterCalc(&CHASSIS.lpf.leg_phi0_accel_filter[i], accel);

        accel = (CHASSIS.fdb.leg[i].rod.dTheta - CHASSIS.fdb.leg[i].rod.ddPhi0) / CHASSIS.duration;
        CHASSIS.fdb.leg[i].rod.ddTheta =
            LowPassFilterCalc(&CHASSIS.lpf.leg_theta_accel_filter[i], accel);

        // 差分计算腿长变化率和腿角速度
        // TODO：结合姿态矩阵消去重力加速度得到更为精确的机体竖直方向运动加速度
        float ddot_z_M = CHASSIS.imu->z_accel - 9.8f;
        float l0 = CHASSIS.fdb.leg[i].rod.L0;
        float v_l0 = CHASSIS.fdb.leg[i].rod.dL0;
        float theta = CHASSIS.fdb.leg[i].rod.Theta;
        float w_theta = CHASSIS.fdb.leg[i].rod.dTheta;

        float dot_v_l0 = CHASSIS.fdb.leg[i].rod.ddL0;
        float dot_w_theta = CHASSIS.fdb.leg[i].rod.ddTheta;
        // clang-format off
        float ddot_z_w = ddot_z_M 
                    - dot_v_l0 * cosf(theta) 
                    + 2.0f * v_l0 * w_theta * sinf(theta) 
                    + l0 * dot_w_theta * cosf(theta) 
                    + l0 * powf(w_theta, 2) * sinf(theta);
        // clang-format on

        // 计算支撑力
        float F[2];
        GetLegForce(
            CHASSIS.fdb.leg[i].J, CHASSIS.fdb.leg[i].joint.T1, CHASSIS.fdb.leg[i].joint.T2, F);
        float F0 = F[0];
        float Tp = F[1];
        float P = F0 * arm_cos_f32(theta) + Tp * arm_sin_f32(theta) / l0;
        CHASSIS.fdb.leg[i].Fn = P + WHEEL_MASS * (9.8f + ddot_z_w);
    }
    ModifyDebugDataPackage(0, CHASSIS.fdb.leg[0].Fn, "FnL");
    ModifyDebugDataPackage(1, CHASSIS.fdb.leg[1].Fn, "FnR");
}

static void UpdateCalibrateStatus(void)
{
    if ((CHASSIS.mode == CHASSIS_CALIBRATE) &&
        fabs(CHASSIS.joint_motor[0].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[1].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[2].fdb.pos) < ZERO_POS_THRESHOLD &&
        fabs(CHASSIS.joint_motor[3].fdb.pos) < ZERO_POS_THRESHOLD) {
        CALIBRATE.calibrated = true;
    }

    // 校准模式的相关反馈数据
    uint32_t now = HAL_GetTick();
    if (CHASSIS.mode == CHASSIS_CALIBRATE) {
        for (uint8_t i = 0; i < 4; i++) {
            CALIBRATE.velocity[i] = CHASSIS.joint_motor[i].fdb.vel;
            if (CALIBRATE.velocity[i] > CALIBRATE_STOP_VELOCITY) {  // 速度大于阈值时重置计时
                CALIBRATE.reached[i] = false;
                CALIBRATE.stpo_time[i] = now;
            } else {
                if (now - CALIBRATE.stpo_time[i] > CALIBRATE_STOP_TIME) {
                    CALIBRATE.reached[i] = true;
                }
            }
        }
    }
}

/**
 * @brief  机体运动状态观测器
 * @param  none
 */
static void BodyMotionObserve(void)
{
    // 使用kf同时估计加速度和速度,滤波更新
    // TODO：使用xz平面上的加速度作为输入？
    OBSERVER.body.v_kf.MeasuredVector[0] = CHASSIS.fdb.body.x_dot;
    OBSERVER.body.v_kf.MeasuredVector[1] = CHASSIS.fdb.body.x_accel;
    OBSERVER.body.v_kf.F_data[1] = CHASSIS.duration;
    Kalman_Filter_Update(&OBSERVER.body.v_kf);
    CHASSIS.fdb.body.x_dot = OBSERVER.body.v_kf.xhat_data[0];
    CHASSIS.fdb.body.x_accel = OBSERVER.body.v_kf.xhat_data[1];

    // 更新2条腿的状态向量
    uint8_t i = 0;
    for (i = 0; i < 2; i++) {
        // clang-format off
        CHASSIS.fdb.leg_state[i].theta     =  M_PI_2 - CHASSIS.fdb.leg[i].rod.Phi0 - CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].theta_dot = -CHASSIS.fdb.leg[i].rod.dPhi0 - CHASSIS.fdb.body.phi_dot;
        CHASSIS.fdb.leg_state[i].x         =  CHASSIS.fdb.body.x;
        CHASSIS.fdb.leg_state[i].x_dot     =  CHASSIS.fdb.body.x_dot;
        CHASSIS.fdb.leg_state[i].phi       =  CHASSIS.fdb.body.phi;
        CHASSIS.fdb.leg_state[i].phi_dot   =  CHASSIS.fdb.body.phi_dot;
        // clang-format on
    }
}

/******************************************************************/
/* Reference                                                      */
/*----------------------------------------------------------------*/
/* main function:      ChassisReference                           */
/* auxiliary function: None                                       */
/******************************************************************/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ChassisReference(void)
{
    int16_t rc_x = 0, rc_wz = 0;
    int16_t rc_length = 0, rc_angle = 0;
    int16_t rc_roll = 0;
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_X_CHANNEL], rc_x, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_WZ_CHANNEL], rc_wz, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_LENGTH_CHANNEL], rc_length, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ANGLE_CHANNEL], rc_angle, CHASSIS_RC_DEADLINE);
    rc_deadband_limit(CHASSIS.rc->rc.ch[CHASSIS_ROLL_CHANNEL], rc_roll, CHASSIS_RC_DEADLINE);

    // 计算速度向量
    ChassisSpeedVector_t v_set = {0.0f, 0.0f, 0.0f};
    v_set.vx = rc_x * RC_TO_ONE * MAX_SPEED_VECTOR_VX;
    v_set.vy = 0;
    v_set.wz = -rc_wz * RC_TO_ONE * MAX_SPEED_VECTOR_WZ;
    switch (CHASSIS.mode) {
        case CHASSIS_FREE: {  // 底盘自由模式下，控制量为底盘坐标系下的速度
            break;
        }
        case CHASSIS_FOLLOW_GIMBAL_YAW: {  // 云台跟随模式下，控制量为云台坐标系下的速度，需要进行坐标转换
            // GimbalSpeedVectorToChassisSpeedVector(&v_set, CHASSIS.dyaw);
            break;
        }
        case CHASSIS_AUTO: {  // 底盘自动模式，控制量为云台坐标系下的速度，需要进行坐标转换
            break;
        }
        case CHASSIS_STAND_UP: {
            v_set.vx = 0;
            v_set.vy = 0;
            v_set.wz = 0;
        } break;
        default:
            break;
    }

    CHASSIS.ref.speed_vector.vx = v_set.vx;
    CHASSIS.ref.speed_vector.vy = 0;
    CHASSIS.ref.speed_vector.wz = v_set.wz;

    // 计算期望状态
    // clang-format off
    for (uint8_t i = 0; i < 2; i++) {
        CHASSIS.ref.leg_state[i].theta     =  0;
        CHASSIS.ref.leg_state[i].theta_dot =  0;
        CHASSIS.ref.leg_state[i].x         =  0;//CHASSIS.ref.speed_vector.vx * CHASSIS_CONTROL_TIME_S * X_ADD_RATIO;
        CHASSIS.ref.leg_state[i].x_dot     =  CHASSIS.ref.speed_vector.vx;
        CHASSIS.ref.leg_state[i].phi       =  0;
        CHASSIS.ref.leg_state[i].phi_dot   =  0;
    }
    // clang-format on

    // 腿部控制
    static float angle = M_PI_2;
    static float length = 0.15f;
    switch (CHASSIS.mode) {
        case CHASSIS_STAND_UP: {
            length = 0.12f;
            angle = M_PI_2;
        } break;
        case CHASSIS_CUSTOM:
        case CHASSIS_DEBUG: {
            angle = M_PI_2 + rc_angle * RC_TO_ONE * 0.3f;
            length = 0.22f + rc_length * RC_TO_ONE * 0.1f;
        } break;
        case CHASSIS_FREE: {
        } break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        default: {
            angle = M_PI_2;
            length = 0.15f;
        }
    }
    length = fp32_constrain(length, MIN_LEG_LENGTH, MAX_LEG_LENGTH);
    angle = fp32_constrain(angle, MIN_LEG_ANGLE, MAX_LEG_ANGLE);

    CHASSIS.ref.rod_L0[0] = length;
    CHASSIS.ref.rod_L0[1] = length;
    CHASSIS.ref.rod_Angle[0] = angle;
    CHASSIS.ref.rod_Angle[1] = angle;

    CHASSIS.ref.body.roll = fp32_constrain(rc_roll * RC_TO_ONE * MAX_ROLL, MIN_ROLL, MAX_ROLL);
}

/******************************************************************/
/* Console                                                        */
/*----------------------------------------------------------------*/
/* main function:      ChassisConsole                             */
/* auxiliary function: LocomotionController                       */
/*                     LegTorqueController                        */
/*                     LegFeedForward                             */
/*                     CalcLQR                                    */
/*                     ConsoleZeroForce                           */
/*                     ConsoleCalibrate                           */
/*                     ConsoleOffHook                             */
/*                     ConsoleNormal                              */
/*                     ConsoleDebug                               */
/*                     ConsoleStandUp                             */
/******************************************************************/

static void LocomotionController(void);
// static void LegPositionController(void);
static void LegTorqueController(void);
static float LegFeedForward(float theta);
static void CalcLQR(float k[2][6], float x[6], float t[2]);

static void ConsoleZeroForce(void);
static void ConsoleCalibrate(void);
static void ConsoleOffHook(void);
static void ConsoleNormal(void);
static void ConsoleDebug(void);
static void ConsoleStandUp(void);

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisConsole(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_CALIBRATE: {
            ConsoleCalibrate();
        } break;
        case CHASSIS_OFF_HOOK: {
            ConsoleOffHook();
        } break;
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            ConsoleNormal();
        } break;
        case CHASSIS_DEBUG: {
            ConsoleDebug();
        } break;
        case CHASSIS_STAND_UP: {
            ConsoleStandUp();
        } break;
        case CHASSIS_OFF:
        case CHASSIS_SAFE:
        default: {
            ConsoleZeroForce();
        }
    }
}

/**
 * @brief      运动控制器
 */
static void LocomotionController(void)
{
    // 计算LQR增益=============================================
    float k[2][6];
    float x[6];
    float T_Tp[2];

    for (uint8_t i = 0; i < 2; i++) {
        GetK(CHASSIS.fdb.leg[i].rod.L0, k);
        // clang-format off
        x[0] = X_0_RATIO * (CHASSIS.fdb.leg_state[i].theta     - CHASSIS.ref.leg_state[i].theta);
        x[1] = X_1_RATIO * (CHASSIS.fdb.leg_state[i].theta_dot - CHASSIS.ref.leg_state[i].theta_dot);
        x[2] = X_2_RATIO * (CHASSIS.fdb.leg_state[i].x         - CHASSIS.ref.leg_state[i].x);
        x[3] = X_3_RATIO * (CHASSIS.fdb.leg_state[i].x_dot     - CHASSIS.ref.leg_state[i].x_dot);
        x[4] = X_4_RATIO * (CHASSIS.fdb.leg_state[i].phi       - CHASSIS.ref.leg_state[i].phi);
        x[5] = X_5_RATIO * (CHASSIS.fdb.leg_state[i].phi_dot   - CHASSIS.ref.leg_state[i].phi_dot);
        // clang-format on
        CalcLQR(k, x, T_Tp);

        CHASSIS.cmd.leg[i].wheel.T = T_Tp[0];
        CHASSIS.cmd.leg[i].rod.Tp = T_Tp[1];
    }

    // ROLL角控制=============================================
    // 计算腿长差值
    float Ld0 = CHASSIS.fdb.leg[0].rod.L0 - CHASSIS.fdb.leg[1].rod.L0;
    float L_diff = CalcLegLengthDiff(Ld0, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // ModifyDebugDataPackage(6, Ld0, "Ld0");
    // ModifyDebugDataPackage(7, L_diff, "L_diff");

    // PID补偿稳态误差
    float delta_L0 =
        PID_calc(&CHASSIS.pid.roll_angle, CHASSIS.fdb.body.roll, CHASSIS.ref.body.roll);

    // 维持腿长在范围内
    CoordinateLegLength(&CHASSIS.ref.rod_L0[0], &CHASSIS.ref.rod_L0[1], L_diff, delta_L0);
    // ModifyDebugDataPackage(8, CHASSIS.ref.rod_L0[0], "L0l");
    // ModifyDebugDataPackage(9, CHASSIS.ref.rod_L0[1], "L0r");

    // 转向控制================================================
    PID_calc(&CHASSIS.pid.yaw_velocity, CHASSIS.fdb.body.yaw_dot, CHASSIS.ref.speed_vector.wz);
    CHASSIS.cmd.leg[0].wheel.T += CHASSIS.pid.yaw_velocity.out;
    CHASSIS.cmd.leg[1].wheel.T -= CHASSIS.pid.yaw_velocity.out;
}

/**
 * @brief 腿部力矩控制
 */
static void LegTorqueController(void)
{
    // 腿长控制
    float F_ff, F_compensate;
    for (uint8_t i = 0; i < 2; i++) {
        // 计算前馈力
        F_ff = LegFeedForward(CHASSIS.fdb.leg_state[i].theta) * FF_RATIO;
        // PID补偿
        F_compensate = PID_calc(
            &CHASSIS.pid.leg_length_length[i], CHASSIS.fdb.leg[i].rod.L0, CHASSIS.ref.rod_L0[i]);
        // 计算总力
        CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate;
        // CHASSIS.cmd.leg[i].rod.F = F_ff + F_compensate - F_ff;
    }

    // 转换为关节力矩
    CalcVmc(
        CHASSIS.cmd.leg[0].rod.F, CHASSIS.cmd.leg[0].rod.Tp, CHASSIS.fdb.leg[0].J,
        CHASSIS.cmd.leg[0].joint.T);
    CalcVmc(
        CHASSIS.cmd.leg[1].rod.F, CHASSIS.cmd.leg[1].rod.Tp, CHASSIS.fdb.leg[1].J,
        CHASSIS.cmd.leg[1].joint.T);
}

/**
 * @brief        前馈控制
 * @param[in]    theta 当前腿与竖直方向夹角
 * @return       前馈量
 */
static float LegFeedForward(float theta) { return BODY_MASS * GRAVITY * cosf(theta) / 2; }

/**
 * @brief         矩阵相乘，计算LQR输出
 * @param[in]     k   LQR反馈矩阵K
 * @param[in]     x   状态变量向量
 * @param[out]    T_Tp 反馈数据T和Tp
 * @return        none
 */
static void CalcLQR(float k[2][6], float x[6], float T_Tp[2])
{
    T_Tp[0] = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] +
              k[0][5] * x[5];
    T_Tp[1] = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] +
              k[1][5] * x[5];
}

//* 各个模式下的控制

static void ConsoleZeroForce(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    CHASSIS.wheel_motor[0].set.vel = 0;
    CHASSIS.wheel_motor[1].set.vel = 0;

    CHASSIS.wheel_motor[0].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[0], CHASSIS.wheel_motor[0].fdb.vel, 0);
    CHASSIS.wheel_motor[1].set.value =
        PID_calc(&CHASSIS.pid.wheel_stop[1], CHASSIS.wheel_motor[1].fdb.vel, 0);
}

static void ConsoleCalibrate(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleOffHook(void)
{
    CHASSIS.joint_motor[0].set.vel = -CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[1].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[2].set.vel = CALIBRATE_VELOCITY;
    CHASSIS.joint_motor[3].set.vel = -CALIBRATE_VELOCITY;

    CHASSIS.wheel_motor[0].set.tor = 0;
    CHASSIS.wheel_motor[1].set.tor = 0;
}

static void ConsoleNormal(void)
{
    LocomotionController();
    LegTorqueController();

    // 给关节电机赋值
    CHASSIS.joint_motor[0].set.tor = CHASSIS.cmd.leg[0].joint.T[0] * (J0_DIRECTION);
    CHASSIS.joint_motor[1].set.tor = CHASSIS.cmd.leg[0].joint.T[1] * (J1_DIRECTION);
    CHASSIS.joint_motor[2].set.tor = CHASSIS.cmd.leg[1].joint.T[0] * (J2_DIRECTION);
    CHASSIS.joint_motor[3].set.tor = CHASSIS.cmd.leg[1].joint.T[1] * (J3_DIRECTION);

    for (uint8_t i = 0; i < 4; i++) {
        CHASSIS.joint_motor[0].set.tor =
            fp32_constrain(CHASSIS.joint_motor[0].set.tor, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
    }

    // 给驱动轮电机赋值
    // QUESTION: 排查电机发送的力矩要反向的问题，这种情况下控制正常
    //不知道为什么要反向，待后续研究
    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));
}

static void ConsoleDebug(void)
{
    CHASSIS.joint_motor[0].set.tor = 0;
    CHASSIS.joint_motor[1].set.tor = 0;
    CHASSIS.joint_motor[2].set.tor = 0;
    CHASSIS.joint_motor[3].set.tor = 0;

    CHASSIS.joint_motor[0].set.vel = 0;
    CHASSIS.joint_motor[1].set.vel = 0;
    CHASSIS.joint_motor[2].set.vel = 0;
    CHASSIS.joint_motor[3].set.vel = 0;

    LocomotionController();

    float phi1_phi4_l[2], phi1_phi4_r[2];
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[0], CHASSIS.ref.rod_L0[0], phi1_phi4_l);
    CalcPhi1AndPhi4(CHASSIS.ref.rod_Angle[1], CHASSIS.ref.rod_L0[1], phi1_phi4_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(phi1_phi4_l[0]) || isnan(phi1_phi4_l[1]) || isnan(phi1_phi4_r[0]) ||
          isnan(phi1_phi4_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(phi1_phi4_l[0], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(phi1_phi4_l[1], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(phi1_phi4_r[0], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(phi1_phi4_r[1], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮控制===
    CHASSIS.wheel_motor[0].set.tor = -(CHASSIS.cmd.leg[0].wheel.T * (W0_DIRECTION));
    CHASSIS.wheel_motor[1].set.tor = -(CHASSIS.cmd.leg[1].wheel.T * (W1_DIRECTION));
}

static void ConsoleStandUp(void)
{
    // ===腿部位置控制===

    double joint_pos_l[2], joint_pos_r[2];
    // LegController(joint_pos_l, joint_pos_r);

    // 当解算出的角度正常时，设置目标角度
    if (!(isnan(joint_pos_l[0]) || isnan(joint_pos_l[1]) || isnan(joint_pos_r[0]) ||
          isnan(joint_pos_r[1]))) {
        CHASSIS.joint_motor[0].set.pos =
            theta_transform(joint_pos_l[1], -J0_ANGLE_OFFSET, J0_DIRECTION, 1);
        CHASSIS.joint_motor[1].set.pos =
            theta_transform(joint_pos_l[0], -J1_ANGLE_OFFSET, J1_DIRECTION, 1);
        CHASSIS.joint_motor[2].set.pos =
            theta_transform(joint_pos_r[1], -J2_ANGLE_OFFSET, J2_DIRECTION, 1);
        CHASSIS.joint_motor[3].set.pos =
            theta_transform(joint_pos_r[0], -J3_ANGLE_OFFSET, J3_DIRECTION, 1);
    }
    // 检测设定角度是否超过电机角度限制
    CHASSIS.joint_motor[0].set.pos =
        fp32_constrain(CHASSIS.joint_motor[0].set.pos, MIN_J0_ANGLE, MAX_J0_ANGLE);
    CHASSIS.joint_motor[1].set.pos =
        fp32_constrain(CHASSIS.joint_motor[1].set.pos, MIN_J1_ANGLE, MAX_J1_ANGLE);
    CHASSIS.joint_motor[2].set.pos =
        fp32_constrain(CHASSIS.joint_motor[2].set.pos, MIN_J2_ANGLE, MAX_J2_ANGLE);
    CHASSIS.joint_motor[3].set.pos =
        fp32_constrain(CHASSIS.joint_motor[3].set.pos, MIN_J3_ANGLE, MAX_J3_ANGLE);

    // ===驱动轮pid控制===
    float feedforward = -220;
    PID_calc(&CHASSIS.pid.stand_up, CHASSIS.fdb.body.phi, 0);
    CHASSIS.wheel_motor[0].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W0_DIRECTION;
    CHASSIS.wheel_motor[1].set.value = (feedforward + CHASSIS.pid.stand_up.out) * W1_DIRECTION;
}

/******************************************************************/
/* Cmd                                                            */
/*----------------------------------------------------------------*/
/* main function:      ChassisSendCmd                             */
/* auxiliary function: SendJointMotorCmd                          */
/*                     SendWheelMotorCmd                          */
/******************************************************************/

#define DM_DELAY 250

#define DEBUG_KP 17
#define DEBUG_KD 2

static void SendJointMotorCmd(void);
static void SendWheelMotorCmd(void);

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ChassisSendCmd(void)
{
    SendJointMotorCmd();
    SendWheelMotorCmd();
}

/**
 * @brief 发送关节电机控制指令
 * @param[in] chassis
 */
static void SendJointMotorCmd(void)
{
    uint8_t cnt;
    if (CHASSIS.mode == CHASSIS_OFF) {
        DmMitStop(&CHASSIS.joint_motor[0]);
        DmMitStop(&CHASSIS.joint_motor[1]);
        delay_us(DM_DELAY);
        DmMitStop(&CHASSIS.joint_motor[2]);
        DmMitStop(&CHASSIS.joint_motor[3]);
    } else {
        // bool flag = false;
        for (uint8_t i = 0; i < 4; i++) {
            if (cnt % 2 == 0) {
                delay_us(DM_DELAY);
            }
            if (CHASSIS.joint_motor[i].fdb.state == DM_STATE_DISABLE) {
                DmEnable(&CHASSIS.joint_motor[i]);
                // flag = true;
                cnt++;
            }
        }

        delay_us(DM_DELAY);

        switch (CHASSIS.mode) {
            case CHASSIS_FOLLOW_GIMBAL_YAW:
            case CHASSIS_CUSTOM:
            case CHASSIS_FREE: {
#if LOCATION_CONTROL
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], NORMAL_POS_KP, NORMAL_POS_KD);
                delay_us(DM_DELAY);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], NORMAL_POS_KP, NORMAL_POS_KD);
#else
                DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
                delay_us(DM_DELAY);
                DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
                DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
#endif
            } break;
            case CHASSIS_STAND_UP: {
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], NORMAL_POS_KP, NORMAL_POS_KD);
                delay_us(DM_DELAY);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], NORMAL_POS_KP, NORMAL_POS_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], NORMAL_POS_KP, NORMAL_POS_KD);
            } break;
            case CHASSIS_CALIBRATE: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);

                if (CALIBRATE.reached[0] && CALIBRATE.reached[1] && CALIBRATE.reached[2] &&
                    CALIBRATE.reached[3]) {
                    delay_us(DM_DELAY);
                    DmSavePosZero(&CHASSIS.joint_motor[0]);
                    DmSavePosZero(&CHASSIS.joint_motor[1]);
                    delay_us(DM_DELAY);
                    DmSavePosZero(&CHASSIS.joint_motor[2]);
                    DmSavePosZero(&CHASSIS.joint_motor[3]);
                }
            } break;
            case CHASSIS_OFF_HOOK: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);
            } break;
            case CHASSIS_DEBUG: {
                DmMitCtrlPosition(&CHASSIS.joint_motor[0], DEBUG_KP, DEBUG_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[1], DEBUG_KP, DEBUG_KD);
                delay_us(DM_DELAY);
                DmMitCtrlPosition(&CHASSIS.joint_motor[2], DEBUG_KP, DEBUG_KD);
                DmMitCtrlPosition(&CHASSIS.joint_motor[3], DEBUG_KP, DEBUG_KD);

                // DmMitCtrlVelocity(&CHASSIS.joint_motor[0], CALIBRATE_VEL_KP);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[1], CALIBRATE_VEL_KP);
                // delay_us(DM_DELAY);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[2], CALIBRATE_VEL_KP);
                // DmMitCtrlVelocity(&CHASSIS.joint_motor[3], CALIBRATE_VEL_KP);

                // DmMitCtrlTorque(&CHASSIS.joint_motor[0]);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[1]);
                // delay_us(DM_DELAY);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[2]);
                // DmMitCtrlTorque(&CHASSIS.joint_motor[3]);
            } break;
            case CHASSIS_SAFE:
            default: {
                DmMitCtrlVelocity(&CHASSIS.joint_motor[0], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[1], ZERO_FORCE_VEL_KP);
                delay_us(DM_DELAY);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[2], ZERO_FORCE_VEL_KP);
                DmMitCtrlVelocity(&CHASSIS.joint_motor[3], ZERO_FORCE_VEL_KP);
            }
        }
    }
}

/**
 * @brief 发送驱动轮电机控制指令
 * @param chassis
 */
static void SendWheelMotorCmd(void)
{
    switch (CHASSIS.mode) {
        case CHASSIS_FOLLOW_GIMBAL_YAW:
        case CHASSIS_CUSTOM:
        case CHASSIS_FREE: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_STAND_UP: {
            LkMultipleIqControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        } break;
        case CHASSIS_CALIBRATE: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_OFF: {
            LkMultipleTorqueControl(WHEEL_CAN, 0, 0, 0, 0);
        } break;
        case CHASSIS_DEBUG: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.tor, CHASSIS.wheel_motor[1].set.tor, 0, 0);
        } break;
        case CHASSIS_SAFE:
        default: {
            LkMultipleTorqueControl(
                WHEEL_CAN, CHASSIS.wheel_motor[0].set.value, CHASSIS.wheel_motor[1].set.value, 0,
                0);
        }
    }
}

/******************************************************************/
/* Public                                                         */
/*----------------------------------------------------------------*/
/* main function:      None                                       */
/* auxiliary function: SetCali                                    */
/*                     CmdCali                                    */
/*                     ChassisSetCaliData                         */
/*                     ChassisCmdCali                             */
/*                     ChassisGetStatus                           */
/*                     ChassisGetDuration                         */
/*                     ChassisGetSpeedVx                          */
/*                     ChassisGetSpeedVy                          */
/*                     ChassisGetSpeedWz                          */
/******************************************************************/

void SetCali(const fp32 motor_middle[4]) {}

bool_t CmdCali(fp32 motor_middle[4])
{
    // if (CALIBRATE.calibrated) {  // 校准完成
    //     CHASSIS.mode = CHASSIS_SAFE;
    //     CALIBRATE.calibrated = false;
    //     return true;
    // }

    // if (CHASSIS.mode != CHASSIS_CALIBRATE) {
    //     CALIBRATE.toggle = true;
    // }

    // return false;

    return true;
}

void ChassisSetCaliData(const fp32 motor_middle[4]) { SetCali(motor_middle); }

bool_t ChassisCmdCali(fp32 motor_middle[4]) { return CmdCali(motor_middle); }

inline uint8_t ChassisGetStatus(void) { return 0; }
inline uint32_t ChassisGetDuration(void) { return CHASSIS.duration; }
inline float ChassisGetSpeedVx(void) { return CHASSIS.fdb.speed_vector.vx; }
inline float ChassisGetSpeedVy(void) { return CHASSIS.fdb.speed_vector.vy; }
inline float ChassisGetSpeedWz(void) { return CHASSIS.fdb.speed_vector.wz; }
#endif /* CHASSIS_BALANCE */
/*------------------------------ End of File ------------------------------*/
