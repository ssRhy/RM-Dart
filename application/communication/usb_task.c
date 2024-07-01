/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "stdbool.h"
#include "string.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_TASK_CONTROL_TIME 1  // ms

#define SEND_DURATION_IMU 1          // ms
#define SEND_DURATION_DEBUG 1        // ms
#define SEND_DURATION_ROBOT_INFO 10  // ms

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 100   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 数据发送结构体
// clang-format off
static DebugSendData_s     SEND_DATA_DEBUG;
static ImuSendData_s       SEND_DATA_IMU;
static RobotInfoSendData_s SEND_DATA_ROBOT_INFO;
// clang-format on

// 数据接收结构体
static ReceiveRobotCmdData_s RECEIVE_ROBOT_CMD_DATA;

// 机器人控制指令数据
static RobotCmdData_t ROBOT_CMD_DATA;

// 发送数据间隔时间
typedef struct
{
    uint8_t imu;
    uint8_t debug;
    uint8_t robot_info;
} Duration_t;
static Duration_t DURATION;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendImuData(uint8_t duration);
static void UsbSendDebugData(uint8_t duration);
static void UsbSendRobotInfoData(uint8_t duration);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void);

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成

    UsbInit();

    if (IMU == NULL || FDB_SPEED_VECTOR == NULL || &SEND_DATA_DEBUG == NULL ||
        &SEND_DATA_IMU == NULL) {
        ;
    }

    while (1) {
        ModifyDebugDataPackage(0, IMU->yaw, "yaw");
        ModifyDebugDataPackage(1, SEND_DATA_IMU.time_stamp, "data1");
        ModifyDebugDataPackage(2, ROBOT_CMD_DATA.speed_vector.vx, "vx_set");
        ModifyDebugDataPackage(3, ROBOT_CMD_DATA.speed_vector.vy, "vy_set");
        ModifyDebugDataPackage(4, ROBOT_CMD_DATA.gimbal.pitch, "pitch");
        ModifyDebugDataPackage(5, ROBOT_CMD_DATA.gimbal.yaw, "yaw");
        
        UsbSendData();
        UsbReceiveData();
        GetCmdData();

        vTaskDelay(USB_TASK_CONTROL_TIME);
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    IMU = Subscribe("imu_data");                        // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe("chassis_fdb_speed");  // 获取底盘速度矢量指针

    // 数据置零
    memset(&DURATION, 0, sizeof(Duration_t));

    // 1.初始化调试数据包
    // 帧头部分
    SEND_DATA_DEBUG.frame_header.sof = 0x5A;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(DebugSendData_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = 0x01;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));

    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }

    // 2.初始化IMU数据包
    // 帧头部分
    SEND_DATA_IMU.frame_header.sof = 0x5A;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(ImuSendData_s) - 6);
    SEND_DATA_IMU.frame_header.id = 0x02;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    // 数据部分

    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_INFO.frame_header.sof = 0x5A;
    SEND_DATA_ROBOT_INFO.frame_header.len = (uint8_t)(sizeof(RobotInfoSendData_s) - 6);
    SEND_DATA_ROBOT_INFO.frame_header.id = 0x03;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_INFO.frame_header), sizeof(SEND_DATA_ROBOT_INFO.frame_header));
    // 数据部分
    SEND_DATA_ROBOT_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_INFO.data.type.arm = MECHANICAL_ARM_TYPE;

    // SEND_DATA_ROBOT_INFO.data.type.chassis = 1;
    // SEND_DATA_ROBOT_INFO.data.type.gimbal = 2;
    // SEND_DATA_ROBOT_INFO.data.type.shoot = 3;
    // SEND_DATA_ROBOT_INFO.data.type.arm = 4;
    // SEND_DATA_ROBOT_INFO.data.type.custom_controller = 5;

    // sizeof(RobotCmdData_s);
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送imu数据
    UsbSendImuData(SEND_DURATION_IMU);
    // 发送debug数据
    UsbSendDebugData(SEND_DURATION_DEBUG);
    // 发送机器人信息数据
    UsbSendRobotInfoData(SEND_DURATION_ROBOT_INFO);
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;

    uint32_t p = 0; // p:sof的位置

    // 读取数据
    USB_Receive(USB_RX_BUF, &len);  // Read data into the buffer
    // 寻找帧头位置
    while (*(USB_RX_BUF+p) != 0x5A && p < len/2)
    {
        p++;
    }

    if (p > len/2){
        return;
    }

    // 检查CRC8校验
    bool crc8_ok = verify_CRC8_check_sum(USB_RX_BUF + p, HEADER_SIZE);
    if (crc8_ok) {
        // 检查整包CRC16校验
        bool crc16_ok = verify_CRC16_check_sum(USB_RX_BUF + p, sizeof(ReceiveRobotCmdData_s));
        if (crc16_ok) {
            memcpy(&RECEIVE_ROBOT_CMD_DATA, USB_RX_BUF, sizeof(ReceiveRobotCmdData_s));
        }
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(uint8_t duration)
{
    if (IMU == NULL) {
        return;
    }

    DURATION.imu++;
    if (DURATION.imu < duration) {
        return;
    }
    DURATION.imu = 0;

    SEND_DATA_IMU.data.yaw = IMU->yaw;
    SEND_DATA_IMU.data.pitch = IMU->pitch;
    SEND_DATA_IMU.data.roll = IMU->roll;

    SEND_DATA_IMU.data.yaw_vel = IMU->yaw_vel;
    SEND_DATA_IMU.data.pitch_vel = IMU->pitch_vel;
    SEND_DATA_IMU.data.roll_vel = IMU->roll_vel;

    SEND_DATA_IMU.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(ImuSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(ImuSendData_s));
}

/**
 * @brief 发送DEBUG数据
 * @param duration 发送周期
 */
static void UsbSendDebugData(uint8_t duration)
{
    DURATION.debug++;
    if (DURATION.debug < duration) {
        return;
    }
    DURATION.debug = 0;

    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(DebugSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(DebugSendData_s));
}

/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotInfoData(uint8_t duration)
{
    DURATION.robot_info++;
    if (DURATION.robot_info < duration) {
        return;
    }
    DURATION.robot_info = 0;

    SEND_DATA_ROBOT_INFO.time_stamp = HAL_GetTick();

    SEND_DATA_ROBOT_INFO.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_DATA_ROBOT_INFO.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_DATA_ROBOT_INFO.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_INFO, sizeof(RobotInfoSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_INFO, sizeof(RobotInfoSendData_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引
 * @param data  发送数据
 * @param name  数据名称
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    SEND_DATA_DEBUG.packages[index].data = data;
    SEND_DATA_DEBUG.packages[index].type = 1;

    uint8_t i = 0;
    while (name[i] != '\0' && i < 10) {
        SEND_DATA_DEBUG.packages[index].name[i] = name[i];
        i++;
    }

    //TODO:添加对数据名称的一些检查工作
}
