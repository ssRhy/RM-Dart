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
  @todo
        1.上下位机建立稳定连接后再进行通信
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "stdbool.h"
#include "string.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_TASK_CONTROL_TIME 1  // ms

#define SEND_DURATION_IMU 1          // ms
#define SEND_DURATION_DEBUG 1        // ms
#define SEND_DURATION_ROBOT_INFO 10  // ms
#define SEND_DURATION_PID 10         // ms
#define SEND_DURATION_ALL_ROBOT_HP 10  // ms
#define SEND_DURATION_GAME_STATUS 10   // ms

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 数据发送结构体
// clang-format off
static DebugSendData_s      SEND_DATA_DEBUG;
static ImuSendData_s        SEND_DATA_IMU;
static RobotInfoSendData_s  SEND_DATA_ROBOT_INFO;
static PidDebugSendData_s   SEND_DATA_PID;
static AllRobotHpSendData_s SEND_DATA_ALL_ROBOT_HP;
static GameStatusSendData_s SEND_DATA_GAME_STATUS;
// clang-format on

// 数据接收结构体
static RobotCmdReceiveData_s RECEIVE_ROBOT_CMD_DATA;
static PidDebugReceiveData_s RECEIVE_PID_DEBUG_DATA;

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
static void UsbSendAllRobotHpData(uint8_t duration);
static void UsbSendGameStatusData(uint8_t duration);

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

    Publish(&ROBOT_CMD_DATA, "ROBOT_CMD_DATA");

    vTaskDelay(10);  //等待USB设备初始化完成

    UsbInit();

    while (1) {
        ModifyDebugDataPackage(0, IMU->yaw, "yaw");
        ModifyDebugDataPackage(1, SEND_DATA_IMU.time_stamp % 1000, "data1");
        ModifyDebugDataPackage(2, ROBOT_CMD_DATA.speed_vector.vx, "vx_set");
        ModifyDebugDataPackage(3, ROBOT_CMD_DATA.speed_vector.vy, "vy_set");
        ModifyDebugDataPackage(4, ROBOT_CMD_DATA.gimbal.pitch, "pitch");

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
    SEND_DATA_DEBUG.frame_header.sof = SEND_SOF;
    SEND_DATA_DEBUG.frame_header.len = (uint8_t)(sizeof(DebugSendData_s) - 6);
    SEND_DATA_DEBUG.frame_header.id = DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_DEBUG.frame_header), sizeof(SEND_DATA_DEBUG.frame_header));
    // 数据部分
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }

    // 2.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(ImuSendData_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));

    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_INFO.frame_header.len = (uint8_t)(sizeof(RobotInfoSendData_s) - 6);
    SEND_DATA_ROBOT_INFO.frame_header.id = ROBOT_INFO_DATA_SEND_ID;
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

    // 4.初始化pid调参数据
    SEND_DATA_PID.frame_header.sof = SEND_SOF;
    SEND_DATA_PID.frame_header.len = (uint8_t)(sizeof(PidDebugSendData_s) - 6);
    SEND_DATA_PID.frame_header.id = PID_DEBUG_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_PID.frame_header), sizeof(SEND_DATA_PID.frame_header));

    // 5.初始化所有机器人血量数据
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(AllRobotHpSendData_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header),
        sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));

    // 6.初始化比赛状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(GameStatusSendData_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),
        sizeof(SEND_DATA_GAME_STATUS.frame_header));
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
    // 发送全场机器人hp信息数据
    UsbSendAllRobotHpData(SEND_DURATION_ALL_ROBOT_HP);
    // 发送比赛状态数据
    UsbSendGameStatusData(SEND_DURATION_GAME_STATUS);
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(RobotCmdReceiveData_s));
                    } break;
                    case PID_DEBUG_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_PID_DEBUG_DATA, sof_address, sizeof(PidDebugReceiveData_s));
                    } break;
                    default:
                        break;
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
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

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(uint8_t duration)
{
    DURATION.robot_info++;
    if (DURATION.robot_info < duration) {
        return;
    }
    DURATION.robot_info = 0;

    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    SEND_DATA_ALL_ROBOT_HP.data.red_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.red_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.red_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.red_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.red_5_robot_hp = 5;
    SEND_DATA_ALL_ROBOT_HP.data.red_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.red_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.red_base_hp = 9;
    SEND_DATA_ALL_ROBOT_HP.data.blue_1_robot_hp = 1;
    SEND_DATA_ALL_ROBOT_HP.data.blue_2_robot_hp = 2;
    SEND_DATA_ALL_ROBOT_HP.data.blue_3_robot_hp = 3;
    SEND_DATA_ALL_ROBOT_HP.data.blue_4_robot_hp = 4;
    SEND_DATA_ALL_ROBOT_HP.data.blue_5_robot_hp = 5;
    SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 7;
    SEND_DATA_ALL_ROBOT_HP.data.blue_outpost_hp = 8;
    SEND_DATA_ALL_ROBOT_HP.data.blue_base_hp = 9;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(AllRobotHpSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(AllRobotHpSendData_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(uint8_t duration)
{
    DURATION.robot_info++;
    if (DURATION.robot_info < duration) {
        return;
    }
    DURATION.robot_info = 0;

    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_progress = 1;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = 100;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(GameStatusSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(GameStatusSendData_s));
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
