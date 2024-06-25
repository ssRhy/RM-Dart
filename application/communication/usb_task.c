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
#include "string.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_RX_DATA_SIZE 512  // byte
#define USB_RECEIVE_LEN 64    // byte

#define SEND_DURATION_IMU 1    // ms
#define SEND_DURATION_DEBUG 1  // ms

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// 数据发送结构体
// clang-format off
static DebugSendData_s SEND_DATA_DEBUG;
static ImuSendData_s   SEND_DATA_IMU;
// static ImuSendData_s   SEND_DATA_IMU;
// clang-format on

// 发送数据间隔时间
typedef struct
{
    uint8_t imu;
    uint8_t debug;
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
        UsbSendData();
        UsbReceiveData();

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

    // 初始化调试数据包
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

    // 初始化IMU数据包
    // 帧头部分
    SEND_DATA_IMU.frame_header.sof = 0x5A;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(ImuSendData_s) - 6);
    SEND_DATA_IMU.frame_header.id = 0x02;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    // 数据部分
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
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    USB_Receive(USB_RX_BUF, &len);  // Read data into the buffer
    // uint8_t receive_ok = 0;
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 
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

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(ImuSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(ImuSendData_s));
}

/**
 * @brief 
 * @param duration 发送周期
 */
static void UsbSendDebugData(uint8_t duration)
{
    DURATION.debug++;
    if (DURATION.debug < duration) {
        return;
    }
    DURATION.debug = 0;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_DEBUG, sizeof(DebugSendData_s));
    USB_Transmit((uint8_t *)&SEND_DATA_DEBUG, sizeof(DebugSendData_s));
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
