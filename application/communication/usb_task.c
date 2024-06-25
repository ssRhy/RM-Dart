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
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_TASK_CONTROL_TIME 1  // ms
#define MAX_RESEND_CNT 10
#define APP_RX_DATA_SIZE 2048
#define APP_TX_DATA_SIZE 2048
#define USB_RECEIVE_LEN 64  // byte

// Variable Declarations
static uint8_t USB_TX_BUF[APP_TX_DATA_SIZE];
static uint8_t USB_RX_BUF[APP_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// clang-format off
static DebugSendData_s SEND_DATA_DEBUG;
static ImuSendData_s   SEND_DATA_IMU;
// static ImuSendData_s   SEND_DATA_IMU;
// clang-format on

// function declaration
static void UsbSendData(uint16_t len);
static void UsbReceiveData(void);

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();

    vTaskDelay(10);  //等待USB设备初始化完成

    IMU = Subscribe("imu_data");                        // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe("chassis_fdb_speed");  // 获取底盘速度矢量指针

    while (IMU == NULL || FDB_SPEED_VECTOR == NULL || &SEND_DATA_DEBUG == NULL ||
           &SEND_DATA_IMU == NULL) {
        break;
    }

    while (1) {
        UsbSendData(sizeof(SEND_DATA_DEBUG));
        UsbReceiveData();
        
        vTaskDelay(USB_TASK_CONTROL_TIME);
    }
}

/**
 * @brief      用USB发送数据
 * @param[in]  len 发送数据的长度
 */
static void UsbSendData(uint16_t len)
{
    uint8_t resend_cnt = 0;
    uint8_t usb_send_state = USBD_FAIL;
    while (usb_send_state != USBD_OK && resend_cnt < MAX_RESEND_CNT) {
        usb_send_state = CDC_Transmit_FS(USB_TX_BUF, len);
        resend_cnt++;
    }
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
}
