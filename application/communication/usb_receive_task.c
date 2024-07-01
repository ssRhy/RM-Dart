#include "usb_receive_task.h"

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "stdbool.h"
#include "string.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_RECEIVE_TASK_CONTROL_TIME 1  // ms

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 100    // byte

#define HEADER_SIZE 4

#define MAX_RECEIVE_CNT 10

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];
// 数据接收结构体
static ReceiveRobotCmdData_s RECEIVE_ROBOT_CMD_DATA;
static RobotCmdData_t ROBOT_CMD_DATA;

// Function Declarations

static void UsbReceiveData(void);
static void GetCmdData(void);

void usb_receive_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, "ROBOT_CMD_DATA");

    vTaskDelay(10);  //等待USB设备初始化完成

    while (1) {
        UsbReceiveData();
        GetCmdData();

        vTaskDelay(USB_RECEIVE_TASK_CONTROL_TIME);
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
