/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       CAN_communication.c/h
  * @brief      CAN通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     May-27-2024     Penguin         1. done
  *
  @verbatim
  ==============================================================================
板间通信时stdid的内容如下
data_type + (data_id << 4) + target_id

bit 0-3: target_id
bit 4-7: data_id
bit 8-11: data_type

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */
#include "CAN_communication.h"

#include "bsp_can.h"
#include "can_typedef.h"
#include "string.h"
#include "user_lib.h"

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

/*-------------------- Private functions --------------------*/
// 板间通信

/**
 * @brief          发送数据
 * @param[in]      hcan CAN句柄
 * @param[in]      std_id 数据包ID
 * @param[in]      data 包含8个字节的数据的指针
 * @retval         none
 */
static void SendData(uint8_t can, uint16_t std_id, uint8_t * data)
{
    if (can == 1)
        CAN_CTRL_DATA.hcan = &hcan1;
    else if (can == 2)
        CAN_CTRL_DATA.hcan = &hcan2;
    else
        return;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    memcpy(CAN_CTRL_DATA.tx_data, data, 8);

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/*-------------------- Public functions --------------------*/

void CanSendRcDataToBoard(uint8_t can, uint16_t target_id, uint16_t index)
{
    uint16_t std_id = CAN_STD_ID_PACK_BASE | CAN_STD_ID_Rc << TYPE_ID_OFFSET |
                      (target_id << TARGET_ID_OFFSET) | index;
    uint8_t data[8] = {0};

    const RC_ctrl_t * rc_ctrl = get_remote_control_point();

    if (index == 0) {  // 发送遥控器数据
        bool offline = GetRcOffline();
        uint16_t ch[5];
        ch[0] = rc_ctrl->rc.ch[0] + RC_CH_VALUE_OFFSET;
        ch[1] = rc_ctrl->rc.ch[1] + RC_CH_VALUE_OFFSET;
        ch[2] = rc_ctrl->rc.ch[2] + RC_CH_VALUE_OFFSET;
        ch[3] = rc_ctrl->rc.ch[3] + RC_CH_VALUE_OFFSET;
        ch[4] = rc_ctrl->rc.ch[4] + RC_CH_VALUE_OFFSET;

        data[0] = (ch[0] >> 3);                          // ch0 * 8
        data[1] = ((ch[0] & 0x07) << 5) | (ch[1] >> 6);  // ch0 * 3 + ch1 * 5
        data[2] = ((ch[1] & 0x3F) << 2) | (ch[2] >> 9);  // ch1 * 6 + ch2 * 2
        data[3] = ((ch[2] >> 1) & 0xFF);                 // ch2 * 8
        data[4] = ((ch[2] & 0x01) << 7) | (ch[3] >> 4);  // ch2 * 1 + ch3 * 7
        data[5] = ((ch[3] & 0x0F) << 4) | (ch[4] >> 7);  // ch3 * 4 + ch4 * 4
        data[6] = (offline << 7) | (ch[4] & 0x7F);       // offline * 1 + ch4 * 7
        data[7] = (rc_ctrl->rc.s[0] & 0x03) << 2 | (rc_ctrl->rc.s[1] & 0x03);
    } else if (index == 1) {  // 键鼠数据（经过压缩）
        data[0] = rc_ctrl->mouse.x >> 8;
        data[1] = (rc_ctrl->mouse.x & 0xFE) | (rc_ctrl->mouse.press_l & 0x01);
        data[2] = rc_ctrl->mouse.y >> 8;
        data[3] = (rc_ctrl->mouse.y & 0xFE) | (rc_ctrl->mouse.press_r & 0x01);
        data[4] = rc_ctrl->mouse.z >> 8;
        data[5] = rc_ctrl->mouse.z & 0xFF;
        data[6] = rc_ctrl->key.v >> 8;
        data[7] = rc_ctrl->key.v & 0xFF;
    }

    SendData(can, std_id, data);
}

/*------------------------------ End of File ------------------------------*/
