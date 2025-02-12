/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       can_cmd_dji.c/h
  * @brief      CAN发送函数，通过CAN信号控制DJI电机 GM3508 GM2006 GM6020.
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Mar-27-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#include "CAN_cmd_dji.h"

#include "bsp_delay.h"

/*------------------------------ Macro Definition ------------------------------*/

#define DJI_SEND_INTERVAL 10  // (us)DJI发送间隔

/*-------------------- Global var --------------------*/

static CanCtrlData_s CAN_CTRL_DATA = {
    .tx_header.IDE = CAN_ID_STD,
    .tx_header.RTR = CAN_RTR_DATA,
    .tx_header.DLC = 8,
};

// std_id of [0][n] = 0x200;of [1][n] = 0x1FF;of [2][n] = 0x2FF
static int16_t cmd_value[3][4] = {0};

/*-------------------- Private functions --------------------*/

/**
 * @brief          通过CAN发送控制量控制DJI电机(支持M3508 M2006 GM6020)
 * @param[in]      can 发送数据使用的can口
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      value_1 电机控制量
 * @param[in]      value_2 电机控制量
 * @param[in]      value_3 电机控制量
 * @param[in]      value_4 电机控制量
 * @return         none
 */
static void MultipleMotorControl(
    hcan_t * hcan, uint16_t std_id, int16_t value_1, int16_t value_2, int16_t value_3,
    int16_t value_4)
{
    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    CAN_CTRL_DATA.tx_data[0] = (value_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = value_1;
    CAN_CTRL_DATA.tx_data[2] = (value_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = value_2;
    CAN_CTRL_DATA.tx_data[4] = (value_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = value_3;
    CAN_CTRL_DATA.tx_data[6] = (value_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = value_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief          通过CAN发送控制电流控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流
 * @param[in]      curr_2 电机控制电流
 * @param[in]      curr_3 电机控制电流
 * @param[in]      curr_4 电机控制电流
 * @return         none
 */
// static void MultipleCurrentControl(
//     hcan_t * hcan, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
// {
//     CAN_CTRL_DATA.hcan = hcan;

//     CAN_CTRL_DATA.tx_header.StdId = std_id;

//     CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);
//     CAN_CTRL_DATA.tx_data[1] = curr_1;
//     CAN_CTRL_DATA.tx_data[2] = (curr_2 >> 8);
//     CAN_CTRL_DATA.tx_data[3] = curr_2;
//     CAN_CTRL_DATA.tx_data[4] = (curr_3 >> 8);
//     CAN_CTRL_DATA.tx_data[5] = curr_3;
//     CAN_CTRL_DATA.tx_data[6] = (curr_4 >> 8);
//     CAN_CTRL_DATA.tx_data[7] = curr_4;

//     CAN_SendTxMessage(&CAN_CTRL_DATA);
// }

/**
 * @brief          通过CAN发送控制电压控制DJI电机(支持GM6020)
 * @param[in]      can 发送数据使用的can口
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      volt_1 电机控制电压
 * @param[in]      volt_2 电机控制电压
 * @param[in]      volt_3 电机控制电压
 * @param[in]      volt_4 电机控制电压
 * @return         none
 */
// static void MultipleVoltageControl(
//     hcan_t * hcan, uint16_t std_id, int16_t volt_1, int16_t volt_2, int16_t volt_3, int16_t volt_4)
// {
//     CAN_CTRL_DATA.hcan = hcan;

//     CAN_CTRL_DATA.tx_header.StdId = std_id;

//     CAN_CTRL_DATA.tx_data[0] = (volt_1 >> 8);
//     CAN_CTRL_DATA.tx_data[1] = volt_1;
//     CAN_CTRL_DATA.tx_data[2] = (volt_2 >> 8);
//     CAN_CTRL_DATA.tx_data[3] = volt_2;
//     CAN_CTRL_DATA.tx_data[4] = (volt_3 >> 8);
//     CAN_CTRL_DATA.tx_data[5] = volt_3;
//     CAN_CTRL_DATA.tx_data[6] = (volt_4 >> 8);
//     CAN_CTRL_DATA.tx_data[7] = volt_4;

//     CAN_SendTxMessage(&CAN_CTRL_DATA);
// }
/*-------------------- User function --------------------*/

/**
 * @brief          通过CAN控制DJI电机(支持GM3508 GM2006 GM6020)
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      std_id 发送数据使用的std_id
 * @param[in]      curr_1 电机控制电流(id=1/5)
 * @param[in]      curr_2 电机控制电流(id=2/6)
 * @param[in]      curr_3 电机控制电流(id=3/7)
 * @param[in]      curr_4 电机控制电流(id=4/8)
 * @return         none
 * @note           老的控制方式的兼容函数，等后期的安全函数上线后会删除
 */
void CanCmdDjiMotor(
    uint8_t can, uint16_t std_id, int16_t curr_1, int16_t curr_2, int16_t curr_3, int16_t curr_4)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    CAN_CTRL_DATA.hcan = hcan;

    CAN_CTRL_DATA.tx_header.StdId = std_id;

    CAN_CTRL_DATA.tx_data[0] = (curr_1 >> 8);
    CAN_CTRL_DATA.tx_data[1] = curr_1;
    CAN_CTRL_DATA.tx_data[2] = (curr_2 >> 8);
    CAN_CTRL_DATA.tx_data[3] = curr_2;
    CAN_CTRL_DATA.tx_data[4] = (curr_3 >> 8);
    CAN_CTRL_DATA.tx_data[5] = curr_3;
    CAN_CTRL_DATA.tx_data[6] = (curr_4 >> 8);
    CAN_CTRL_DATA.tx_data[7] = curr_4;

    CAN_SendTxMessage(&CAN_CTRL_DATA);
}

/**
 * @brief 测试阶段，谨慎使用！！！ dji多电机控制
 * @param[in]      can 发送数据使用的can口(1/2)
 * @param[in]      motor_num 电机数量
 * @param[in]      p_motor_array 电机数组指针
 */
void DjiMultipleControl(uint8_t can, uint8_t motor_num, Motor_s * motor_array)
{
    hcan_t * hcan = NULL;
    if (can == 1)
        hcan = &hcan1;
    else if (can == 2)
        hcan = &hcan2;
    if (hcan == NULL) return;

    bool using_flag[3] = {0};  // 0:0x200 1:0x1FF 2:0x2FF
    uint8_t std_id_index = 0;
    uint8_t motor_id_index = 0;

    for (uint8_t i = 0; i < motor_num; i++) {
        motor_id_index = (motor_array[i].id - 1) % 4;  // 计算电机id对应的索引

        switch (motor_array[i].type) {
            case DJI_M2006:
            case DJI_M3508: {
                if (motor_array[i].id > 8) {  // 2006和3508电机id最大为8
                    continue;
                }
                if (motor_array[i].mode == DJI_CURRENT_MODE) {
                    std_id_index = (motor_array[i].id - 1) / 4;  // 计算std_id对应的索引

                    cmd_value[std_id_index][motor_id_index] = motor_array[i].set.value;

                    using_flag[std_id_index] = true;
                }
            } break;
            case DJI_M6020: {
                if (motor_array[i].id > 7) {  // 6020电机id最大为7
                    continue;
                }
                if (motor_array[i].mode == DJI_CURRENT_MODE) {
                    ;
                } else if (motor_array[i].mode == DJI_VOLTAGE_MODE) {
                    std_id_index = 1 + (motor_array[i].id - 1) / 4;  // 计算std_id对应的索引

                    cmd_value[std_id_index][motor_id_index] = motor_array[i].set.value;

                    using_flag[std_id_index] = true;
                }
            } break;

            default:
                break;
        }
    }

    if (using_flag[0]) {  // 0x200
        MultipleMotorControl(
            hcan, 0x200, cmd_value[0][0], cmd_value[0][1], cmd_value[0][2], cmd_value[0][3]);
        delay_us(DJI_SEND_INTERVAL);
    }

    if (using_flag[1]) {  // 0x1FF
        MultipleMotorControl(
            hcan, 0x1FF, cmd_value[1][0], cmd_value[1][1], cmd_value[1][2], cmd_value[1][3]);
        delay_us(DJI_SEND_INTERVAL);
    }

    if (using_flag[2]) {  // 0x2FF
        MultipleMotorControl(
            hcan, 0x2FF, cmd_value[2][0], cmd_value[2][1], cmd_value[2][2], cmd_value[2][3]);
    }
}

#undef DJI_SEND_INTERVAL

/************************ END OF FILE ************************/
