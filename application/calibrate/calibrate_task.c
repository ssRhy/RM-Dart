/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       calibrate_task.c/h
  * @brief      校准任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jul-27-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#define CALIBRATE_CONTROL_TIME 1

#include "cmsis_os.h"
/**
  * @brief          校准任务
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void calibrate_task(void const * pvParameters)
{
    while (1) {
        ;
        vTaskDelay(CALIBRATE_CONTROL_TIME);
    }
}

void cali_param_init(void) { ; }

int8_t get_control_temperature(void) { return 30.0f; }
