/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       SupCap.c/h
  * @brief      超级电容相关部分定义
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-29-2024     Penguin         1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef SUPCAP_H
#define SUPCAP_H

#include "struct_typedef.h"

typedef struct
{
    int16_t voltage_in;
    int16_t voltage_cap;
    int16_t current_in;
    int16_t power_target;

    uint32_t last_fdb_time;  //上次反馈时间
} SupCapMeasure_s;

#endif
/************************ END OF FILE ************************/
