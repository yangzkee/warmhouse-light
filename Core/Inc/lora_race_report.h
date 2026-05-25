/**
 * @file lora_race_report.h
 * @brief 仅在 9.60m、20.10m 累计路程门限经 USART3(LoRa) 上报；其余 LoRa 调试由工程侧关断。
 */
#ifndef LORA_RACE_REPORT_H
#define LORA_RACE_REPORT_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void LoraRaceReport_Try(float total_dist_m);

#ifdef __cplusplus
}
#endif

#endif /* LORA_RACE_REPORT_H */
