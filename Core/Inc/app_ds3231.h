#ifndef APP_DS3231_H
#define APP_DS3231_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t week;
  uint8_t day;
  uint8_t month;
  uint8_t year;
} DS3231_Time_t;

void DS3231_InitAndSyncToBuildTime(void);
void DS3231_PrintTimeOnce(void);
HAL_StatusTypeDef DS3231_GetTime(DS3231_Time_t *t);

#ifdef __cplusplus
}
#endif

#endif /* APP_DS3231_H */
