#ifndef APP_LORA_EWM22A_H
#define APP_LORA_EWM22A_H

#include "main.h"
#include "stm32g0xx_hal_uart.h"

/* 1=透传联调：上电/周期发 HELLO、收行回显（回显在主循环 Poll，勿在 RX 回调里 printf）。正式跑图建议 0 */
#ifndef EWM22A_LINK_TEST_ENABLE
#define EWM22A_LINK_TEST_ENABLE  0
#endif

#ifndef EWM22A_LINK_TEST_PERIOD_MS
#define EWM22A_LINK_TEST_PERIOD_MS  3000U
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 硬件 USART3（huart3_aux）：PA5 TX → 模块 RX，PB0 RX ← 模块 TX。
 * 须先 MX_USART3_UART_Init()，再调用本接口启动中断收字节。
 */
void EWM22A_StartReceiveIT(void);
void EWM22A_OnUartRxCplt(void);
void EWM22A_OnUartError(UART_HandleTypeDef *huart);

HAL_StatusTypeDef EWM22A_SendBytes(const uint8_t *data, uint16_t len);
void EWM22A_LinkTest_PingOnce(void);
/** 联调：周期性经 USART3 发 HELLO（便于不必反复复位即可在电脑串口抓包） */
void EWM22A_LinkTest_Tick(void);
/** 主循环调用：将 RX 回调中缓存的一行 LoRa 文本用 printf 发出（避免在 USART3 ISR 里阻塞发串口） */
void EWM22A_PollDeferredLog(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LORA_EWM22A_H */
