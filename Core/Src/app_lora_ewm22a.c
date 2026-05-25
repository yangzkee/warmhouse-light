#include "app_lora_ewm22a.h"
#include "usart.h"
#if EWM22A_LINK_TEST_ENABLE
#include <stdio.h>
#endif

static uint8_t s_rx_byte;

#if EWM22A_LINK_TEST_ENABLE
static char s_rx_line[128];
static uint8_t s_rx_line_len;
static volatile uint8_t s_deferred_log_ready;
static char s_deferred_line[128];
#endif

void EWM22A_StartReceiveIT(void)
{
  (void)HAL_UART_Receive_IT(&huart3_aux, &s_rx_byte, 1);
}

HAL_StatusTypeDef EWM22A_SendBytes(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U)
  {
    return HAL_ERROR;
  }
  return HAL_UART_Transmit(&huart3_aux, (uint8_t *)data, len, 300U);
}

void EWM22A_LinkTest_PingOnce(void)
{
#if EWM22A_LINK_TEST_ENABLE
  static const uint8_t k[] = "HELLO_FROM_MCU\r\n";
  (void)EWM22A_SendBytes(k, (uint16_t)(sizeof(k) - 1U));
#endif
}

void EWM22A_LinkTest_Tick(void)
{
#if EWM22A_LINK_TEST_ENABLE
  static uint32_t s_last_ms;
  uint32_t t = HAL_GetTick();
  if ((t - s_last_ms) < EWM22A_LINK_TEST_PERIOD_MS)
  {
    return;
  }
  s_last_ms = t;
  {
    static const uint8_t k[] = "PING_MCU\r\n";
    (void)EWM22A_SendBytes(k, (uint16_t)(sizeof(k) - 1U));
  }
#endif
}

void EWM22A_PollDeferredLog(void)
{
#if EWM22A_LINK_TEST_ENABLE
  if (s_deferred_log_ready == 0U)
  {
    return;
  }
  s_deferred_log_ready = 0U;
  printf("[LoRa_RX] %s\r\n", s_deferred_line);
#endif
}

void EWM22A_OnUartRxCplt(void)
{
#if EWM22A_LINK_TEST_ENABLE
  {
    uint8_t c = s_rx_byte;
    uint8_t i;

    if (c == (uint8_t)'\r' || c == (uint8_t)'\n')
    {
      if (s_rx_line_len > 0U)
      {
        s_rx_line[s_rx_line_len] = '\0';
        /* ISR 内禁止 printf：仅当上一行已取走时再缓存，避免与主循环竞态 */
        if (s_deferred_log_ready == 0U)
        {
          for (i = 0U; i < s_rx_line_len && i < (uint8_t)(sizeof(s_deferred_line) - 1U); i++)
          {
            s_deferred_line[i] = s_rx_line[i];
          }
          s_deferred_line[i] = '\0';
          s_deferred_log_ready = 1U;
        }
        s_rx_line_len = 0U;
      }
    }
    else if (s_rx_line_len < (sizeof(s_rx_line) - 1U))
    {
      s_rx_line[s_rx_line_len++] = (char)c;
    }
    else
    {
      s_rx_line_len = 0U;
    }
  }
#else
  (void)s_rx_byte;
#endif
  (void)HAL_UART_Receive_IT(&huart3_aux, &s_rx_byte, 1);
}

void EWM22A_OnUartError(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_PEFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  (void)HAL_UART_Receive_IT(&huart3_aux, &s_rx_byte, 1);
}
