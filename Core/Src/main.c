/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DF_Communication.h"
#include "ir_grayscale.h"
#include "nav_odom.h"
#include "LineTracking.h"
#include "odom_y_calib.h"
#include "debug_odom_uart.h"
#include "app_ds3231.h"
#include "app_lora_ewm22a.h"
#include "oled.h"
#include <stdio.h>
#if LT_RADAR_BRANCH_ENABLE
#include "radar_uart.h"
#include "radar_oled_vis.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_LINE_TRACKING 1
#ifndef MAIN_STARTUP_DELAY_MS
#define MAIN_STARTUP_DELAY_MS  8000u
#endif
#ifndef LT_RADAR_BRANCH_ENABLE
#define LT_RADAR_BRANCH_ENABLE  1
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */
 

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aRxBuffer1; /* USART1：Dcar 协议 RX */
uint8_t aRxBuffer2; /* USART2：灰度传感器 */
#if LT_RADAR_BRANCH_ENABLE
uint8_t aRxBuffer4; /* USART4：毫米波雷达 */
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 重要：USART1 与 Dcar 底盘协议共用。若把 printf/puts 重定向到 USART1，会在二进制流里插入
 * 可打印字符，导致 Deal_DF_Usart 组帧错乱、校验失败，ODOM 永远解析不成功。
 * 0 = 丢弃标准输出（推荐接 Dcar 时；进一步减轻 USART3 负载可设 0）
 * 3 = 输出到 USART3（LoRa 无线串口，115200，与 EWM22A 共用物理口）
 */
#ifndef APP_STDIO_UART
/* 0=不向 USART3(LoRa) 打 printf；比赛经 LoRa 仅 lora_race_report。 */
#define APP_STDIO_UART  0
#endif

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
#if APP_STDIO_UART == 3
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 20);
#elif APP_STDIO_UART == 1
#error "Do not use USART1 for stdio — same link as Dcar (breaks ODOM framing)"
#else
  (void)ch;
#endif
  return ch;
}

int _write(int file, char *ptr, int len)
{
#if APP_STDIO_UART == 3
  if (file != 1 && file != 2)
    return -1;
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, 50);
#elif APP_STDIO_UART == 1
#error "Do not use USART1 for stdio — same link as Dcar (breaks ODOM framing)"
#else
  (void)file;
  (void)ptr;
#endif
  return len;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    DF_Uart1_ResetParser();
    (void)HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);
  }
  else if (huart->Instance == USART3)
  {
    EWM22A_OnUartError(huart);
  }
#if LT_RADAR_BRANCH_ENABLE
  else if (huart->Instance == USART4)
  {
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart4, &aRxBuffer4, 1);
  }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Deal_DF_Usart(aRxBuffer1);
    HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);
  }
  else if (huart->Instance == USART2)
  {
    Deal_IR_Usart(aRxBuffer2);
    HAL_UART_Receive_IT(&huart2, &aRxBuffer2, 1);
  }
  else if (huart->Instance == USART3)
  {
    EWM22A_OnUartRxCplt();
  }
#if LT_RADAR_BRANCH_ENABLE
  else if (huart->Instance == USART4)
  {
    RadarUart_OnRxByte(aRxBuffer4);
    HAL_UART_Receive_IT(&huart4, &aRxBuffer4, 1);
  }
#endif
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UserLED_GPIO_Init();
  {
    u8 i;
    for (i = 0; i < 3u; i++) {
      LED_Set(1);
      HAL_Delay(90);
      LED_Set(0);
      HAL_Delay(90);
    }
  }
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USART3（PA5/PB0）：EWM22A LoRa + 原 LPUART1 调试/printf/ODOM 转发（115200） */
  MX_USART3_UART_Init();
#if LT_RADAR_BRANCH_ENABLE
  MX_USART4_UART_Init();
#endif

  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);
  HAL_UART_Receive_IT(&huart2, &aRxBuffer2, 1);
#if DEBUG_ODOM_UART_ENABLE
  DebugUsart3_BootPing();
#endif
#if LT_RADAR_BRANCH_ENABLE
  HAL_UART_Receive_IT(&huart4, &aRxBuffer4, 1);
#endif
  EWM22A_StartReceiveIT();
  /* LoRa 联调：BWL 模块上电/AUX 就绪需更长时间，过短发串口可能被吞；电脑端应 115200 8N1 收无线透传 */
  HAL_Delay(2000);
  EWM22A_LinkTest_PingOnce();

  /* 复位/上电后先发零速，避免底盘仍执行复位前的速度指令；稍延时再跑协议与巡线 */
  HAL_Delay(50);
  sendVel_NoWait(0.0f, 0.0f, 0.0f);
  HAL_Delay(100);

  System_Init();
  Odom_Init();

  DS3231_InitAndSyncToBuildTime();

  HAL_Delay(100);
  IR_Send_Control_Data(0, 0, 1);
  HAL_Delay(50);

#if USE_LINE_TRACKING || ODOM_Y_1M_CALIB_TEST
  NavOdom_SetProfile(NAV_ODOM_PROFILE_DEFAULT);
#endif
#if USE_LINE_TRACKING && !ODOM_Y_1M_CALIB_TEST
  LineTracking_Init();
#endif
#if ODOM_Y_1M_CALIB_TEST
  OdomYCalib_Init();
#endif
#if OLED_ENABLE
  Oled_Init();
#endif
  HAL_Delay(MAIN_STARTUP_DELAY_MS);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if ODOM_Y_1M_CALIB_TEST
    OdomYCalib_Tick();
#elif USE_LINE_TRACKING
    LineTracking_Step();
#if LT_RADAR_BRANCH_ENABLE && OLED_ENABLE
    RadarVis_DrainPendingToWaterfall();
#endif
#endif
/* 不再经 USART3 周期打印 DS3231；LoRa 负载见 lora_race_report.c */
    EWM22A_LinkTest_Tick();
    EWM22A_PollDeferredLog();
    /* OLED 整帧 I2C 阻塞可达数十 ms；放在固定节拍 delay 之后，避免插在巡线计算与发速之间拉长本周期 dt */
    HAL_Delay(LINE_TRACK_LOOP_MS);
#if OLED_ENABLE
    Oled_Tick();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

extern TIM_HandleTypeDef htim7;

/**
  * @brief  Period elapsed callback in non blocking mode
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
