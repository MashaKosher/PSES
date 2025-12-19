/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : IR Remote Control Decoder
 * @description    : Decodes NEC protocol IR signals and sends them via UART
 *                   to a Python application for monitoring.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include "cmsis_os2.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define IR_BUFFER_SIZE 68
#define UART_TX_BUFFER_SIZE 64

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* IR decoder variables */
/* Use 16-bit timings to save SRAM (max NEC pulse < 10ms -> fits in 0..65535us) */
volatile uint16_t ir_buffer[IR_BUFFER_SIZE];
volatile uint8_t ir_index = 0;
volatile bool ir_data_ready = false;

static uint32_t last_edge_time = 0;
static bool first_edge = true;
static uint32_t last_activity = 0;

/* UART transmission */
volatile bool uart_busy = false;
static char tx_buffer[UART_TX_BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

static uint32_t Micros(void);
static void SendIRCode(uint32_t code);
static void ProcessIRData(void);

static void Hex8(uint32_t value, char *out);
static void Hex2(uint8_t value, char *out);

/* RTOS objects */
static osThreadId_t irTaskHandle;

/* RTOS task prototypes */
static void irTask(void const *argument);

/* Signal flags */
#define IR_SIGNAL_DATA_READY (1U << 0)

static const osThreadAttr_t irTaskAttr = {
    .name = "irTask",
    .priority = osPriorityNormal,
    .stack_size = 384};

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Enable DWT for microsecond timing */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* Send startup message */
  const char *startup_msg = "IR Decoder Ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)startup_msg, strlen(startup_msg), 100);

  /* RTOS init */
  osKernelInitialize();

  irTaskHandle = osThreadNew((osThreadFunc_t)irTask, NULL, &irTaskAttr);

  /* Start scheduler (should not return) */
  osKernelStart();

  /* Fallback loop */
  while (1) {}
}

/**
 * @brief RTOS task: waits for notification from EXTI ISR and decodes IR frame.
 */
static void irTask(void const *argument) {
  (void)argument;

  for (;;) {
    (void)osThreadFlagsWait(IR_SIGNAL_DATA_READY, osFlagsWaitAny, osWaitForever);
    if (ir_data_ready) {
      ProcessIRData();
      ir_data_ready = false;
    }
  }
}

/**
 * @brief  Process captured IR timing data and decode NEC protocol
 */
static void ProcessIRData(void) {
  uint32_t code = 0;
  int bit_count = 0;

  /* NEC protocol validation:
   * Start pulse: ~9ms burst
   * Header space: ~4.5ms for data, ~2.25ms for repeat
   */
  if (ir_buffer[0] > 7000 && ir_buffer[0] < 11000 && ir_buffer[1] > 3000 &&
      ir_buffer[1] < 6000) {
    /* Decode 32 bits (address + inverse address + command + inverse command) */
    for (int i = 2; i < 66; i += 2) {
      uint32_t space = ir_buffer[i + 1];

      if (space > 1200 && space < 2100) {
        /* Logic 1: ~1.6875ms space */
        code = (code << 1) | 1;
        bit_count++;
      } else if (space > 300 && space < 900) {
        /* Logic 0: ~562.5us space */
        code = (code << 1);
        bit_count++;
      } else {
        /* Invalid timing */
        bit_count = 0;
        break;
      }
    }

    if (bit_count == 32) {
      SendIRCode(code);
    }
  } else if (ir_buffer[0] > 7000 && ir_buffer[0] < 11000 &&
             ir_buffer[1] > 2000 && ir_buffer[1] < 2500) {
    /* NEC repeat code */
    const char *repeat_msg = "REPEAT\r\n";
    if (!uart_busy) {
      uart_busy = true;
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)repeat_msg, strlen(repeat_msg));
    }
  }
}

/**
 * @brief  Send decoded IR code via UART in hex format
 * @param  code: 32-bit decoded IR code
 */
static void SendIRCode(uint32_t code) {
  if (uart_busy)
    return;

  /* Extract NEC protocol fields */
  uint8_t addr = (code >> 24) & 0xFF;
  uint8_t cmd = (code >> 8) & 0xFF;

  /* Format without printf to save SRAM/ROM (avoid libspace/stdio overhead) */
  char *p = tx_buffer;
  memcpy(p, "CODE:0x", 7); p += 7;
  Hex8(code, p); p += 8;
  memcpy(p, " ADDR:0x", 8); p += 8;
  Hex2(addr, p); p += 2;
  memcpy(p, " CMD:0x", 7); p += 7;
  Hex2(cmd, p); p += 2;
  memcpy(p, "\r\n", 2); p += 2;
  *p = '\0';

  uart_busy = true;
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buffer, strlen(tx_buffer));
}

static void Hex8(uint32_t value, char *out) {
  static const char hex[] = "0123456789ABCDEF";
  for (int i = 0; i < 8; i++) {
    out[7 - i] = hex[value & 0xFU];
    value >>= 4;
  }
}

static void Hex2(uint8_t value, char *out) {
  static const char hex[] = "0123456789ABCDEF";
  out[0] = hex[(value >> 4) & 0xFU];
  out[1] = hex[value & 0xFU];
}

/**
 * @brief  Get current time in microseconds using DWT
 * @retval Microseconds since startup
 */
static uint32_t Micros(void) { return DWT->CYCCNT / 72; /* 72 MHz clock */ }

/**
 * @brief  GPIO EXTI callback for IR receiver
 * @param  GPIO_Pin: Specifies the pin connected to EXTI line
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin != IR_RECEIVER_Pin)
    return;
  if (ir_data_ready)
    return;

  uint32_t now = Micros();

  /* Reset on timeout (100ms gap indicates new transmission) */
  if (now - last_activity > 100000) {
    ir_index = 0;
    first_edge = true;
  }
  last_activity = now;

  if (first_edge) {
    last_edge_time = now;
    first_edge = false;
    return;
  }

  uint32_t duration = now - last_edge_time;
  last_edge_time = now;

  /* Filter noise (< 100us) */
  if (duration < 100)
    return;

  if (ir_index < IR_BUFFER_SIZE) {
    if (duration > 0xFFFFU) {
      duration = 0xFFFFU;
    }
    ir_buffer[ir_index++] = (uint16_t)duration;
  }

  /* NEC protocol: 67 edges for full transmission */
  if (ir_index >= 67) {
    ir_data_ready = true;
    ir_index = 0;
    first_edge = true;

    /* Wake decoding task (if scheduler is running) */
    if (osKernelGetState() == osKernelRunning) {
      (void)osThreadFlagsSet(irTaskHandle, IR_SIGNAL_DATA_READY);
    }
  }
}

/**
 * @brief  UART TX complete callback
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    uart_busy = false;
  }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow:
 *         System Clock source = PLL (HSE)
 *         SYSCLK(Hz)          = 72000000
 *         HCLK(Hz)            = 72000000
 *         AHB Prescaler       = 1
 *         APB1 Prescaler      = 2
 *         APB2 Prescaler      = 1
 *         HSE Frequency(Hz)   = 8000000
 *         PLL_MUL             = 9
 *         Flash Latency(WS)   = 2
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  USART2 Initialization Function
 * @param  None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

/**
 * @brief  GPIO Initialization Function
 * @param  None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure IR Receiver pin: PB10 with interrupt on rising and falling edges
   */
  GPIO_InitStruct.Pin = IR_RECEIVER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_RECEIVER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init */
  /* Note: with RTOS, any interrupt that calls osSignalSet/osSemaphoreRelease from ISR
   * must have a priority numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
   * CubeMX+FreeRTOS typically uses priorities like 5..15 for such IRQs. */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}

/* -------------------------------------------------------------------------- */
/* HAL tick overrides for RTOS                                                */
/* -------------------------------------------------------------------------- */

uint32_t HAL_GetTick(void) {
  /* When RTX owns SysTick, use the RTOS tick count for HAL time base. */
  return (uint32_t)osKernelGetTickCount();
}

void HAL_Delay(uint32_t Delay) {
  /* Let RTOS handle delays when running; fallback to busy-wait before start. */
  if (osKernelGetState() == osKernelRunning) {
    (void)osDelay(Delay);
    return;
  }
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < Delay) {}
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority) {
  /* RTX configures SysTick; prevent HAL from reconfiguring it. */
  (void)TickPriority;
  return HAL_OK;
}

void HAL_SuspendTick(void) {}
void HAL_ResumeTick(void) {}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
}
#endif
