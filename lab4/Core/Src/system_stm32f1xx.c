/**
  ******************************************************************************
  * @file    system_stm32f1xx.c
  * @author  MCD Application Team
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *
  * 1.  This file provides two functions and one global variable to be called from
  *     user application:
  *      - SystemInit(): Setups the system clock (System clock source, PLL Multiplier
  *                      factors, AHB/APBx prescalers and Flash settings).
  *                      This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32f1xx_xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  * 2. After each device reset the HSI (8 MHz) is used as system clock source.
  *    Then SystemInit() function is called, in "startup_stm32f1xx_xx.s" file, to
  *    configure the system clock before to branch to main program.
  *
  * 4. The default value of HSE crystal is set to 8 MHz (or 25 MHz, depending on
  *    the product used), refer to "HSE_VALUE".
  *    When HSE is used as system clock source, directly or through PLL, and you
  *    are using different crystal you have to adapt the HSE value to your own
  *    configuration.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include "stm32f1xx.h"

#if !defined(HSE_VALUE)
  #define HSE_VALUE    8000000U
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE    8000000U
#endif

uint32_t SystemCoreClock = 8000000;
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the
  *         SystemCoreClock variable.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* Nothing to configure here - clock is set up in SystemClock_Config() */
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  *         The SystemCoreClock variable contains the core clock (HCLK), it can
  *         be used by the user application to setup the SysTick timer or configure
  *         other parameters.
  *
  * @note   Each time the core clock (HCLK) changes, this function must be called
  *         to update SystemCoreClock variable value. Otherwise, any configuration
  *         based on this variable will be incorrect.
  *
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  uint32_t tmp = 0U, pllmull = 0U, pllsource = 0U;

  /* Get SYSCLK source */
  tmp = RCC->CFGR & RCC_CFGR_SWS;

  switch (tmp)
  {
    case 0x00U:  /* HSI used as system clock */
      SystemCoreClock = HSI_VALUE;
      break;
    case 0x04U:  /* HSE used as system clock */
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x08U:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor */
      pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
      pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;

      pllmull = (pllmull >> 18U) + 2U;

      if (pllsource == 0x00U)
      {
        /* HSI oscillator clock divided by 2 selected as PLL clock entry */
        SystemCoreClock = (HSI_VALUE >> 1U) * pllmull;
      }
      else
      {
        /* HSE selected as PLL clock entry */
        if ((RCC->CFGR & RCC_CFGR_PLLXTPRE) != (uint32_t)RESET)
        {
          /* HSE oscillator clock divided by 2 */
          SystemCoreClock = (HSE_VALUE >> 1U) * pllmull;
        }
        else
        {
          SystemCoreClock = HSE_VALUE * pllmull;
        }
      }
      break;

    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }

  /* Compute HCLK clock frequency */
  /* Get HCLK prescaler */
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4U)];
  /* HCLK clock frequency */
  SystemCoreClock >>= tmp;
}

