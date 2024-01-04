/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>

#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bq24072.h"

#ifdef ENABLE_EMULATOR_GB
#if FORCE_GNUBOY == 0
#include "main_gb_tgbdual.h"
#endif
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///////////////////////////////////////////////////////////////////////////////
// Inspired by
// Source: https://github.com/memfault/interrupt/blob/master/example/cortex-m-fault-debug/startup.c
// License: Content released under CC-by-SA license
// Copyright (c) 2019 Memfault, Inc.

typedef struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
} sContextStateFrame;

__attribute__((optimize("O0")))
void common_fault_handler_c(sContextStateFrame *frame, int type)
{
  BSOD(type, frame->return_address, frame->lr);
}

#define HARDFAULT_HANDLING_ASM(_x)               \
  __asm volatile(                                \
      "tst lr, #4 \n"                            \
      "ite eq \n"                                \
      "mrseq r0, msp \n"                         \
      "mrsne r0, psp \n"                         \
      "movs r1, %0 \n"                           \
      "b common_fault_handler_c \n"              \
      : : "r" (_x)                               )

///////////////////////////////////////////////////////////////////////////////


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern LTDC_HandleTypeDef hltdc;
extern OSPI_HandleTypeDef hospi1;
extern DMA_HandleTypeDef hdma_sai1_a;
extern SAI_HandleTypeDef hsai_BlockA1;
extern TIM_HandleTypeDef htim1;
extern WWDG_HandleTypeDef hwwdg1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM(BSOD_HARDFAULT);
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  HARDFAULT_HANDLING_ASM(BSOD_MEMFAULT);
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM(BSOD_BUSFAULT);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  HARDFAULT_HANDLING_ASM(BSOD_USAGEFAULT);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Window watchdog interrupt.
  */
void WWDG_IRQHandler(void)
{
  /* USER CODE BEGIN WWDG_IRQn 0 */

  /* USER CODE END WWDG_IRQn 0 */
  HAL_WWDG_IRQHandler(&hwwdg1);
  /* USER CODE BEGIN WWDG_IRQn 1 */

  /* USER CODE END WWDG_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_2))
  {
    bq24072_handle_power_good();
  }

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_sai1_a);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_7))
  {
    bq24072_handle_charging();
  }

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_7);

  /* USER CODE END EXTI9_5_IRQn 1 */
}

// FIXME Move into shared header
static inline void delay_us(volatile uint32_t microseconds)
{
    uint32_t au32_initial_ticks = DWT->CYCCNT;
    uint32_t au32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);
    microseconds *= au32_ticks;
    while ((DWT->CYCCNT - au32_initial_ticks) < microseconds-au32_ticks);
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
#ifdef ENABLE_EMULATOR_GB
#if FORCE_GNUBOY == 0

  if(__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_14))
  {

    // FIXME Disable ALL IRQs to avoid being cut during transfer
    __disable_irq();

    // FIXME Remove debug spike
    // Signal IRQ start
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

    // Sample line every <n> microseconds
    uint8_t data = 0;
    delay_us(2);//(4);

    // FIXME Remove debug spike
    // Signal reading start
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

    for (int i=0; i<8; i++) {
      data = data << 1;
      data |= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_14) & 0x01;
      
      // FIXME Remove debug spike
      // Signal IRQ bit read
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
      delay_us(1);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
      
      delay_us(3);//(2);
    }

    // Send current SB value
    // This function also updates current SB value and raises GB serial interrupt
    uint8_t response = handle_incoming_serial_data(data);
    uint8_t resp = response;
    
    delay_us(4);//(5);

    // FIXME Remove debug spike
    // Signal IRQ response start
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

    // Write bits
    for (int i=0; i<8; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, (GPIO_PinState) ((response & 0x80) >> 7));
        delay_us(4);
        response = response << 1;
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);

    // FIXME Remove debug spike
    // Signal IRQ end
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
    delay_us(1);

    //printf("SERIAL IRQ: recv=0x%x resp=0x%x\n", data, resp);

    // FIXME Re-enable ALL IRQs
    __enable_irq();
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
  }

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_14);

#endif
#endif
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  uptime_inc();

  bq24072_poll();

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles SAI1 global interrupt.
  */
void SAI1_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_IRQn 0 */

  /* USER CODE END SAI1_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA1);
  /* USER CODE BEGIN SAI1_IRQn 1 */

  /* USER CODE END SAI1_IRQn 1 */
}

/**
  * @brief This function handles LTDC global interrupt.
  */
void LTDC_IRQHandler(void)
{
  /* USER CODE BEGIN LTDC_IRQn 0 */

  /* USER CODE END LTDC_IRQn 0 */
  HAL_LTDC_IRQHandler(&hltdc);
  /* USER CODE BEGIN LTDC_IRQn 1 */

  /* USER CODE END LTDC_IRQn 1 */
}

/**
  * @brief This function handles OCTOSPI1 global interrupt.
  */
void OCTOSPI1_IRQHandler(void)
{
  /* USER CODE BEGIN OCTOSPI1_IRQn 0 */

  /* USER CODE END OCTOSPI1_IRQn 0 */
  HAL_OSPI_IRQHandler(&hospi1);
  /* USER CODE BEGIN OCTOSPI1_IRQn 1 */

  /* USER CODE END OCTOSPI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
