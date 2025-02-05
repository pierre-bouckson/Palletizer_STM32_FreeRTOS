/**
 ******************************************************************************
 * @file    Templates/Src/stm32f0xx_it.c
 * @author  MCD Application Team
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2016 STMicroelectronics.
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
#include "stm32f0xx_it.h"


/** @addtogroup STM32F0xx_HAL_Examples
 * @{
 */

/** @addtogroup Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
//void SVC_Handler(void)
//{
//}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
//void PendSV_Handler(void)
//{
//}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
//void SysTick_Handler(void)
//{
//HAL_IncTick();
//}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/


/**
 * This function handles EXTI line 13 interrupt request.
 */
extern xSemaphoreHandle xSem;

void EXTI4_15_IRQHandler()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Test for line 13 pending interrupt
	if ((EXTI->PR & EXTI_PR_PR13_Msk) != 0)
	{
		// Clear pending bit 13 by writing a '1'
		EXTI->PR = EXTI_PR_PR13;

		// Release the semaphore
		xSemaphoreGiveFromISR(xSem, &xHigherPriorityTaskWoken);

	}
}

extern uint8_t timebase_irq;

void TIM6_DAC_IRQHandler()
{
	// Test for TIM6 update pending interrupt
	if ((TIM6->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		// Clear pending interrupt flag
		TIM6->SR &= ~TIM_SR_UIF;

		// Do what you need
		timebase_irq = 1;
	}
}


extern xSemaphoreHandle xSem_DMA_TC;
void DMA1_Channel4_5_6_7_IRQHandler()
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Test for Channel 5 Transfer Complete
	if ((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
	{
		// Clear the interrupt pending bit
		DMA1->IFCR |= DMA_IFCR_CTCIF4;

		xSemaphoreGiveFromISR(xSem_DMA_TC, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	}
}



extern uint8_t	rtc_irq;

void RTC_IRQHandler()
{
	// Test for RTC Alarm A
	if ((RTC->ISR & RTC_ISR_ALRAF) == RTC_ISR_ALRAF)
	{
		// Clear the interrupt pending bit
		RTC->ISR &= ~RTC_ISR_ALRAF;

		// Clear EXTI pending bit also
		EXTI->PR = EXTI_PR_PR17;

		// Set global flag
		rtc_irq = 1;
	}
}

