/*
 * main.c
 *
 *  Created on: 28/02/2018
 *      Author: Laurent
 */




#include "main.h"

uint8_t timebase_irq;
uint8_t  console_rx_byte[10];
uint8_t	console_rx_irq;
uint8_t	rx_dma_irq;
uint8_t	rtc_irq;
uint8_t rx_dma_buffer[8];
xSemaphoreHandle xSem;


// Static functions
static void SystemClock_Config	(void);

// FreeRTOS tasks
void vTask1 (void *pvParameters);
void vTask2 (void *pvParameters);
void vTask3 (void *pvParameters);

// Kernel objects
EventGroupHandle_t myEventGroup;               // <-- Declare Event Group here

// Define Event Group flags
#define	BIT0	( (EventBits_t)( 0x01 <<0) )   // This is not mandatory but it provides
#define BIT1	( (EventBits_t)( 0x01 <<1) )   // friendly alias for individual event

// Trace User Events Channels
traceString ue1;

// Main function
int main()
{
	// Configure System Clock
	SystemClock_Config();

	// Initialize LED pin
	BSP_LED_Init();

	// Initialize Debug Console
	BSP_Console_Init();

	// Start Trace Recording
	xTraceEnable(TRC_START);

	// Create Event Group                   // <-- Create Event Group here
	myEventGroup = xEventGroupCreate();

	// Register the Trace User Event Channels
	ue1 = xTraceRegisterString("state");

	// Create Tasks
	xTaskCreate(vTask1, "Task_1",  256, NULL, 1, NULL);
	xTaskCreate(vTask2, "Task_2",  256, NULL, 2, NULL);
	xTaskCreate(vTask3, "Task_3",  256, NULL, 3, NULL);

	// Start the Scheduler
	vTaskStartScheduler();

	while(1)
	{
		// The program should never be here...
	}
}

/*
 *	Task_1 - State machine
 */
void vTask1 (void *pvParameters)
{
	uint8_t state;

	state = 0;

	while(1)
	{
		// LED toggle
		BSP_LED_Toggle();

		switch(state)
		{
			case 0:
			{
				vTracePrintF(ue1, "%d", state);
				xEventGroupClearBits(myEventGroup, BIT0 | BIT1);  // [0 0]

				state = 1;
				break;
			}

			case 1:
			{
				vTracePrintF(ue1, "%d", state);
				xEventGroupSetBits(myEventGroup, BIT0);          // [x 1]

				state = 2;
				break;
			}

			case 2:
			{
				vTracePrintF(ue1, "%d", state);
				xEventGroupSetBits(myEventGroup, BIT1);          // [1 x]

				state = 3;
				break;
			}

			case 3:
			{
				vTracePrintF(ue1, "%d", state);
				xEventGroupSetBits(myEventGroup, BIT0 | BIT1);  // [1 1]

				state = 0;
				break;
			}
		}

		// Wait for 2ms
		vTaskDelay(2);
	}
}
/*
 *	Task_2
 */
void vTask2 (void *pvParameters)
{
	while(1)
	{
		// Wait for myEventGroup :
		// - bit #0, bit #1
		// - Clear on Exit
		// - Wait for All bits (AND)
		xEventGroupWaitBits(myEventGroup, (BIT0 | BIT1), pdTRUE, pdTRUE, portMAX_DELAY);

		// If the bit is set
		my_printf("#");
	}
}

/*
 * Task 3
 */
void vTask3 (void *pvParameters)
{
	while(1)
	{
		// Wait for myEventGroup
		// - bit #0, bit #1
		// - Clear on Exit
		// - Do not Wait for All bits (OR)
		xEventGroupWaitBits(myEventGroup, (BIT0 | BIT1), pdTRUE, pdFALSE, portMAX_DELAY);

		// If the bit is set
		my_printf("-");
	}
}

static void SystemClock_Config()
{
	uint32_t	HSE_Status;
	uint32_t	PLL_Status;
	uint32_t	SW_Status;
	uint32_t	timeout = 0;
	timeout = 1000000;
	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;
	// Wait until HSE is ready
	do
	{
		HSE_Status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((HSE_Status == 0) && (timeout > 0));
	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);
	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;
	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);
	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;
	// Wait until PLL is ready
	do
	{
		PLL_Status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((PLL_Status == 0) && (timeout > 0));
    // Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;
	// Enable FLASH Prefetch Buffer and set Flash Latency
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
	/* --- Until this point, MCU was still clocked by HSI at 8MHz ---*/
	/* --- Switching to PLL at 48MHz Now!  Fasten your seat belt! ---*/
	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	// Wait until PLL becomes main switch input
	do
	{
		SW_Status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((SW_Status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	/*--- Use PA8 as LSE output ---*/
	// Set MCO source as LSE
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_LSE;     // Change here

	// No prescaler
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	SystemCoreClockUpdate();
}
