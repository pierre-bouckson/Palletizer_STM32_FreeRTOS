/*
 * main.c
 *
 *  Created on: 24/02/2018
 *      Author: Laurent
 */

#include "main.h"

uint8_t rx_dma_buffer[8];
// Define the message_t type as an array of 64 char
typedef uint8_t msg_t[64];

// Kernel Objects
xSemaphoreHandle xConsoleMutex;

// Kernel Objects
xQueueHandle	xConsoleQueue;

// Static functions
static void SystemClock_Config	(void);

// FreeRTOS tasks
void vTask1 		(void *pvParameters);
void vTask2 		(void *pvParameters);
void vTaskConsole 	(void *pvParameters);

// Main program
int main()
{
	// Configure System Clock
	SystemClock_Config();

	// Initialize LED pin
	BSP_LED_Init();

	// Initialize the user Push-Button
	BSP_PB_Init();

	// Initialize Debug Console
	BSP_Console_Init();

	// Start Trace Recording
	vTraceEnable(TRC_START);

	// Create a Mutex for accessing the console
	xConsoleMutex = xSemaphoreCreateMutex();

	// Give a nice name to the Mutex in the trace recorder
	vTraceSetMutexName(xConsoleMutex, "Console Mutex");

    // Create Queue to hold console messages
	xConsoleQueue = xQueueCreate(4, sizeof(msg_t));

	// Give a nice name to the Queue in the trace recorder
	vTraceSetQueueName(xConsoleQueue, "Console Queue");

	// Create Tasks
	xTaskCreate(vTask1, 		"Task_1",       256, NULL, 3, NULL);
	xTaskCreate(vTask2, 		"Task_2",       256, NULL, 2, NULL);
	xTaskCreate(vTaskConsole, 	"Task_Console", 256, NULL, 1, NULL);

	// Start the Scheduler
	vTaskStartScheduler();

	while(1)
	{
		// The program should never be here...
	}
}

/*
 *	Task_1
 */
void vTask1 (void *pvParameters)
{
	msg_t 	msg;

	while(1)
	{
		// Prepare message
		my_sprintf((char *)msg, "With great power comes great responsibility\r\n");

		// Send message to the Console Queue
		xQueueSendToBack(xConsoleQueue, &msg, 0);

		// Wait for 20ms
		vTaskDelay(20);
	}
}

/*
 *	Task_2
 */
void vTask2 (void *pvParameters)
{
	msg_t 	msg;
	uint8_t	index = 0;

	while(1)
	{
		// Prepare message
		my_sprintf((char *)msg, "%d# ", index);

		// Send message to Console Queue
		xQueueSendToBack(xConsoleQueue, &msg, 0);

		// Increment index
		(index==9) ? index=0 : index++;

		// Wait for 2ms
		vTaskDelay(2);
	}
}

/*
 * Task_Console
 */
void vTaskConsole (void *pvParameters)
{
	msg_t msg;

	while(1)
	{
		// Wait for something in the message Queue
		xQueueReceive(xConsoleQueue, &msg, portMAX_DELAY);

		// Send message to console
		my_printf((char *)msg);
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
