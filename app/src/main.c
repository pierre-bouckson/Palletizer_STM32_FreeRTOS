/*
 * main.c
 *
 *  Created on: 28/02/2018
 *      Author: Laurent
 */

#include "main.h"
#include "factory_io.h"
#include <string.h>

uint8_t timebase_irq;
uint8_t  console_rx_byte[10];
uint8_t	console_rx_irq;
uint8_t	rx_dma_irq;
uint8_t	rtc_irq;
uint8_t tx_dma_buffer[50];


uint8_t N = 4;

#define SENSOR_TABLE_SIZE 2
uint8_t sensor[SENSOR_TABLE_SIZE];

// Static functions
static void SystemClock_Config	(void);

// FreeRTOS tasks
void vTask1 		(void *pvParameters);
void vTask2 		(void *pvParameters);
void vTask3 		(void *pvParameters);
void Task_Pub 		(void *pvParameters);
void Task_Write		(void *pvParameters);

xQueueHandle xSubscribeQueue;
xQueueHandle xComQueue;
xSemaphoreHandle xSem;
xSemaphoreHandle xSem1;
xSemaphoreHandle xSem2;
xSemaphoreHandle xSem3;
xSemaphoreHandle xSem_UART_TC;
xSemaphoreHandle xSem_DMA_TC;

xTaskHandle			vTask1_handle;
xTaskHandle			vTask2_handle;
xTaskHandle			vTask3_handle;
typedef struct
{
	xSemaphoreHandle *xSem;
	uint32_t sensor_id; // Awaited sensor ID
	uint32_t sensor_state; // Awaited sensor State

}subscribe_message_t;

// Define the command_message_t type as an array of xx char
typedef struct{
	uint32_t cmd;
	uint8_t on_off;

}command_message_t;

// Kernel objects
EventGroupHandle_t myEventGroup;               // <-- Declare Event Group here

// Define Event Group flags
//#define	BIT0	( (EventBits_t)( 0x01 <<0) )   // This is not mandatory but it provides
///#define BIT1	( (EventBits_t)( 0x01 <<1) )   // friendly alias for individual event

// Trace User Events Channels
traceString ue1;

// Main function
int main()
{
	// Configure System Clock
	SystemClock_Config();
	// Initialize LED pin
	BSP_LED_Init();

	BSP_PB_Init();
	// Initialize Debug Console

	BSP_Console_Init();

	my_printf("Console Ready!\r\n");

	while(BSP_PB_GetState() == 0);

	//	// Read all states from the scene
	FACTORY_IO_update();

	// Start Trace Recording
	xTraceEnable(TRC_START);
	// Create Semaphore object (this is not a 'give')
	xSem = xSemaphoreCreateBinary();
	xSem1 = xSemaphoreCreateBinary();
	xSem2 = xSemaphoreCreateBinary();
	xSem3 = xSemaphoreCreateBinary();
	xSem_UART_TC = xSemaphoreCreateBinary();
	xSem_DMA_TC = xSemaphoreCreateBinary();

	xSubscribeQueue = xQueueCreate(4, sizeof(subscribe_message_t));
	xComQueue = xQueueCreate(4, sizeof(command_message_t));

	vTraceSetQueueName(xComQueue, "Write Queue");

	// Give a nice name to the Semaphore in the trace recorder
	vTraceSetSemaphoreName(xSem1, "xSEM1");
	vTraceSetSemaphoreName(xSem2, "xSEM2");
	vTraceSetSemaphoreName(xSem_UART_TC, "xSem_UART_TC");
	vTraceSetSemaphoreName(xSem_DMA_TC, "xSem_DMA_TC");

	// Create Tasks
	xTaskCreate(vTask1, "Task_1", 256, NULL, 2, &vTask1_handle);
	xTaskCreate(vTask2, "Task_2", 256, NULL, 1, &vTask2_handle);
	xTaskCreate(vTask3, "Task_3", 256, NULL, 1, &vTask3_handle);
	xTaskCreate(Task_Pub, "Task_Pub", 256, NULL, 3, NULL);
	xTaskCreate(Task_Write, "Task_Write", 256, NULL, 4, NULL);
	// Start the Scheduler
	vTaskStartScheduler();



	while(1)
	{
		// The program should never be here...
	}
}


void vTask1 (void *pvParameters)
{

	command_message_t message;
	subscribe_message_t msg;

	message.cmd = tapis_distrib_carton | tapis_CVP;
	message.on_off = 1;
	xQueueSendToBack(xComQueue, &message, 0);

	while(1)
	{

		message.cmd = distribution_carton;
		message.on_off = 1;
		xQueueSendToBack(xComQueue, &message, 0);

		vTaskDelay(100);

		message.cmd = distribution_carton;
		message.on_off = 0;
		xQueueSendToBack(xComQueue, &message, 0);

		msg.xSem = &xSem1;
		msg.sensor_id = carton_envoye;  //ID du capteur
		msg.sensor_state = 0; //Etat Attendu
		xQueueSendToBack(xSubscribeQueue, &msg, 0);

		xSemaphoreTake(xSem1,portMAX_DELAY);

		message.cmd = distribution_carton;
		message.on_off = 1;
		xQueueSendToBack(xComQueue, &message, 0);

		vTaskDelay(100);

		message.cmd = distribution_carton;
		message.on_off = 0;
		xQueueSendToBack(xComQueue, &message, 0);

		msg.xSem = &xSem1;
		msg.sensor_id = entre_pal;  //ID du capteur
		msg.sensor_state = 0; //Etat Attendu
		xQueueSendToBack(xSubscribeQueue, &msg, 0);

		xSemaphoreTake(xSem1,portMAX_DELAY);

		vTaskDelay(2000);

		msg.xSem = &xSem1;
		msg.sensor_id = entre_pal;  //ID du capteur
		msg.sensor_state = 0; //Etat Attendu
		xQueueSendToBack(xSubscribeQueue, &msg, 0);

		xSemaphoreTake(xSem1,portMAX_DELAY);

		xTaskNotifyGive(vTask2_handle);

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	}

}

void vTask2 (void *pvParameters)
{

	command_message_t message2;
	subscribe_message_t msg2;
	uint8_t count = 0;

	message2.cmd = charger_palettiseur | BEP;
	message2.on_off = 1;
	xQueueSendToBack(xComQueue, &message2, 0);

	while(1)
	{


		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		message2.cmd = BEP ;
		message2.on_off = 0;
		xQueueSendToBack(xComQueue, &message2, 0);

		vTaskDelay(3000);

		message2.cmd = BEP ;
		message2.on_off = 1;
		xQueueSendToBack(xComQueue, &message2, 0);

		xTaskNotifyGive(vTask1_handle);

		message2.cmd = poussoir;
		message2.on_off = 1;
		xQueueSendToBack(xComQueue, &message2, 0);



		vTaskDelay(2000);

		message2.cmd = poussoir;
		message2.on_off = 0;
		xQueueSendToBack(xComQueue, &message2, 0);

		vTaskDelay(2000);

		if(count < 3)
		{
			count++;
		}else
		{
			message2.cmd = clamp;
			message2.on_off = 0;
			xQueueSendToBack(xComQueue, &message2, 0);

			vTaskDelay(500000000);
		}
	}
}

void vTask3 (void *pvParameters)
{

	command_message_t message3;
	subscribe_message_t msg3;


	while(1)
	{

		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


	}

}


void Task_Pub (void *pvParameters)
{
	portTickType	xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	subscribe_message_t message = {NULL,0,0};

	portBASE_TYPE	xStatus;
	subscribe_message_t tab[N];
	memset(tab, 0, N*sizeof(subscribe_message_t));
	uint8_t exist = 0;
	uint8_t i;
	while(1)
	{

		for(i=0;i<N;i++)
		{
			if(tab[i].xSem != NULL)
			{
				if(tab[i].sensor_state == (FACTORY_IO_Sensors_Get() & tab[i].sensor_id))
				{

					xSemaphoreGive(*(tab[i].xSem));

					tab[i].xSem = NULL;
					tab[i].sensor_id = 0;
					tab[i].sensor_state = 0;

				}
			}
		}

		xStatus = xQueueReceive(xSubscribeQueue, &message, 0);
		exist = 0;

		// If there is something in USART RDR register message.sem_id != 0 || message.sensor_id != 0 || message.sensor_state != 0
		if(xStatus == pdPASS)
		{


			for(i=0;i<N;i++)
			{
				if(tab[i].xSem == message.xSem)
				{
					exist = 1;
					my_printf("Semaphome already exist \n\r");
					break;
				}
			}
			if(exist == 0)
			{
				for(i=0;i<N;i++)
				{
					if(tab[i].xSem == NULL)
					{
						tab[i].xSem = message.xSem;
						tab[i].sensor_id = message.sensor_id;
						tab[i].sensor_state = message.sensor_state;
						break;
					}
				}
			}
			message.xSem = NULL;
			message.sensor_id = 0;
			message.sensor_state = 0;
		}

		vTaskDelayUntil (&xLastWakeTime, (200/portTICK_RATE_MS));

	}
}

void Task_Write(void *pvParameters)
{

	command_message_t message;

	uint8_t buffer[7];

	// Set maximum priority for EXTI line 4 to 15 interrupts
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 9);

	// Enable EXTI line 4 to 15 (user button on line 13) interrupts
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

	while(1)
	{
		xQueueReceive(xComQueue, &message, portMAX_DELAY);

		FACTORY_IO_Actuators_Set(message.cmd, buffer);

		tx_dma_buffer[0] = buffer[0];
		tx_dma_buffer[5] = buffer[5];
		tx_dma_buffer[6] = buffer[6];

		if (message.on_off == 1){
			tx_dma_buffer[1] |= buffer[1];
			tx_dma_buffer[2] |= buffer[2];
			tx_dma_buffer[3] |= buffer[3];
			tx_dma_buffer[4] |= buffer[4];
		}
		if (message.on_off == 0){
			tx_dma_buffer[1] &= ~buffer[1];
			tx_dma_buffer[2] &= ~buffer[2];
			tx_dma_buffer[3] &= ~buffer[3];
			tx_dma_buffer[4] &= ~buffer[4];
		}

		DMA1_Channel4->CNDTR = 7;
		// Enable DMA1 Channel 5
		DMA1_Channel4->CCR |= DMA_CCR_EN;
		// Enable USART2 DMA Request on RT
		USART2->CR3 |= USART_CR3_DMAT;

		xSemaphoreTake(xSem_DMA_TC, portMAX_DELAY);

		DMA1_Channel4->CCR &= ~DMA_CCR_EN;
		// Enable USART2 DMA Request on RT
		USART2->CR3 &= ~USART_CR3_DMAT;
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
