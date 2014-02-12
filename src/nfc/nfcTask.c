/*
 * nfcTask.c
 *
 *  Created on: 2014. 1. 22.
 *      Author: jongwonk
 */

#include <stdarg.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"

#include "utils.h"

#define _TESTING_

static void nfc_uart3_config(void);
static void uart_send(uint8_t byte);

static void nfc_exti0_config(void);
static void vNFCTask(void *vParameter);

xSemaphoreHandle nfc_task_rxEvent;

void createNFCTask(){

	nfc_exti0_config(); //IRQ from PN532
	nfc_task_rxEvent = xSemaphoreCreateBinary();

	xTaskCreate( vNFCTask, ( signed char * ) "NFCTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, (tskIDLE_PRIORITY+1)| portPRIVILEGE_BIT, NULL );

}

void vNFCTask(void *vParameter){

	nfc_uart3_config();

	for(;;){

#ifdef _TESTING_
		uart_send(0x55);
		vTaskDelay(100);
#endif


	}
}

void nfc_uart3_config(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitstructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* GPIOD Configuration:  USART3 TX on PD8 and RX on PD9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect USART3 pins to AF */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3); // TX = PD8
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3); // RX = PD9

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

/*
	/ USART Clock Initialization
	USART_ClockInitstructure.USART_Clock   = USART_Clock_Disable ;
	USART_ClockInitstructure.USART_CPOL    = USART_CPOL_Low ;
	USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInitstructure.USART_CPHA    = USART_CPHA_1Edge;
	USART_ClockInit(USART3, &USART_ClockInitstructure);
*/
	// USART IRQ init
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART3, ENABLE); // enable USART3
}

void USART3_IRQHandler(void)
{
	portBASE_TYPE pxHigherPriorityTaskWokenv = pdFALSE;

	if( USART_GetITStatus(USART3, USART_IT_RXNE))
	{
	  uint8_t c = USART3->DR;
	  xQueueSendFromISR(nfc_task_rxEvent,&c,&pxHigherPriorityTaskWokenv);
	  USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	}

}

void uart_send(uint8_t byte)
{
	USART_SendData(USART3,byte);
   // while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
   // USART_SendData(USART3, byte);
}

void nfc_exti0_config(void){

    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    /* Enable GPIOD clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Connect EXTI Line0 to PG10 pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);

    /* Configure PD10 pin as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
{
	portBASE_TYPE prio = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line0) != RESET) //check if EXTI line is asserted
	{
		EXTI_ClearFlag(EXTI_Line0); //clear interrupt

		// to do something
	}
}


