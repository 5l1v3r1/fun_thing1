/*
 * nfcTask.c
 *
 *  Created on: 2014. 1. 22.
 *      Author: jongwonk
 */

#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "debug.h"

#include "stm32f4xx.h"

#include "utils.h"

#include "nfc/nfc.h"

#include "nfc_task.h"

#define RX3_QUEUE_SIZE 64
#define TX3_QUEUE_SIZE 3

#define _TESTING_

static void nfc_uart3_config(void);
static void DMA_usart3_Configuration(void);
void USARTx_Send(DMA_Stream_TypeDef* DMAy_Streamx, uint8_t* buf, int len);

static void vNFCTxTask(void* pvParameters );
static void vNFCTask(void *vParameter);

portBASE_TYPE usart3_readSingleByte(uint8_t* ch,portTickType timeout);

static xSemaphoreHandle xSemaphoreTx;
xQueueHandle xQueueNFCRx;
static xQueueHandle xQueueNFCTx;

static nfc_device *pnd = NULL;
static nfc_context *context;

void initNFC(void){

	nfc_uart3_config();
	DMA_usart3_Configuration();

	xSemaphoreTx = xSemaphoreCreateBinary();
	xQueueNFCRx = xQueueCreate( RX3_QUEUE_SIZE, sizeof(uint8_t));
	xQueueNFCTx = xQueueCreate( TX3_QUEUE_SIZE, sizeof(TQ));
}

void createNFCTask(void){

	// nfc_exti0_config(); //IRQ from PN532

	xTaskCreate( vNFCTxTask, ( signed char * ) "nfcTx", configMINIMAL_STACK_SIZE*2, ( void * ) NULL, (tskIDLE_PRIORITY+3)| portPRIVILEGE_BIT, NULL );
	xTaskCreate( vNFCTask,   ( signed char * ) "NFC",   configMINIMAL_STACK_SIZE*10, ( void * ) NULL, (tskIDLE_PRIORITY+2)| portPRIVILEGE_BIT, NULL );
}

void NFC_Send(uint8_t* tx_data,uint16_t len)
{
	TQ tq;

	assert(xQueueNFCTx);

	memcpy(tq.data,tx_data,len);
	tq.length = len;

	while(xQueueSend(xQueueNFCTx,&tq,(portTickType)1)==pdFALSE);
}

portBASE_TYPE NFC_ReadByte(USART_TypeDef* USARTx, uint8_t* rcvdByte,portTickType timeout){
	return usart3_readSingleByte(rcvdByte,timeout);
}

void vNFCTxTask(void* pvParameters ) {
	TQ tq;

	vDebugString("NFC TX task started");

	vTaskDelay(500);

	for(;;) {

		if(xQueueReceive( xQueueNFCTx, &tq, (portTickType)portMAX_DELAY) == pdPASS){
			USARTx_Send(DMA1_Stream3,tq.data,tq.length*sizeof(uint8_t));

			if(xSemaphoreTake(xSemaphoreTx,(portTickType)100) != pdTRUE){
				vDebugString("NFC TX task timeout");
			}
		}
		else
		{
			vTaskDelay(50);
		}
	}
}

void vNFCTask(void *vParameter){

	uint8_t ch1 = 0x55;

// BEGINNING OF TEST CODE
	while(1)
	{
		USARTx_Send(DMA1_Stream3,&ch1,sizeof(uint8_t));
		vTaskDelay(1000);
	}
// END OF TEST CODE


	vDebugString("vNFCTask started");

	vTaskDelay(1000);

	nfc_init(&context);

	pnd = nfc_open(context);

	nfc_close(pnd);

	nfc_exit(context);

	for(;;){

			vTaskDelay(1000);
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

	/* GPIOD Configuration: USART3 TX on PD8 and RX on PD9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//GPIO_PuPd_NOPULL ;
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

	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE); // enable USART3

	/* USART Clock Initialization  */
	USART_ClockInitstructure.USART_Clock   = USART_Clock_Disable ;
	USART_ClockInitstructure.USART_CPOL    = USART_CPOL_Low ;
	USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInitstructure.USART_CPHA    = USART_CPHA_1Edge;

	USART_ClockInit(USART3, &USART_ClockInitstructure);

	// USART IRQ init
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	// Enable the USART3 TX DMA Interrupt
	NVIC_Init(&NVIC_InitStructure);

	// USART IRQ init
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	// Enable the USART3 RX DMA Interrupt
	NVIC_Init(&NVIC_InitStructure);


}

uint8_t usart3_rx_fifo_single_buffer;
uint8_t ch1;

portBASE_TYPE usart3_readSingleByte(uint8_t* ch,portTickType timeout){
	  if(xQueueReceive( xQueueNFCRx, ch , timeout) == pdPASS){
		  vDebugString("rx p");
		  return pdPASS;
	  }

	  vDebugString("rx f");
	  return pdFAIL;
}

void DMA_usart3_Configuration(void)
{
  DMA_InitTypeDef DMA_RX_InitStructure;
  DMA_InitTypeDef DMA_TX_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Stream3);
  DMA_TX_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_TX_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; // Transmit
  DMA_TX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_TX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_TX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_TX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_TX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_TX_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_TX_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_TX_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_TX_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_TX_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_TX_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_Init(DMA1_Stream3, &DMA_TX_InitStructure);

  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC, ENABLE);
  DMA_Cmd(DMA1_Stream3, ENABLE);

  DMA_DeInit(DMA1_Stream1);
  DMA_RX_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_RX_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
  DMA_RX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_RX_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_RX_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_RX_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_RX_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_RX_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_RX_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_RX_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  DMA_RX_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_RX_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_RX_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

  DMA_RX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&usart3_rx_fifo_single_buffer;
  DMA_RX_InitStructure.DMA_BufferSize = (uint16_t)1;

  DMA_Init(DMA1_Stream1, &DMA_RX_InitStructure);

  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);

  DMA_Cmd(DMA1_Stream1, ENABLE);

}

void USARTx_Send(DMA_Stream_TypeDef* DMAy_Streamx, uint8_t* buf, int len){

    DMA_Cmd(DMAy_Streamx, DISABLE);
    while(DMA_GetCmdStatus(DMAy_Streamx)!= DISABLE); // wait until it is disabled
    DMAy_Streamx->M0AR = (uint32_t)buf;
    DMAy_Streamx->NDTR = (uint16_t)len;
    DMA_Cmd(DMAy_Streamx, ENABLE);
}

void DMA1_Stream3_IRQHandler(void){ // USART TX

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3))
    {
        DMA_ClearITPendingBit(DMA1_Stream3,DMA_IT_TCIF3);
        xSemaphoreGiveFromISR(xSemaphoreTx,&xHigherPriorityTaskWoken);
    }
}

void DMA1_Stream1_IRQHandler(void){ // USART RX

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
    {
    	xQueueSendFromISR(xQueueNFCRx,&usart3_rx_fifo_single_buffer,&xHigherPriorityTaskWoken);
        DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);

    }
}

