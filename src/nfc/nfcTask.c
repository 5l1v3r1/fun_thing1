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

#define RX3_QUEUE_SIZE 32
#define TX3_QUEUE_SIZE 3

#define _TESTING_

static void nfc_uart3_config(void);
static void DMA_usart3_Configuration(void);
void USARTx_Send(DMA_Stream_TypeDef* DMAy_Streamx, uint8_t* buf, int len);

static void nfc_exti0_config(void);
static void vNFCTxTask(void* pvParameters );
static void vNFCTask(void *vParameter);

portBASE_TYPE usart3_readSingleByte(uint8_t* ch,portTickType timeout);

static xSemaphoreHandle xSemaphoreTx;
xQueueHandle xQueueNFCRx;
static xQueueHandle xQueueNFCTx;

static nfc_device *pnd = NULL;
static nfc_context *context;

extern xQueueHandle rxq;

void initNFC(void){

	xSemaphoreTx = xSemaphoreCreateBinary();
	xQueueNFCRx = xQueueCreate( RX3_QUEUE_SIZE, sizeof(uint8_t));
	xQueueNFCTx = xQueueCreate( TX3_QUEUE_SIZE, sizeof(TQ));
	rxq = xQueueCreate(10,sizeof(char));
}

void createNFCTask(void){

	// nfc_exti0_config(); //IRQ from PN532

	xTaskCreate( vNFCTxTask, ( signed char * ) "nfcTx", configMINIMAL_STACK_SIZE*2, ( void * ) NULL, (tskIDLE_PRIORITY+3)| portPRIVILEGE_BIT, NULL );
	xTaskCreate( vNFCTask,   ( signed char * ) "NFC",   configMINIMAL_STACK_SIZE*10, ( void * ) NULL, (tskIDLE_PRIORITY+2)| portPRIVILEGE_BIT, NULL );
}

void NFC_Send(uint8_t* tx_data,uint16_t len)
{
	TQ tq1;
	TQ* ptq = &tq1;

	assert(xQueueNFCTx);

	memcpy(tq1.data,tx_data,len);
	tq1.length = len;

	while(xQueueSend(xQueueNFCTx,ptq,(portTickType)0)==pdFALSE);
}

portBASE_TYPE NFC_ReadByte(USART_TypeDef* USARTx, uint8_t* rcvdByte,portTickType timeout){
	// return xQueueReceive(xQueueNFCRx,rcvdByte,timeout);
	return usart3_readSingleByte(rcvdByte,timeout);
}

void vNFCTxTask(void* pvParameters ) {
	TQ tq;
	portBASE_TYPE xStatus;

	vDebugString("NFC TX task started");

	for(;;) {

		if(xSemaphoreTake(xSemaphoreTx,(portTickType)50) != pdTRUE){
			xStatus = xQueueReceive( xQueueNFCTx, &tq, (portTickType)portMAX_DELAY);
			USARTx_Send(DMA1_Stream3,tq.data,tq.length*sizeof(uint8_t));
		}
	}
}

void vNFCTask(void *vParameter){

	nfc_uart3_config();
	DMA_usart3_Configuration();

	vDebugString("vNFCTask started");


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

	/* enable peripheral clock for USART3 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* GPIOD Configuration: USART3 TX on PD8 and RX on PD9 */
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

	/* USART configuration */
	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE); // enable USART3
}

static uint8_t usart3_rx_fifo_single_buffer;
uint8_t ch1;

portBASE_TYPE usart3_readSingleByte(uint8_t* ch,portTickType timeout){
	  if(xQueueReceive( xQueueNFCRx, ch , timeout) == pdPASS){
		  return pdPASS;
	  }

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
  DMA_TX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
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
  DMA_RX_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
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

  DMA_RX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&usart3_rx_fifo_single_buffer);
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
        DMA_ClearITPendingBit(DMA1_Stream1,DMA_IT_TCIF1);
        xQueueSendFromISR(xQueueNFCRx,&usart3_rx_fifo_single_buffer,&xHigherPriorityTaskWoken);
    }
}

void nfc_exti0_config(void){

    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

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
