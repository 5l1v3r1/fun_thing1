/*
 * ethernetTask.c
 *
 *  Created on: 2014. 2. 17.
 *      Author: jongwonk
 */

#include "stm32f4xx.h"
#include "ethernetTask.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static xTaskHandle xEthernet;
uint8_t aTxBuffer[ETH_BUFFERSIZE];
uint8_t aRxBuffer[ETH_BUFFERSIZE];

void vEthernetTask(void* pvParameters );


void createEthernetTask(void){

	xTaskCreate(vEthernetTask,(char*) "EthernetTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, (tskIDLE_PRIORITY+1)| portPRIVILEGE_BIT, &xEthernet );

}

void vEthernetTask(void* pvParameters )
{

	while(1){

		vTaskDelay(1000);

	}

}


void spi2_config(){
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	/* Peripheral Clock Enable -------------------------------------------------*/

	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* Enable GPIOB clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Enable DMA clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	/* SPI GPIO Configuration --------------------------------------------------*/
	/* GPIO Deinitialisation */
	GPIO_DeInit(GPIOB);

	/* Connect SPI pins to AF5 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/* SPI NSS pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI  MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	/* DMA configuration -----------------------------------------*/
	/* Deinitialize DMA Streams */
	DMA_DeInit(DMA1_Stream4);
	DMA_DeInit(DMA1_Stream3);

	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = ETH_BUFFERSIZE ;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR)) ;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0 ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer ;
	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
	/* Configure RX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_0 ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aRxBuffer ;
	DMA_Init(DMA1_Stream3, &DMA_InitStructure);
}
