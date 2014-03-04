/*
 * debug.c
 *
 *  Created on: Jan. 5, 2012
 *      Author: James Kemp
 */
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f4xx.h"
#include "debug.h"
#include "utils.h"

static DMA_InitTypeDef  DMA_TX_InitStructure;
static xSemaphoreHandle xSemaphore;

// Total buffer size for all debug messages.
#define DEBUG_QUEUE_SIZE	64

static xQueueHandle xDebugQueue;
static xTaskHandle hDebugTask;

void debug_uart2_config(void);
void DMA_usart2_Configuration(void);

void vNum2String( char *s, uint8_t *pPos, uint32_t u32Number, uint8_t u8Base);

void InitDebug(void)
{
	debug_uart2_config();
	DMA_usart2_Configuration();

	vDebugInitQueue();

	xSemaphore = xSemaphoreCreateBinary();
}

void debug_uart2_config(void){

	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitstructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* enable peripheral clock for USART2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* GPIOA clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOA Configuration:  USART2 TX on PA2 and RX on PA3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect USART2 pins to AF */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // TX = PA2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // RX = PA3

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;

	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE); // enable USART2

	/* USART Clock Initialization  */
	USART_ClockInitstructure.USART_Clock   = USART_Clock_Disable ;
	USART_ClockInitstructure.USART_CPOL    = USART_CPOL_Low ;
	USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInitstructure.USART_CPHA    = USART_CPHA_1Edge;

	USART_ClockInit(USART2, &USART_ClockInitstructure);

	// USART IRQ init
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	// Enable the USART2 TX DMA Interrupt
	NVIC_Init(&NVIC_InitStructure);

	// Enable the USART2 RX DMA Interrupt
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
//	NVIC_Init(&NVIC_InitStructure);
}

void DEBUG_usart_send(uint8_t* buf, int len){

    DMA_Cmd(DMA1_Stream6, DISABLE);
    while(DMA_GetCmdStatus(DMA1_Stream6)!= DISABLE); // wait until it is disabled
      DMA1_Stream6->M0AR = (uint32_t)buf;
      DMA1_Stream6->NDTR = (uint16_t)len;
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

void DMA_usart2_Configuration(void)
{
  // DMA_InitTypeDef  DMA_RX_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  DMA_DeInit(DMA1_Stream6);

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

//  DMA_TX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)teststring;
//  DMA_TX_InitStructure.DMA_BufferSize = (uint16_t)6;

  DMA_Init(DMA1_Stream6, &DMA_TX_InitStructure);

  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
  DMA_Cmd(DMA1_Stream6, ENABLE);
/*
  memcpy(&DMA_RX_InitStructure,&DMA_TX_InitStructure,sizeof(DMA_InitTypeDef));

  DMA_DeInit(DMA1_Stream5);

  DMA_RX_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; // Receive
  DMA_RX_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
  DMA_RX_InitStructure.DMA_BufferSize = DMA_RX_BUF_SIZE;
  DMA_RX_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_Init(DMA1_Stream6, &DMA_RX_InitStructure);

  USART_DMACmd(UART2, USART_DMAReq_Rx, ENABLE);

  // Enable DMA Stream Half Transfer and Transfer Complete interrupt
  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, ENABLE);
*/

}

void DMA1_Stream6_IRQHandler(void){

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6))
    {
        DMA_ClearITPendingBit(DMA1_Stream6,DMA_IT_TCIF6);
        xSemaphoreGiveFromISR(xSemaphore,&xHigherPriorityTaskWoken);
    }
}

/*
void DMA1_Stream5_IRQHandler(void)
{
  // Test on DMA Stream Transfer Complete interrupt
  if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF2))
  {
    // Clear DMA Stream Transfer Complete interrupt pending bit
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF2);
  }

  // Test on DMA Stream Half Transfer interrupt
  if (DMA_GetITStatus(DMA1_Stream5, DMA_IT_HTIF2))
  {
    // Clear DMA Stream Half Transfer interrupt pending bit
    DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_HTIF2);
  }
}
*/

// ============================================================================
void vDebugInitQueue( void ) {
	xDebugQueue = xQueueCreate( DEBUG_QUEUE_SIZE, sizeof(DQ) );
}

// ============================================================================
void vDebugTask(void* pvParameters ) {
	DQ dq;
	portBASE_TYPE xStatus;

	vDebugString("Debug task started");

	for(;;) {
		xStatus = xQueueReceive( xDebugQueue, &dq, (portTickType)portMAX_DELAY);
		DEBUG_usart_send(dq.data,dq.length*sizeof(uint8_t));

		if(xSemaphoreTake(xSemaphore,100) != pdTRUE){
			vTaskDelay(100);
		}
		else{
			vTaskDelay(4); // FIFO has 4 words
		}
	}
}

void createDebugTask(void){

	xTaskCreate( vDebugTask, ( signed char * ) "DebugTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, (tskIDLE_PRIORITY+1)| portPRIVILEGE_BIT, &hDebugTask );
}

// This function copies the the given string into the OS queue.  If the queue
// is full then the rest of the string is ignored.
// ToDo: Ignoring a full queue is not good.
// ============================================================================
void vDebugString( char* s ) {
	portBASE_TYPE xStatus;
	DQ dq;

	dq.length = (uint16_t)strlen(s);
	memcpy((char*)&dq.data,(char*)s,dq.length*sizeof(char)+sizeof(uint16_t));

	dq.data[dq.length++] = '\r';
	dq.data[dq.length++] = '\n';

	// Once we start coping a string into the queue we don't want to get
	// interrupted.  The copy must be done quickly since interrupts are off!
	taskENTER_CRITICAL();
		xStatus = xQueueSendToBack( xDebugQueue, &dq , 0 );
	taskEXIT_CRITICAL();
}

// Simply print to the debug console a string based on the type of reset.
// ============================================================================
void vDebugPrintResetType( void ) {

	if ( PWR_GetFlagStatus( PWR_FLAG_WU ) )
		vDebugPrintf( "PWR: Wake Up flag\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_SB ) )
		vDebugPrintf( "PWR: StandBy flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_PVDO ) )
		vDebugPrintf( "PWR: PVD Output.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_BRR ) )
		vDebugPrintf( "PWR: Backup regulator ready flag.\r\n" );
	if ( PWR_GetFlagStatus( PWR_FLAG_REGRDY ) )
		vDebugPrintf( "PWR: Main regulator ready flag.\r\n" );

	if ( RCC_GetFlagStatus( RCC_FLAG_BORRST ) )
		vDebugPrintf( "RCC: POR/PDR or BOR reset\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PINRST ) )
		vDebugPrintf( "RCC: Pin reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_PORRST ) )
		vDebugPrintf( "RCC: POR/PDR reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_SFTRST ) )
		vDebugPrintf( "RCC: Software reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_IWDGRST ) )
		vDebugPrintf( "RCC: Independent Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_WWDGRST ) )
		vDebugPrintf( "RCC: Window Watchdog reset.\r\n" );
	if ( RCC_GetFlagStatus( RCC_FLAG_LPWRRST ) )
		vDebugPrintf( "RCC: Low Power reset.\r\n" );
}


// DebugPrintf - really trivial implementation, however, it's reentrant!
// ToDo - This needs a rewrite! Add code to check we're not overflowing.
// ============================================================================
void vDebugPrintf(const char *fmt, ...) {
	char sTmp[80];	// String build area.  String lives on the stack!
	uint8_t pos=0;
	char *bp = (char *)fmt;
    va_list ap;
    char c;
    char *p;
    int i;

    va_start(ap, fmt);

    while ((c = *bp++)) {
        if (c != '%') {
            sTmp[pos++] = c;
            continue;
        }

        switch ((c = *bp++)) {
			// d - decimal value
			case 'd':
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 10);
				break;

			// %x - value in hex
			case 'x':
				sTmp[pos++] = '0';
				sTmp[pos++] = 'x';
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 16);
				break;

			// %b - binary
			case 'b':
				sTmp[pos++] = '0';
				sTmp[pos++] = 'b';
				vNum2String( sTmp, &pos, va_arg(ap, uint32_t), 2);
				break;

			// %c - character
			case 'c':
				sTmp[pos++] = va_arg(ap, int);
				break;

			// %i - integer
			case 'i':
				i = va_arg(ap, int32_t);
				if(i < 0){
					sTmp[pos++] = '-';
					vNum2String( sTmp, &pos, (~i)+1, 10);
				} else {
					vNum2String( sTmp, &pos, i, 10);
				}
				break;

			// %s - string
			case 's':
				p = va_arg(ap, char *);
				do {
					sTmp[pos++] = *p++;
				} while (*p);
				break;

			// %% - output % character
			case '%':
				sTmp[pos++] = '%';
				break;

			// Else, must be something else not handled.
			default:
				sTmp[pos++] = '?';
				break;
        }
    }
    sTmp[pos++] = 0;		// Mark the end of the string.
    vDebugString( sTmp );	// Copy the string into the OS queue.
    return;
}


// Convert a number to a string - used in vDebugPrintf.
// ============================================================================
void vNum2String( char *s, uint8_t *pPos, uint32_t u32Number, uint8_t u8Base) {

    char buf[33];
    char *p = buf + 33;
    uint32_t c, n;

    *--p = '\0';
    do {
        n = u32Number / u8Base;
        c = u32Number - (n * u8Base);
        if (c < 10) {
            *--p = '0' + c;
        } else {
            *--p = 'a' + (c - 10);
        }
        u32Number /= u8Base;
    } while (u32Number != 0);

    while (*p){
    	s[ *pPos ] = *p;
    	*pPos += 1;
        p++;
    }
    return;
}

