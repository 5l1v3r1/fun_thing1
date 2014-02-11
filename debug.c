/*
 * debug.c
 *
 *  Created on: Jan. 5, 2012
 *      Author: James Kemp
 */
#include <stdarg.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"

#include "debug.h"
#include "utils.h"


// Private functions.
void vNum2String( char *s, uint8_t *pPos, uint32_t u32Number, uint8_t u8Base);

// Total buffer size for all debug messages.
#define DEBUG_QUEUE_SIZE	256
xQueueHandle xDebugQueue;

uint8_t TxBuffer[TxBufferLength];

static xTaskHandle hDebugTask;


void InitDebug()
{


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

	/* GPIOA Configuration:  USART2 TX on PD2 and RX on PD3 */
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

	/* USART Clock Initialization  */
	USART_ClockInitstructure.USART_Clock   = USART_Clock_Disable ;
	USART_ClockInitstructure.USART_CPOL    = USART_CPOL_Low ;
	USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
	USART_ClockInitstructure.USART_CPHA    = USART_CPHA_1Edge;

	// USART IRQ init
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	/* USART configuration */
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitstructure);
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART2, ENABLE); // enable USART2
}

// ============================================================================
void vDebugInitQueue( void ) {
	xDebugQueue = xQueueCreate( DEBUG_QUEUE_SIZE, sizeof( char ) );
}

// ============================================================================
void vDebugTask(void* pvParameters ) {
	DQ dq;
	portBASE_TYPE xStatus;

	/* The parameters are not used. */
	( void ) pvParameters;

	debug_uart2_config();

	vDebugString( "Debug task started.\r\n");

	for(;;) {

		xStatus = xQueueReceive( xDebugQueue, &dq, 10 / portTICK_RATE_MS );

		memcpy(TxBuffer,dq.data,dq.length*sizeof(uint8_t));

		// DMA_TX_command

		taskYIELD();
	}
}


// This function copies the the given string into the OS queue.  If the queue
// is full then the rest of the string is ignored.
// ToDo: Ignoring a full queue is not good.
// ============================================================================
void vDebugString( char *s ) {
	portBASE_TYPE xStatus;

	// Once we start coping a string into the queue we don't want to get
	// interrupted.  The copy must be done quickly since interrupts are off!
	taskENTER_CRITICAL();
	while ( *s ) {
		xStatus = xQueueSendToBack( xDebugQueue, s++, 0 );
		if ( xStatus == errQUEUE_FULL ) break;
	}
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



