/*
 * debug.h
 *
 *  Created on: Feb 9, 2011
 *      Author: James
 */

#ifndef DEBUG_H_
#define DEBUG_H_

void vDebugInitQueue( void );
void vDebugString( uint8_t* s );
void vDebugPrintf(const char *fmt, ...);
void vDebugPrintResetType( void );
void createDebugTask(void);

#define TxBufferLength 32

typedef struct DebugQueue
{
	uint16_t length;
	uint8_t  data[TxBufferLength];
}DQ;

#endif /* DEBUG_H_ */
