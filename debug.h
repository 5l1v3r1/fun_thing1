/*
 * debug.h
 *
 *  Created on: Feb 9, 2011
 *      Author: James
 */

#ifndef DEBUG_H_
#define DEBUG_H_

void vDebugInitQueue( void );
void vDebugString( char *s );
void vDebugPrintf(const char *fmt, ...);
void vDebugPrintResetType( void );

#define TxBufferLength 32

typedef struct DebugQueue
{
	uint8_t  data[TxBufferLength];
	uint32_t length;
}DQ;

#endif /* DEBUG_H_ */
