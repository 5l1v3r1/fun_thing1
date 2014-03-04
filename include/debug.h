/*
 * debug.h
 *
 *  Created on: Feb 9, 2011
 *      Author: James
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "stdint.h"

void vDebugInitQueue( void );
void vDebugString(char* s );
void vDebugPrintf(const char *fmt, ...);
void vDebugPrintResetType( void );
void createDebugTask(void);

void InitDebug(void);

#define DEBUG_TxBufferLength 64
#define DEBUG_RxBufferLength 8

typedef struct DebugQueue
{
	uint16_t length;
	uint8_t  data[DEBUG_TxBufferLength];
}DQ;

#endif /* DEBUG_H_ */
