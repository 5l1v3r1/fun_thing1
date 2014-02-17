/*
 * nfc_task.h
 *
 *  Created on: 2014. 2. 16.
 *      Author: jongwonk
 */

#ifndef NFC_TASK_H_
#define NFC_TASK_H_


typedef struct TxQueueData
{
	uint16_t length;
	uint8_t  data[128];
}TQ;

void NFC_Send(uint8_t* tx_data,uint16_t len);

#endif /* NFC_TASK_H_ */
