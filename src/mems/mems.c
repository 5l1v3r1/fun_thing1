/*
 * mems.c
 *
 *  Created on: 2014. 3. 20.
 *      Author: jongwonk
 */

#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"

xTaskHandle memsTask;
static void vMemsTask(void *vParameter);

void initMems(void)
{

}

void createMemsTask(void){
	xTaskCreate( vMemsTask, ( signed char * ) "MemsTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, (tskIDLE_PRIORITY+1)| portPRIVILEGE_BIT, &memsTask );
}

void vMemsTask(void *vParameter){

}
