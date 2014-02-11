
#include <stdarg.h>
#include <ctype.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"

#include "utils.h"

GPIO_InitTypeDef  GPIO_InitStructure;

void InitLED(void){

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configure PD12, 13, 14 and PD15 in output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void vLedTask(void *vParameter){

	InitLED();

	for(;;){
	    /* Set PD15 Blue */
	    GPIOD->BSRRL = GPIO_Pin_15;
	    /* Reset PD12 Green, PD13 Orange, PD14 Red */
	    GPIOD->BSRRH = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	    vTaskDelay(500);

		/* Set PD12 Green */
	    GPIOD->BSRRL = GPIO_Pin_12;
	    /* Reset PD13 Orange, PD14 Red, PD15 Blue */
	    GPIOD->BSRRH = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	    vTaskDelay(500);

	    // Set PD13 Orange
	    GPIOD->BSRRL = GPIO_Pin_13;
	    // Reset PD12 Green, PD14 Red, PD15 Blue
	    GPIOD->BSRRH = GPIO_Pin_12 | GPIO_Pin_14 | GPIO_Pin_15;
	    vTaskDelay(500);

	    // Set PD14 Red
	    GPIOD->BSRRL = GPIO_Pin_14;
	    // Reset PD12 Green, PD13 Orange, PD15 Blue
	    GPIOD->BSRRH = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
	    vTaskDelay(500);

	}
}

void createLEDTask(void){
	xTaskCreate( vLedTask, ( signed char * ) "LedTest", configMINIMAL_STACK_SIZE, ( void * ) NULL, (tskIDLE_PRIORITY+1)| portPRIVILEGE_BIT, NULL );
}


