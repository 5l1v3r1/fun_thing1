/*-
 * Free/Libre Near Field Communication (NFC) library
 *
 * Libnfc historical contributors:
 * Copyright (C) 2009      Roel Verdult
 * Copyright (C) 2009-2013 Romuald Conty
 * Copyright (C) 2010-2012 Romain Tartière
 * Copyright (C) 2010-2013 Philippe Teuwen
 * Copyright (C) 2012-2013 Ludovic Rousseau
 * See AUTHORS file for a more comprehensive list of contributors.
 * Additional contributors of this file:
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 *
 */

/**
 * @file uart.c
 * @brief UART driver
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif // HAVE_CONFIG_H

#include "uart.h"

#ifndef STM32F40_41xxx
#include <sys/ioctl.h>
#include <sys/select.h>
#endif

#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <ctype.h>

#ifndef STM32F40_41xxx
#include <dirent.h>
#endif

#ifdef STM32F40_41xxx
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#endif

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>

#ifndef STM32F40_41xxx
#include <termios.h>
#endif

#include <unistd.h>
#include <stdlib.h>

#include <nfc/nfc.h>
#include "nfc-internal.h"

#include "debug.h"

#define LOG_GROUP    NFC_LOG_GROUP_COM
#define LOG_CATEGORY "libnfc.bus.uart"

#  if defined(__APPLE__)
const char *serial_ports_device_radix[] = { "tty.SLAB_USBtoUART", "tty.usbserial-", NULL };
#  elif defined (__FreeBSD__) || defined (__OpenBSD__)
const char *serial_ports_device_radix[] = { "cuaU", "cuau", NULL };
#  elif defined (__linux__)
const char *serial_ports_device_radix[] = { "ttyUSB", "ttyS", "ttyACM", "ttyAMA", "ttyO", NULL };
#  elif defined (STM32F40_41xxx)
const char *serial_ports_device_radix[] = { "USART", NULL };
#  else
#    error "Can't determine serial string for your system"
#  endif

// Work-around to claim uart interface using the c_iflag (software input processing) from the termios struct
#  define CCLAIMED 0x80000000

#define UART_DATA( X ) ((struct serial_port_unix *) X)

void uart_close_ext(USART_TypeDef* USARTx, const bool restore_termios);

void uart_open(USART_TypeDef* USARTx)
{
	vDebugString("entered uart_open()\n");
}

void uart_flush_input(USART_TypeDef* USARTx)
{
	vDebugString("entered uart_flush_input()\n");

}

void uart_set_speed(USART_TypeDef* USARTx, const uint32_t uiPortSpeed)
{
  // Portability note: on some systems, B9600 != 9600 so we have to do
  // uint32_t <=> speed_t associations by hand.
  
  int32_t stPortSpeed = 115200;

  vDebugString("entered uart_set_speed()\n");

}

uint32_t uart_get_speed(USART_TypeDef* USARTx)
{
  uint32_t uiPortSpeed = 0;

  vDebugString("entered uart_get_speed()\n");

  return uiPortSpeed;
}

void
uart_close_ext(USART_TypeDef* USARTx, const bool restore_termios)
{
	vDebugString("entered uart_close_ext()\n");
}

void
uart_close(USART_TypeDef* USARTx)
{
	vDebugString("entered uart_close()\n");
  uart_close_ext(USARTx, true);
}

extern portBASE_TYPE NFC_ReadByte(USART_TypeDef* USARTx, uint8_t* rcvdByte,portTickType timeout);
/**
 * @brief Receive data from UART and copy data to \a pbtRx
 *
 * @return 0 on success, otherwise driver error code
 */
int uart_receive(USART_TypeDef* USARTx, uint8_t *pbtRx, const size_t szRx, void *abort_p, int timeout)
{
  int res = NFC_SUCCESS;
  
  for(int i = 0;i<szRx;i++){
	  if((res = NFC_ReadByte(USARTx,&pbtRx[i],timeout)) < 0)
	  {
		  return res;
	  }
  }

  return NFC_SUCCESS;
}

/**
 * @brief Send \a pbtTx content to UART
 *
 * @return 0 on success, otherwise a driver error is returned
 */
int uart_send(USART_TypeDef* USARTx, const uint8_t *pbtTx, const size_t szTx, int timeout)
{
  (void) timeout;
  LOG_HEX(LOG_GROUP, "TX", pbtTx, szTx);

#ifdef STM32F40_41xxx

#else
  if ((int) szTx == write(UART_DATA(sp)->fd, pbtTx, szTx))
    return NFC_SUCCESS;
  else
    return NFC_EIO;
#endif
    
  return NFC_SUCCESS;
}

