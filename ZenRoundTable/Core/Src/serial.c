/*
 * usb_printf.c
 *
 *  Created on: 09.09.2020
 *      Author: Tobias Ellermeyer
 */
#include "main.h"
#include "serial.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions


extern UART_HandleTypeDef huart1,huart3;

// Configuration of output interface
const enum outputInterface {NONE,ALL,BLUETOOTH,USBC} userMessageConfig = ALL ;

// print User- or Debug-Messages optional over Usb or Bluetooth
void userMessage(const char *userMessage, ...) {

  static char msgBuffer[256];
  va_list msgArgs;
  va_start(msgArgs, userMessage);
  vsnprintf(msgBuffer, sizeof(msgBuffer), userMessage, msgArgs);
  va_end(msgArgs);

  int len = strlen(msgBuffer);

  if ( ( userMessageConfig == USBC ) || ( userMessageConfig == ALL ) ){
	  HAL_UART_Transmit(&huart1, (uint8_t*)msgBuffer, len, HAL_MAX_DELAY);
  }
  if ( ( userMessageConfig == BLUETOOTH ) || ( userMessageConfig == ALL ) ) {
	  HAL_UART_Transmit(&huart3, (uint8_t*)msgBuffer, len, HAL_MAX_DELAY);
  }
}

/*


int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart1, &*c, 1, 10); //  huart1 > USB / huart 3 -> BT
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}


//TEST SCANF();

int __io_getchar(void)
{
  uint8_t ch = 0;

  // Clear the Overrun flag just before receiving the first character
  //__HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart1);

  // Wait for reception of a character on the USART RX line and echo this
  // character on console
  //HAL_UART_Receive(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _read(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = __io_getchar();
	}

return len;
}
*/






