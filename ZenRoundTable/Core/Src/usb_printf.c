/*
 * usb_printf.c
 *
 *  Created on: 09.09.2020
 *      Author: Tobias Ellermeyer
 */
#include "main.h"

//huart2 -> USB, huart3-> Bluetooth
extern UART_HandleTypeDef huart1,huart3;


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

  /* Clear the Overrun flag just before receiving the first character */
  //__HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart1);

  /* Wait for reception of a character on the USART RX line and echo this
   * character on console */
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



//TEST SCANF()




