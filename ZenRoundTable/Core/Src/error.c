/*
 * error.c
 *
 *  Created on: Dec 3, 2024
 *      Author: tobi
 */

#include "main.h"
#include "error.h"
#include "scara_robot.h"
#include "leds.h"    // for NeoPixel LED ring
#include "serial.h"

void error(uint16_t err)
{
	uint16_t i,j=0;
	//                       red         orange      blue      violet
	color_t color_code[]={RGB(255,0,0),RGB(160,96,0),RGB(64,64,192),RGB(128,0,128)};
	color_t colors[]={RGB(0,0,0),RGB(24,0,0),RGB(48,0,0),RGB(128,0,0), RGB(48,0,0), RGB(24,0,0)};

	scara_stop_all();
	userMessage("FATAL ERROR (%i)... Stopping\r\n",err);
	ledring_black();
//	-9 . . -6 . -4 . . -1 . 1 . . 4 . 6 . . 9
//  -13    -9   -7     -4   -2    1   3     6
	ledring_set_rng_color(LEDRING_CNT/2-13,LEDRING_CNT/2-9,color_code[0]);
	ledring_set_rng_color(LEDRING_CNT/2-7,LEDRING_CNT/2-4,color_code[ (err/100) ]);
	ledring_set_rng_color(LEDRING_CNT/2-2,LEDRING_CNT/2+1,color_code[ (err/10)%10 ]);
	ledring_set_rng_color(LEDRING_CNT/2+3,LEDRING_CNT/2+6,color_code[ (err%10) ]);

	while(1)
	{
		for (i=0;i<LEDRING_CNT;i++)
		{
			if (( i < (LEDRING_CNT/2-14) ) || ( i > (LEDRING_CNT/2+7) ) )
			{
				ledring_set_color(i,colors[(i+j)%6]);
			}
		}
		ledring_update();
		j++;
		HAL_Delay(50);
	}
}


