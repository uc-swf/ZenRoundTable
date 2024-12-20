/************************************************************************************
 *
 *    **
 *   ****   ***
 *   ****   ***
 *   ****
 *    **           Fachhochschule Suedwestfalen
 *    **           Mechatronik / Mikrocomputer / EmbSys
 *                 (c) Prof. Tobias Ellermeyer
 *    **
 *    **
 *
 *
 * Led_user.c
 *
 *  Created on: Apr 4, 2022
 *      see also: https://controllerstech.com/interface-ws2812-with-stm32/
 *
 *   Values for 72 MHz:  ARR=90-1   LO=30  HI=60
 *
 */

#include "main.h"
#include "leds.h"
#include <stdlib.h>   // rand()

#define PI 3.14159265					t

#define SGN(x) ((x>0)-(x<0))
#define PWM_HI	60
#define PWM_LO  30
#define PWM_ARR 90-1

#define PWM_TIMER htim1

extern TIM_HandleTypeDef PWM_TIMER;			//Timer 1 -> LED-Ring data

static volatile int datasentflag=1;				// is DMA transfer completed (1=yes)
static uint16_t pwm_data[(24*LEDRING_CNT)+50];	// Array with PWM values for DMA transfer
static color_t led_data[LEDRING_CNT];		    // Array with LED data
static uint8_t dma_mode = DMA_NON_BLOCKING;

void ledring_init()
{
	PWM_TIMER.Instance->ARR = PWM_ARR;
	//HAL_TIM_Base_Start(&PWM_TIMER);
	for (int i=0;i<24*LEDRING_CNT+50;i++)
	{
		pwm_data[i]=0;
	}
	ledring_black();
}

/************************************************
 * Switch all LEDs of the ring off
 ************************************************/
void ledring_black()
{
	for (int i=0;i<LEDRING_CNT;i++)
	{
		led_data[i].r=0;
		led_data[i].g=0;
		led_data[i].b=0;
	}
}

/**************************************************
 * Welcome animation
 *
 * 5 seconds ...
 *************************************************/
void ledring_welcome()
{
	uint16_t bright = 128;

	float inc = (float)bright/(float)LEDRING_CNT;
	float val = 0.;

	uint16_t rounds;

	rounds = 5000/(2*(10*LEDRING_CNT));
	if (rounds<1) rounds=1;
	if (rounds>5) rounds=5;
	for (uint16_t r=0;r<rounds;r++)
	{
		for (uint16_t i=0;i<LEDRING_CNT;i++)
		{
			ledring_set_rgb(i,val,bright-val,0);
			val+=inc;
			if (i%3==0)
			{
				ledring_update();
				HAL_Delay(10);
			}
		}
		val = 0.;
		for (uint16_t i=0;i<LEDRING_CNT;i++)
		{
			ledring_set_rgb(i,bright-val,0,val);
			val+=inc;
			if (i%3==0)
			{
				ledring_update();
				HAL_Delay(10);
			}
		}

	}

	for (int i=0;i<LEDRING_CNT;i++)
	{
		ledring_set_rgb(i,0,0,0);
		ledring_update();
		if (i%3==0)
		{
			ledring_update();
			HAL_Delay(10);
		}
	}
}

/****************************************************
 * Set one led to the specified color (rgb values)
 ***************************************************/
void ledring_set_rgb(uint8_t led_num, uint8_t red, uint8_t green, uint8_t blue)
{
	if (led_num<LEDRING_CNT)
	{
		led_data[led_num].r = red;
		led_data[led_num].g = green;
		led_data[led_num].b = blue;
	}
}

/****************************************************
 * Set one led to the specified color (color_t)
 ***************************************************/
void ledring_set_color(uint8_t led_num, color_t color)
{
	if (led_num<LEDRING_CNT)
	{
		led_data[led_num].r = color.r;
		led_data[led_num].g = color.g;
		led_data[led_num].b = color.b;
	}
}

/****************************************************
 * Get led color (color_t)
 ***************************************************/
color_t ledring_get_color(uint8_t led_num)
{
	color_t color={0,0,0};
	if (led_num<LEDRING_CNT)
	{
		color.r = led_data[led_num].r;
		color.g = led_data[led_num].g;
		color.b = led_data[led_num].b;
	}
	return color;
}

/****************************************************
 * Get led color (rgb)
 ***************************************************/
void ledring_get_rgb(uint8_t led_num, uint8_t *red, uint8_t *green, uint8_t *blue)
{
	if (led_num<LEDRING_CNT)
	{
		(*red)=led_data[led_num].r;
		(*green)=led_data[led_num].g;
		(*blue)=led_data[led_num].b;
	}
}

/****************************************************
 * Set color for multiple leds (start...stop) (color_t)
 ***************************************************/
void ledring_set_rng_color(uint8_t start, uint8_t stop, color_t color)
{
	if (start>=LEDRING_CNT) start=LEDRING_CNT;
	if (stop>=LEDRING_CNT) stop=LEDRING_CNT;
	if (stop == start ) return;
	if (start>stop)	// swap if necessary
	{
		int16_t h;
		h=start; start=stop; stop=h;
	}
	for (int i=start;i<=stop;i++)
	{
		led_data[i] = color;
	}
}

void ledring_highlight(float angle, color_t color)
{
	static uint32_t last_tick=0;

	uint16_t led;
	uint16_t i;
	uint8_t r,g,b;

	// Update only every 0.1 second
	if ( (HAL_GetTick()-last_tick) <100) return;

	last_tick= HAL_GetTick();

	if (angle <0.) angle+=360.;
	if (angle >360.) angle -=360.;

	led = ( (uint8_t)(117.*angle/360.+53) )%117;	// 53 -> Ring offset

	ledring_set_color((led-1)%117,color);
	ledring_set_color((led)%117,color);
	ledring_set_color((led+1)%117,color);

	r=128; g=128; b=128;
	for (i=2;i<58;i++)
	{
		if (i<=12)  // to green
		{
			r=128-i*10;
			b=128-i*10;
		} // final: r = b = 8  ; g = 128
		else if (i<=22)  // to yellow
		{
			r = 8+(i-12)*6;
			g = 128-(i-12)*6;
		}  // final: b = 8 ; r = 68 ;  g = 68
		else if (i<=32) // to red
		{
			g = 68-(i-22)*6;
			r = 68+(i-22)*6;
		}  // r = 128; g = 8;  b = 8
		else if (i<=42) // to pink
		{
			r = 128-(i-32)*6;
			b = 8+(i-32)*6;
		}  // r = 68; g = 8; b = 68
		else if (i<=52) // to blue
		{
			r = 68 - (i-42)*6;
		    b = 68 + (i-42)*6;
		}
		else
		{
			r=0; b=0; g = 0;
		}
		ledring_set_rgb((led-i)%117,r/2,g/2,b/2);
		ledring_set_rgb((led+i)%117,r/2,g/2,b/2);
	}
	ledring_update();

}
/**************************************************
 *
 * Sparkle/Twinkle effect during wiper
 *
 *************************************************/
void ledring_wiper_init()
{
	uint8_t i;
	ledring_black();

	for (i=0;i<6;i++)
	{
		ledring_set_rgb((float)rand()/RAND_MAX*LEDRING_CNT,128,128,128);
	}
	ledring_update();
}

void ledring_wiper_loop()
{
	static uint32_t lastcall=0;
	uint8_t i;

	// Slow down if called to often...
	if ( (HAL_GetTick()-lastcall)<100) return;

	lastcall = HAL_GetTick();

	// Dim to blue
	for (i=0;i<LEDRING_CNT;i++)
	{
		led_data[i].r /= 2;
		led_data[i].g /= 2;
		led_data[i].b = led_data[i].b/2 + led_data[i].b/4;  // :1,5
	}

	// New sparkle
	ledring_set_rgb((float)rand()/RAND_MAX*LEDRING_CNT,128,128,128);
	ledring_update();
}

/*****************************************************************************
 * UPDATE pwm_data and trigger sending them to the Neopixel ring
 *****************************************************************************/
void ledring_update(void)
{
	uint32_t *color;
	uint32_t idx = 0;

	// if non-blocking and dma not completed -> dismiss
	if ( (datasentflag==0) && (dma_mode == DMA_NON_BLOCKING) ) { return;}

	// if previous transfer is still ongoing -> wait for completion
	while (!datasentflag){};

	// uint32 pointer to struct (we arranged the struct that way)
	color = (uint32_t*)&led_data;

	for (int i=0;i<LEDRING_CNT;i++)
	{
		for (uint32_t i=0x00800000; i!=0; i>>=1) 		// step through all 24 bits of color value
		{
			pwm_data[idx] = ( (*color)&i?PWM_HI:PWM_LO);		// if bit is set, use pwm=60 otherwise 30
			idx++;	// next pwm array element
		}
		color++;	// next led
	}

	// start DMA transfer
	HAL_TIM_PWM_Start_DMA(&PWM_TIMER, TIM_CHANNEL_1, (uint32_t *)pwm_data, 24*LEDRING_CNT+50);
	datasentflag = 0;  // mark transfer ongoing
}


//Funktion die nach senden der Bytes aufgerufen wird
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1); 	//Senden/PWM wird angehalten
	datasentflag=1;									//auf 1 gesetzt -> Nur einmalige sendung der Daten

}

