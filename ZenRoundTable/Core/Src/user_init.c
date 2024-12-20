/*
 * Init.c
 *
 */


#include <file_thr.h>
#include <scara_robot.h>
#include <stdio.h>
#include <user_init.h>
#include "main.h"
#include "leds.h"
#include "error.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

void user_init()
{

	// Start PWM-Mode in Timer 4
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);

	//Set Duty Cycle to Zero at beginning
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);

	// Activate Interrupt for Encoder
	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1);

	ledring_init();

	// Mount SD-Card with thr-Files
	if (file_mount()!=0)  // !=0 -> Error
	{
		error(ERROR_SDMOUNT);
	}
	HAL_Delay(100);

	// Stop Motors
	scara_stop_all();

	//Enable Bluetooth
	HAL_GPIO_WritePin(BT_PWR_GPIO_Port,BT_PWR_Pin,GPIO_PIN_SET);
	HAL_Delay(100);

	debugPrint(DEBUG_MAIN,"init: Done...\r\n");
}
