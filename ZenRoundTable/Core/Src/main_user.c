/*
 * main_user.c
 *
 */

//########################################
//############# Include´s#################
//########################################
#include "main.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include <pid_controller.h>
#include <scara_robot.h>
#include <sd_thr.h>
#include <user_init.h>
#include "fatfs.h"
#include "math.h"

//########################################
//############# Variables ################
//########################################

//--------------- Timer ------------------
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;

//----------------- UART ------------------
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;
char rx_Buffer[255];
uint8_t i_rxBuffer=0;
uint8_t UART_rx=0;
volatile enum commands {WAITING,STOP,START,REFERENCE,MANUAL} remoteControl=WAITING;
volatile char *filename="";
files readfiles;

//--------------- THR-Files ----------------
thrValues_t actValues={
		.rho=0,
		.state=INIT,
		.theta=0
};
FIL fileObject;
BYTE readBuffer[100]="";
BYTE textHeader[100]="";
enum stateReadThrFile result=INIT;

//------------- SCARA-Robot ----------------
enum statePositioning stateScaraRobot=IN_POSITION;


//########################################
//############ Application ###############
//########################################

void main_user()
{
	HAL_Delay(4000);
	debugPrint(DEBUG_MAIN,"main_user()...\r\n");
	init();

	HAL_UART_Receive_IT (&huart1, &UART_rx,1); //huart1 -> USB
	HAL_UART_Receive_IT (&huart3, &UART_rx,1); //huart3 -> BT
	HAL_Delay(6000);
	userMessage("#################### \r\n");
	userMessage("##### FH SWF ######## \r\n");
	userMessage("#### Sand-Table ####### \r\n");
	userMessage("#### SCARA-Robot ##### \r\n");
	userMessage("#################### \r\n");

	userMessage("---- St.Mueller ---- \r\n");
	userMessage("-------------------- \r\n");

	referenceMotors();
	remoteControl=WAITING;
	__HAL_TIM_SET_COUNTER(&htim2,11854);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	debugPrint(DEBUG_MAIN,"Referenziert!\r\n Position Motor 1 : %f \r\n Position Motor 2 : %f \r\n",getPositionM1(),getPositionM2());

	while(1){

		readfiles=list_files();
		for ( int i=0 ; i<readfiles.nFile ; i++ ){
			userMessage("Filename: %d. %s \r\n",i,readfiles.fileNames[i]);
			HAL_Delay(10);
		}
		userMessage("Choose File 1 - 9 by sending 1 - 9\r\n");
		HAL_Delay(100);
		userMessage("Send L to list all files\r\n");
		HAL_Delay(100);
		userMessage("Send R for Referencing\r\n");
		HAL_Delay(100);
		userMessage("Send S for Start\r\n");
		HAL_Delay(100);
		userMessage("Send X for Stop\r\n");

		while( ( ( strcmp((const char*)filename, "") == 0 ) && ( remoteControl != REFERENCE ) && ( remoteControl != MANUAL )) ){
			HAL_Delay(1000);
		}

		if( ( remoteControl == MANUAL ) ){
				enum statePositioning stateManual=IN_POSITION;

				referenceMotors();
				__HAL_TIM_SET_COUNTER(&htim2,11854); // htim2 muss hier gesetzt werden damit der Wert übernommen wird?
				__HAL_TIM_SET_COUNTER(&htim3,0);


				debugPrint(DEBUG_MANUAL,"htim3 Get Counter: [ %"PRIu32" ]\r\n",__HAL_TIM_GET_COUNTER(&htim3));
				debugPrint(DEBUG_MANUAL,"htim2 Get Counter: [ %"PRIu32" ] \r\n",__HAL_TIM_GET_COUNTER(&htim2));


				while( ( remoteControl != STOP )){
					do {
						stateManual=positioningScaraRobot(6.283,0.9); // 0° 2Pi
					} while(stateManual == POSITIONING);
					debugPrint(DEBUG_MANUAL,"Position 0° reached! \r\n");
					HAL_Delay(5000);
					do {
						stateManual=positioningScaraRobot(1.571,0.9); // 90°
					} while(stateManual == POSITIONING);
					debugPrint(DEBUG_MANUAL,"Position 90° reached! \r\n");
					HAL_Delay(5000);
					do {
						stateManual=positioningScaraRobot(3.142,0.9); // 180°
					} while(stateManual == POSITIONING);
					debugPrint(DEBUG_MANUAL,"Position 180° reached! \r\n");
					HAL_Delay(5000);
					do {
						stateManual=positioningScaraRobot(4.712,0.9); // 270°
					} while(stateManual == POSITIONING);
					debugPrint(DEBUG_MANUAL,"Position 270° reached! \r\n");
					HAL_Delay(5000);
				}
				remoteControl=WAITING;
			}

		if(( remoteControl != REFERENCE ) && ( remoteControl != START )){
			userMessage("File %s selected, start the Program by Sending an S...\r\n", filename);
		}

		while( ( remoteControl != START ) && ( remoteControl != REFERENCE ) ){
			HAL_Delay(1000);
		}

		if ( ( remoteControl == START ) ){
			debugPrint(DEBUG_THR,"Start reading THR-File....\r\n");

		  if ( openThrFile(filename,&fileObject) == OPEN_FILE ){
			  debugPrint(DEBUG_THR,"openThrFile done \r\n");
				result = readThrHeader(&readBuffer,&fileObject,&textHeader);
				debugPrint(DEBUG_THR,"readThrHeader: result %d \r\n", result);
				if( result == READ_HEADER){
					do {
						if ( ( stateScaraRobot == IN_POSITION ) || ( stateScaraRobot == IN_REFERENCE )){
							actValues=readThrValuesPerLine(&readBuffer,&fileObject);
							debugPrint(DEBUG_THR,"actValues.state= %d\r\n",actValues.state);
							if(actValues.state == READ_LINE){
								debugPrint(DEBUG_THR,"actValues: theta: %f, rho: %f \r\n",actValues.theta,actValues.rho);
								debugPrint(DEBUG_THR,"readThrValuesPerLine done \r\n");
								stateScaraRobot=positioningScaraRobot(actValues.theta,actValues.rho);
							} else if ( (actValues.rho == END_OF_FILE)){
								stopScaraRobot();
								if (closeThrFile(&fileObject) == CLOSE_FILE){
									debugPrint(DEBUG_THR,"closeThrFile done \r\n");
									debugPrint(DEBUG_ROBOT,"Positionierung beendet!\r\n Position Motor 1 : %f \r\n Position Motor 2 : %f \r\n",getPositionM1(),getPositionM2());
									filename = "";
								} else {
									debugPrint(DEBUG_THR,"closeThrFile Error \r\n");
								}
							} else if (actValues.state == ERR){
								stopScaraRobot();
								debugPrint(DEBUG_THR,"readThrValuesPerLine Error \r\n");
							}
						}
						if( actValues.state == READ_LINE){
						stateScaraRobot=positioningScaraRobot(actValues.theta,actValues.rho);
						}
					} while(actValues.state == READ_LINE && ( remoteControl != STOP ));
				} else {
					debugPrint(DEBUG_THR,"readThrHeader: Error reading Header \r\n");
				}
				stopScaraRobot();
				closeThrFile(&fileObject);
				filename="";
				remoteControl = WAITING;
			} else {
				debugPrint(DEBUG_THR,"openThrFile: Error open file \r\n");
			}
		}
			if( ( remoteControl == REFERENCE ) ){

				referenceMotors();
				remoteControl=WAITING;
				__HAL_TIM_SET_COUNTER(&htim2,11854);
				__HAL_TIM_SET_COUNTER(&htim3,0);
				debugPrint(DEBUG_MAIN,"Referenziert!\r\n Position Motor 1 : %f \r\n Position Motor 2 : %f \r\n",getPositionM1(),getPositionM2());

			}
	}
}

void remoteControlUart(){

		rx_Buffer[i_rxBuffer]=UART_rx;

		i_rxBuffer++;

		if (i_rxBuffer>0){
			i_rxBuffer=0;
		}

		switch (rx_Buffer[0]){

		/* Control Commands */
		case  'S': 	remoteControl = START;
					debugPrint(DEBUG_COM,"Recieved Command Start\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;
		case  'X': 	remoteControl = STOP;
					debugPrint(DEBUG_COM,"Recieved Command Stop\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;
		case  'R': 	remoteControl = REFERENCE;
					debugPrint(DEBUG_COM,"Recieved Command Referencing\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;

		case  'M': 	remoteControl = MANUAL;
					debugPrint(DEBUG_COM,"Recieved Command Manual \r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;

		/* File Commands */
		case  'L': 	readfiles=list_files();
					for ( int i=0 ; i<readfiles.nFile ; i++ ){
						userMessage("Filename: %d. %s \r\n",i,readfiles.fileNames[i]);
					}
					break;
		case  '0': 	filename=readfiles.fileNames[0];
					userMessage("Choose File 0: %s \r\n",filename);
					break;
		case  '1': 	filename=readfiles.fileNames[1];
					userMessage("Choose File 1: %s \r\n",filename);
					break;
		case  '2': 	filename=readfiles.fileNames[2];
					userMessage("Choose File 2: %s \r\n",filename);
					break;
		case  '3': 	filename=readfiles.fileNames[3];
					userMessage("Choose File 3: %s \r\n",filename);
					break;
		case  '4': 	filename=readfiles.fileNames[4];
					userMessage("Choose File 4: %s \r\n",filename);
					break;
		case  '5': 	filename=readfiles.fileNames[5];
					userMessage("Choose File 5: %s \r\n",filename);
					break;
		case  '6': 	filename=readfiles.fileNames[6];
					userMessage("Choose File 6: %s \r\n",filename);
					break;
		case  '7': 	filename=readfiles.fileNames[7];
					userMessage("Choose File 7: %s \r\n",filename);
					break;
		case  '8': 	filename=readfiles.fileNames[8];
					userMessage("Choose File 8: %s \r\n",filename);
					break;
		case  '9': 	filename=readfiles.fileNames[9];
					userMessage("Choose File 9: %s \r\n",filename);
					break;
		default: 	break;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart==&huart1){ //huart1 -> USB huart3-> BT
		remoteControlUart();
		HAL_UART_Receive_IT (&huart1, &UART_rx,1);
	}
	if(huart==&huart3){ //huart1 -> USB huart3-> BT
		remoteControlUart();
		HAL_UART_Receive_IT (&huart3, &UART_rx,1);
	}


} // HAL_UART_RxCpltCallback


