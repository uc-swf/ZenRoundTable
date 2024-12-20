/*
 * main_user.c
 *
 */

//########################################
//############# Include´s#################
//########################################
#include <file_thr.h>
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
#include <user_init.h>
#include "fatfs.h"
#include "math.h"
#include "leds.h"    // for NeoPixel LED ring
#include "error.h"

//########################################
//############# Variables ################
//########################################

//--------------- Timer ------------------
//extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef ENC1_TMR;
extern TIM_HandleTypeDef ENC2_TMR;

//----------------- UART ------------------
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart3;
char rx_Buffer[255];
uint8_t i_rxBuffer=0;
uint8_t UART_rx=0;
uint16_t err;
volatile enum commands {WAITING,STOP,START,REFERENCE,MANUAL} remoteControl=WAITING;
//volatile char *filename;
volatile int8_t fileidx = -1;
volatile int8_t filecnt = -1;
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

volatile uint8_t recmode = 0;

//########################################
//############ Application ###############
//########################################

void main_user()
{
	HAL_Delay(500);	// Wait 500ms before powering up

	user_init();

	ledring_welcome();
	debugPrint(DEBUG_MAIN,"\r\n\n\n\n\n\n*** MAIN ***\n\n\r");

	HAL_UART_Receive_IT (&huart1, &UART_rx,1); //huart1 -> USB
	HAL_UART_Receive_IT (&huart3, &UART_rx,1); //huart3 -> BT

	userMessage("############################\r\n");
	userMessage("#### FH SWF             ####\r\n");
	userMessage("#### Sand-Table         ####\r\n");
	userMessage("#### ZenRoundTable V1.1 ####\r\n");
	userMessage("############################\r\n");

	userMessage("-based on the BA Thesis of -\r\n" \
			    "-St.Mueller ----------------\r\n\n");

	userMessage("Referencing ...");
	err=scara_reference();
	if (err) error(err);
	userMessage(" done!\r\n");

/*	int ref1,ref2;
	while(1)
	{
		HAL_Delay(200);
		ref1 = (HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==1);
		ref2 = (HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_2_Pin)==1);
		userMessage("References %i %i\r\n",ref1,ref2);
	}
	*/
	// Fast clean
	//scara_wiper();

	remoteControl=WAITING;
	__HAL_TIM_SET_COUNTER(&htim2,11854);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	debugPrint(DEBUG_MAIN,"Referenziert!\r\n" \
			              "Pos M1 : %f\r\n" \
			              "Pos M2 : %f\r\n",scara_get_pos_m1(),scara_get_pos_m2());

	while(1){

		readfiles=file_list();
		userMessage("\n\n***Directory***\r\n");
		for ( int i=0 ; i<readfiles.nFile ; i++ ){
			userMessage("  |- %d. %s \r\n",i,readfiles.fileNames[i]);
			HAL_Delay(10);
		}
		userMessage("\nOptions\r\n");
//		HAL_Delay(100);
		userMessage("1...9   Choose File No 1...9\r\n");
//		HAL_Delay(100);
		userMessage("  R     Start new referencing\r\n");
//		HAL_Delay(100);
		userMessage("  S     Start drawing selected file\r\n");
//		HAL_Delay(100);
		userMessage("  X     Stop drawing\r\n");

		// Redo output after 1 sec w/o user interaction
		while( ( ( fileidx < 0 ) && ( remoteControl != REFERENCE ) && ( remoteControl != MANUAL )) )
		{
			HAL_Delay(1000);
		}

		// Test Mode -> Manual drive to some positions
		if( ( remoteControl == MANUAL ) ){
				enum statePositioning stateManual=IN_POSITION;

				/*
				if (referenceMotors()) error();
				__HAL_TIM_SET_COUNTER(&htim2,11854); // htim2 muss hier gesetzt werden damit der Wert übernommen wird?
				__HAL_TIM_SET_COUNTER(&htim3,0);


				debugPrint(DEBUG_MANUAL,"htim3 Get Counter: [ %"PRIu32" ]\r\n",__HAL_TIM_GET_COUNTER(&htim3));
				debugPrint(DEBUG_MANUAL,"htim2 Get Counter: [ %"PRIu32" ] \r\n",__HAL_TIM_GET_COUNTER(&htim2));
				*/

				while( ( remoteControl != STOP )){
/*					do
					{
						stateManual=positioningScaraRobot(6.283,0.1); // 0° 2Pi
					}
					while( (stateManual == POSITIONING) && (remoteControl != STOP));
					debugPrint(DEBUG_MANUAL,"Position 0° reached! \r\n");

					HAL_Delay(5000);
					do
					{
						stateManual=positioningScaraRobot(1.571,0.1); // 90°
					}
					while( (stateManual == POSITIONING) && (remoteControl != STOP));
					debugPrint(DEBUG_MANUAL,"Position 90° reached! \r\n");

					HAL_Delay(5000);*/
					do
					{
						stateManual=positioningScaraRobot(0.,0.9); // 180°
					}
					while( (stateManual == POSITIONING) && (remoteControl != STOP));
					debugPrint(DEBUG_MANUAL,"Position 180° reached! \r\n");

/*					HAL_Delay(5000);
					do
					{
						stateManual=positioningScaraRobot(4.712,0.1); // 270°
					}
					while( (stateManual == POSITIONING) && (remoteControl != STOP));
					debugPrint(DEBUG_MANUAL,"Position 270° reached! \r\n");

					HAL_Delay(5000);
					*/
				}

				remoteControl=WAITING;
			}

		if(( remoteControl != REFERENCE ) && ( remoteControl != START ))
		{
			if ((fileidx>0) && (fileidx<filecnt))
			{
				userMessage("File %s selected, start the Program by Sending an S...\r\n", readfiles.fileNames[fileidx]);
			}
		}

		while( ( remoteControl != START ) && ( remoteControl != REFERENCE ) ){
			HAL_Delay(1000);
		}

		//********* DRAW the File
		if ( ( remoteControl == START ) )
		{
			debugPrint(DEBUG_THR,"Start reading THR-File....\r\n");

			if ( file_open_thr(readfiles.fileNames[fileidx],&fileObject) == OPEN_FILE )
			{
				debugPrint(DEBUG_THR,"openThrFile done \r\n");
				result = file_thr_header(&readBuffer,&fileObject,&textHeader);
				debugPrint(DEBUG_THR,"readThrHeader: result %d \r\n", result);
				float angle;
				if( result == READ_HEADER){
					do {
						if ( ( stateScaraRobot == IN_POSITION ) || ( stateScaraRobot == IN_REFERENCE )){
							actValues=readThrValuesPerLine(&readBuffer,&fileObject);
							debugPrint(DEBUG_THR,"actValues.state= %d\r\n",actValues.state);
							if(actValues.state == READ_LINE){
								debugPrint(DEBUG_THR,"actValues: theta: %f, rho: %f \r\n",actValues.theta,actValues.rho);
								debugPrint(DEBUG_THR,"readThrValuesPerLine done \r\n");
								angle = 180./M_PI*actValues.theta;
								//userMessage("Angle %5.1f\r\n",angle);
								ledring_highlight(angle, (color_t){255,255,255,0});
								stateScaraRobot=positioningScaraRobot(actValues.theta,actValues.rho);
							} else if ( (actValues.rho == END_OF_FILE)){
								scara_stop_all();
								if (file_close(&fileObject) == CLOSE_FILE){
									debugPrint(DEBUG_THR,"closeThrFile done \r\n");
									debugPrint(DEBUG_ROBOT,"Positionierung beendet!\r\n Position Motor 1 : %f \r\n Position Motor 2 : %f \r\n",scara_get_pos_m1(),scara_get_pos_m2());
									userMessage("THR file closed\r\n");
									fileidx=-1;
								} else {
									debugPrint(DEBUG_THR,"closeThrFile Error \r\n");
									userMessage("ERROR in close file\r\n");
								}
							} else if (actValues.state == ERR){
								scara_stop_all();
								debugPrint(DEBUG_THR,"readThrValuesPerLine Error \r\n");
							}
						}
						if( actValues.state == READ_LINE){
						stateScaraRobot=positioningScaraRobot(actValues.theta,actValues.rho);
						}
					} while(actValues.state == READ_LINE && ( remoteControl != STOP ) && (fileidx>0));
				} else {
					debugPrint(DEBUG_THR,"readThrHeader: Error reading Header \r\n");
				}
				scara_stop_all();
				file_close(&fileObject);
				//filename="";
				fileidx = -1;
				remoteControl = WAITING;
			} else {
				debugPrint(DEBUG_THR,"openThrFile: Error open file \r\n");
			}
		}
			if( ( remoteControl == REFERENCE ) ){
				err=scara_reference();
				if (err) error(err);
				remoteControl=WAITING;
				//__HAL_TIM_SET_COUNTER(&htim2,11854);
				//__HAL_TIM_SET_COUNTER(&htim3,0);
				debugPrint(DEBUG_MAIN,"Referenziert!\r\n Position Motor 1 : %f \r\n Position Motor 2 : %f \r\n",scara_get_pos_m1(),scara_get_pos_m2());

			}
	}
}

void remoteControlUart()
{
	static char fbuf[100];
	static uint8_t fptr=0;

	rx_Buffer[i_rxBuffer]=UART_rx;

	i_rxBuffer++;

	if (i_rxBuffer>0){
		i_rxBuffer=0;
	}

	if (recmode)
	{
		userMessage("RM: %i ",rx_Buffer[0]);
		fbuf[fptr++] = rx_Buffer[0];
		if (rx_Buffer[0] == 10) // EOL
		{
			fbuf[fptr]=0;
			file_write_line(fbuf);
			fptr=0;
		}

		if (rx_Buffer[0] == '@')
		{
			userMessage("RM: done\r\n");
			file_write_close();
			recmode = 0;
			fptr=0;
		}
		return;
	}

		switch (rx_Buffer[0]){

		/* Control Commands */
		case  's':	// no break
		case  'S': 	remoteControl = START;

					debugPrint(DEBUG_COM,"Recieved Command Start\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;
		case  'x':
		case  'X': 	remoteControl = STOP;
					debugPrint(DEBUG_COM,"Recieved Command Stop\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;
		case  'r':
		case  'R': 	remoteControl = REFERENCE;
					debugPrint(DEBUG_COM,"Recieved Command Referencing\r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;

		case  'm':
		case  'M': 	remoteControl = MANUAL;
					debugPrint(DEBUG_COM,"Recieved Command Manual \r\n");
					debugPrint(DEBUG_COM,"remoteControl= %i \r\n",remoteControl);
					break;

		/* File Commands */
		case  'l':
		case  'L': 	readfiles=file_list();
					filecnt = readfiles.nFile;
					for ( int i=0 ; i<readfiles.nFile ; i++ ){
						userMessage("Filename: %d. %s \r\n",i,readfiles.fileNames[i]);
					}
					break;
		case  't':
		case  'T':
					remoteControl = STOP;
					recmode = 1;
					file_write_open("upload.thr");
					break;
		case  '0' ... '9':
					fileidx=rx_Buffer[0]-'0';
					if ( (fileidx>0) && (fileidx<filecnt) )
					{
						userMessage("Choose File %i: %s \r\n",fileidx, readfiles.fileNames[fileidx]);
					}
					else
					{
						userMessage("ERROR: No such File %i\r\n",fileidx);
						fileidx = -1;
					}
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


