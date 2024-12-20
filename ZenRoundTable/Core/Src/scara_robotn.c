

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include <math.h>
#include "pid_controller.h"
#include "scara_robot.h"
#include "leds.h"
#include "error.h"

#define ABTASTRATE 4

extern TIM_HandleTypeDef htim2;		// Encoder
extern TIM_HandleTypeDef htim3;		// Encoder
extern TIM_HandleTypeDef htim4;		// PWM


const float radToDegree=(180/M_PI);
const float countsPerDegree=(360.0/23707.0); // counts per degree OLD:23760


pid_params_t pidParameterM1 = {
		.kp=3,
		.ki=0.1,
		.kd=0.00011,
		.Ta=ABTASTRATE,
		.threshold=0.1,
		.y_max=30,
		.y_min=0,
		.esum =0,
		.eold=0
};

pid_params_t pidParameterM2 = {
		.kp=3,
		.ki=0.1,
		.kd=0.0001,
		.Ta=ABTASTRATE,
		.threshold=0.1,
		.y_max=30,
		.y_min=0,
		.esum =0,
		.eold=0
};

uint32_t lt_PID=0;

/***************************************************************************
 *
 * motorPwmCpmtrol
 *
 */

void scara_motor_pwm(int8_t motor_nr, int16_t speed, uint8_t direction){

	if (motor_nr==1){
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,abs(speed));
		if ( direction == MOTOR_CW ){
			// Direction Clockwise
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_RESET);
		} else if ( direction == MOTOR_CCW ){
			//Direction Counter clockwise
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_SET);
		} else {
			// No movement at all... -> STOP
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_RESET);
		}
	}

	if (motor_nr==2){
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,abs(speed));
		if ( direction == MOTOR_CW){
			// Direction Clockwise
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_SET);
		} else if ( direction == MOTOR_CCW ){
			//Direction Counter clockwise
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_RESET);
		} else {
			// -> STOP
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_RESET);
		}
	}

}

/*********************************************************
 *
 * referenceMotors
 *
 ********************************************************/
uint16_t scara_reference() {

	int step;
	int pos;
	uint32_t ticks_timeout;
	uint32_t ticks_encchk;

	const color_t refcolor=RGB(0,0,64);
	const color_t idxcolor=RGB(64,0,64);
	ledring_black();

	for (step = 1; step <= 7; step++)
	{
		ledring_set_rng_color((LEDRING_CNT/7.0+0.5)*(step-1)+1,(LEDRING_CNT/7.0+0.5)*step-1,refcolor);
		ledring_set_color((LEDRING_CNT/7.0+0.5)*step,idxcolor);
		ledring_update();

		switch (step)
		{
			case 1: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 1 Motor 1 fast forward\r\n");

					// Systicks for timeout
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					// Systick when to check if encoder has changed
					ticks_encchk  = HAL_GetTick()+100;
					pos = ENC1_CNT;

					scara_motor_pwm(MOTOR1, SPEED_REF_FAST, MOTOR_CW);
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_SET))
					{
						if (ticks_encchk && (HAL_GetTick()>ticks_encchk) ) // Check if encoder has changed
						{
							if ( abs(pos - ENC1_CNT) <10)
							{
								scara_stop_all();
								return ERROR_ENC1;
							}
							ticks_encchk = 0;    // no further checks
						}
						if (HAL_GetTick()>ticks_timeout)   // Check for timeout
						{
							scara_stop_all();
							return ERROR_REF1_SEARCH;	// TIMEOUT
						}
					}
					scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
					break;

			case 2: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 2 Motor 1 slowly backward\r\n");
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					scara_motor_pwm(MOTOR1, SPEED_REF_SLOW, MOTOR_CCW);
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_RESET)) {
						if (HAL_GetTick()>ticks_timeout)
						{
							scara_stop_all();
							return ERROR_REF1_FINE1;	// TIMEOUT
						}
					}
					HAL_Delay(500);  // Overdrive
					scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
					break;

			case 3: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 3 Motor 1 slowly forward\r\n");
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					scara_motor_pwm(1, SPEED_REF_SLOW, MOTOR_CW);
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_SET)) {
						if (HAL_GetTick()>ticks_timeout)
						{
							scara_stop_all();
							return ERROR_REF1_FINE2;	// TIMEOUT
						}
					}
					scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
					__HAL_TIM_SET_COUNTER(&ENC1_TMR,ENC_VAL_MID);
					debugPrint(DEBUG_ROBOT,"referenceMotors: Encoder counter 1 initialized...\r\n");
					break;

			case 4: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 4 Motor 2 fast forward\r\n");
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					ticks_encchk  = HAL_GetTick()+100;
					pos = ENC2_CNT;
					scara_motor_pwm(MOTOR2, SPEED_REF_FAST, MOTOR_CW);

					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_SET)) {
						if (ticks_encchk && (HAL_GetTick()>ticks_encchk) ) // Check if encoder has changed
						{
							if ( abs(pos - ENC2_CNT) <10)
							{
								scara_stop_all();
								return ERROR_ENC2;
							}
							ticks_encchk = 0;    // no further checks
						}
						if (HAL_GetTick()>ticks_timeout)   // Check for timeout
						{
							scara_stop_all();
							return ERROR_REF2_SEARCH;	// TIMEOUT
						}
					}
					scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
					break;

			case 5: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 5 Motor 2 slowly backward\r\n");
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					scara_motor_pwm(MOTOR2, SPEED_REF_SLOW, MOTOR_CCW);
					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_RESET)) {
						if (HAL_GetTick()>ticks_timeout)
						{
							scara_stop_all();
							return ERROR_REF2_FINE1;	// TIMEOUT
						}
					}
					HAL_Delay(500);
					scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
					break;

			case 6: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 6 Motor 2 slowly forward\r\n");
					ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
					scara_motor_pwm(MOTOR2, SPEED_REF_SLOW, MOTOR_CW);
					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_SET)) {
						if (HAL_GetTick()>ticks_timeout)
						{
							scara_stop_all();
							return ERROR_REF1_FINE2;	// TIMEOUT
						}
					}
					scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
					__HAL_TIM_SET_COUNTER(&ENC2_TMR,0);
					debugPrint(DEBUG_ROBOT,"referenceMotors: Encoder counter 2 initialized...\r\n");
					debugPrint(DEBUG_ROBOT,"referenceMotors: Done...\r\n");
					break;

			default:break;
		}
	}
	return NO_ERROR;
}

/***************************************************************
 *
 * motorTravelDirection
 *
 */
int motorTravelDirection(float actualPosition, float targetPosition ){

	float positionDifference = 0.0, positionDifferenceComp=0.0;

	if(actualPosition < targetPosition){
		debugPrint(DEBUG_ROBOT,"actualPosition < targetPosition\r\n");
		positionDifference = targetPosition - actualPosition;
		positionDifferenceComp = 360 - positionDifference;
		debugPrint(DEBUG_ROBOT,"positionDifference: %f , positionDifferenceComp%f \r\n",positionDifference,positionDifferenceComp);
		if  ( positionDifference < positionDifferenceComp ) {
			return MOTOR_CW;
		} else {
			return MOTOR_CCW;
		}
	} else {
		debugPrint(DEBUG_ROBOT,"actualPosition > targetPosition\r\n");
		positionDifference = actualPosition - targetPosition;
		positionDifferenceComp = 360 - positionDifference;
		debugPrint(DEBUG_ROBOT,"positionDifference: %f , positionDifferenceComp%f\r\n",positionDifference,positionDifferenceComp);
		if  ( positionDifference < positionDifferenceComp ) {
			return MOTOR_CCW;
		} else {
			return MOTOR_CW;
		}
	}
}

/**************************************************
 *
 * statePositioning
 *
 */
enum statePositioning positioningScaraRobot(float theta, float rho){

	static enum statePositioning statePos=IN_POSITION;;
	static uint8_t startPositioning=0;
	static uint8_t directionM1=MOTOR_CCW,directionM2=MOTOR_CCW;
	static float setpointPwmM1,setpointPwmM2;

	float positionDegreeM1 =scara_get_pos_m1() ;
	float positionDegreeM2 =scara_get_pos_m2();
	const float positionWindowM1=0.1,positionWindowM2=0.1;
	static uint8_t positionReachedM1=0, positionReachedM2=0;
	scaraAngles_t scaraTargetAnglesDeg;

	float thetaUnitCircle = fmodf( theta, (2*M_PI) );
	if (thetaUnitCircle < 0 ) {thetaUnitCircle = (2*M_PI) + thetaUnitCircle;}
	debugPrint(DEBUG_ROBOT,"positioningScaraRobot: thetaUnitCircle= %f \r\n",thetaUnitCircle);

	scaraTargetAnglesDeg = calculateAngleScara( thetaUnitCircle , rho);
	debugPrint(DEBUG_ROBOT,"positioningScaraRobot: postionDegreeM1  %f, positionDegreeM2 %f \r\n",positionDegreeM1, positionDegreeM2);
	debugPrint(DEBUG_ROBOT,"positioningScaraRobot: targetPositionM1 %f, targetPositionM2 %f \r\n", scaraTargetAnglesDeg.phi1,scaraTargetAnglesDeg.phi2);

	if (!startPositioning){ // Richtung fuer einen Positioniervorgang festsetzen
		startPositioning = 1;
		debugPrint(DEBUG_ROBOT,"positioningScaraRobot: Positioning started  \r\n");
	}

	if(timeTick(ABTASTRATE,&lt_PID)){ // Abtastrate des PID-Reglers 4ms
		debugPrint(DEBUG_ROBOT,"positioningScaraRobot: timerTick starts \r\n");

		// Wenn Ziel erreicht neues Ziel übergeben
		if ( checkPositionWindow(positionDegreeM1,scaraTargetAnglesDeg.phi1,positionWindowM1) == 1 ){
			scara_motor_pwm(MOTOR1,0,MOTOR_STOP);
			positionReachedM1 = 1;
			setpointPwmM1=0;
			pidParameterM1.esum=0;
			pidParameterM1.eold=0;
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot: Positioning M1 done! \r\n");
		}

		if ( checkPositionWindow(positionDegreeM2,scaraTargetAnglesDeg.phi2,positionWindowM2) == 1 ){
			scara_motor_pwm(MOTOR2,0,MOTOR_STOP);
			positionReachedM2 = 1;
			setpointPwmM2=0;
			pidParameterM2.esum=0;
			pidParameterM2.eold=0;
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot: Positioning M2 done! \r\n");
		}

		if(!positionReachedM1){
			setpointPwmM1=digPIDControl(positionDegreeM1,scaraTargetAnglesDeg.phi1,&pidParameterM1);
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot: pidParameterM1.esum = %f\r\n;",pidParameterM1.esum);
		}

		if (!positionReachedM2){
			setpointPwmM2=digPIDControl(positionDegreeM2,scaraTargetAnglesDeg.phi2,&pidParameterM2);
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot: pidParameterM2.esum = %f\r\n;",pidParameterM2.esum);
		}

		debugPrint(DEBUG_ROBOT,"positioningScaraRobot: setpointPwmM1 %f, setpointPwmM2 %f \r\n", setpointPwmM1, setpointPwmM2);

		if (positionReachedM1 && positionReachedM2){
			startPositioning = 0;
			positionReachedM1 = 0;
			positionReachedM2 = 0;
		}

		if (startPositioning ==1 ){

			directionM1 = motorTravelDirection(positionDegreeM1, scaraTargetAnglesDeg.phi1 );
			directionM2 = motorTravelDirection(positionDegreeM2, scaraTargetAnglesDeg.phi2 );
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot:directionM1 %d, directionM2 %d  \r\n", directionM1, directionM2);

			if( !positionReachedM1 ) {
				scara_motor_pwm(MOTOR1,setpointPwmM1,directionM1);
			}else {
				scara_motor_pwm(MOTOR1,0,MOTOR_STOP);
			}
			if( !positionReachedM2 ) {
				scara_motor_pwm(MOTOR2,setpointPwmM2,directionM2);
			}else {
				scara_motor_pwm(MOTOR2,0,MOTOR_STOP);
			}
			statePos = POSITIONING;
			return statePos;

		} else {

			scara_stop_all();
			debugPrint(DEBUG_ROBOT,"positioningScaraRobot: ScaraRobot stopped \r\n");
			statePos = IN_POSITION;
			return statePos;
		}
	}
	debugPrint(DEBUG_ROBOT,"positioningScaraRobot: Executed without timetick \r\n");
	return statePos;
}

void scara_stop_all(){
	scara_motor_pwm(MOTOR1,0,MOTOR_STOP);
	scara_motor_pwm(MOTOR2,0,MOTOR_STOP);
	debugPrint(DEBUG_ROBOT,"stopScaraRobot: Stopping ScaraRobot M1&M2 \r\n");
}

uint8_t checkPositionWindow(float actualPosition, float targetPosition, float positionWindow){

	uint8_t outsideWindow=0,insideWindow=1;
	float positionWindowUpperLimit = targetPosition + positionWindow;
	float positionWindowLowerLimit = targetPosition - positionWindow;

	if ( ( actualPosition >= positionWindowLowerLimit ) && ( actualPosition <= positionWindowUpperLimit ) ){
		return insideWindow;
		debugPrint(DEBUG_ROBOT,"positionWindow: Inside Positionwindow \r\n");
	} else {
		debugPrint(DEBUG_ROBOT,"positionWindow: Outside Positionwindow \r\nactualPosition: %f, upperLimit: %f, lowerLimit: %f  \r\n",actualPosition,positionWindowUpperLimit,positionWindowLowerLimit);
		return outsideWindow;
	}
}

float scara_get_pos_m1(){

	float positionDegree =(float)__HAL_TIM_GET_COUNTER(&htim3)*countsPerDegree ;
	return positionDegree;

}

float scara_get_pos_m2(){

	float positionDegree =(float)__HAL_TIM_GET_COUNTER(&htim2)*countsPerDegree ;
	return positionDegree;

}

scaraAngles_t calculateAngleScara(float theta, float rho){

	scaraAngles_t scaraValues;
    float L1 = 127.5;
    float L2 = 127.5;
    float L_gesamt = L1 + L2;
    float dist = rho * L_gesamt;
    debugPrint(DEBUG_ROBOT,"calculatingAngleScara: dist= %f \r\n", dist);

    float alpha = acosf( ( ( dist*dist + L1*L1 - L2*L2 ) / ( 2*dist*L1) ) );
    scaraValues.phi1 = ( ( theta - alpha ) ) * radToDegree;

	if (scaraValues.phi1 < 0){scaraValues.phi1 = 360 + scaraValues.phi1;}
	if (scaraValues.phi1 > 360){scaraValues.phi1 = scaraValues.phi1 - 360;}

    debugPrint(DEBUG_ROBOT,"calculatingAngleScara: angleM1= %f \r\n", scaraValues.phi1);

	float beta = acosf( ( ( L2*L2 + L1*L1 - dist*dist ) / ( 2*L2*L1 ) ) );
	scaraValues.phi2 = ( (M_PI - beta) + ( theta - alpha ) )   * radToDegree;

	if (scaraValues.phi2 < 0){scaraValues.phi2 = 360 + scaraValues.phi2;}
	if (scaraValues.phi2 > 360){scaraValues.phi2 = scaraValues.phi2 - 360;}

	debugPrint(DEBUG_ROBOT,"calculatingAngleScara: angleM2= %f \r\n", scaraValues.phi2);

    if ( rho == 0 ){
    	scaraValues.phi1 = 0.0;
    	scaraValues.phi2 = 180.0;
    }
    return scaraValues;
}


/***************************************************
 *
 * Wiper routine (Half circles)
 *
 * Position assumed to be in reference position ...
 *
 *************************************************/

static uint16_t constrain_pos(uint16_t pos, uint16_t start)
{
	int32_t tmp;
	tmp = pos-start;
	if (tmp < 0) tmp += ENC_VAL_MAX;	// Handle MAX->0 overflow
	return ((uint16_t)tmp);
}


void scara_wiper()
{
	enum statePositioning state;
	uint8_t stop=0;
	uint16_t loopcnt=0;

	uint16_t m1start,m2start,m1pos,m2pos;
	uint32_t ticks_timeout;
	uint32_t ticks_last;

	ledring_wiper_init();	// Init effect for wiper
	// Move to center
	do
	{
		state=positioningScaraRobot(0.,0.); // 180°
	}
	while( (state == POSITIONING) );

	// Set Encoders to center
	 __HAL_TIM_SET_COUNTER(&ENC1_TMR,ENC_VAL_MID);
	 __HAL_TIM_SET_COUNTER(&ENC2_TMR,0);

	while(loopcnt < WIPE_LINES)
	{
		loopcnt++;

		//**** Half circle from center to outer
		m1start = ENC1_CNT;
		m2start = ENC2_CNT;
	//	m1go    = ENC_VAL_MID;
	//	stop    = ENC_VAL_MID+(ENC_VAL_MAX / WIPE_LINES);
		userMessage("A (start) Enc1: %6i  Enc2: %6i\r\n", ENC1_CNT, ENC2_CNT);

		ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;
		ticks_last = 0;
		scara_motor_pwm(MOTOR2, SPEED_WIPE, MOTOR_CW);
		//HAL_Delay(500);  // Move a while before waiting
		for (uint8_t i=0;i<20;i++)
		{
			ledring_wiper_loop();
			HAL_Delay(20);
		}
		stop=0;
		do {
			m1pos = constrain_pos(ENC1_CNT,m1start);
			m2pos = constrain_pos(ENC2_CNT,m2start);
			if (m2pos > ENC_VAL_MID)
			{
				scara_motor_pwm(MOTOR1, SPEED_WIPE, MOTOR_CW);
			}
			if (m2pos > (ENC_VAL_MID + ENC_VAL_MAX / WIPE_LINES))
			{
				scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
				stop |= 0x01;
			}
			if (m1pos > (ENC_VAL_MAX / WIPE_LINES))
			{
				scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
				stop |= 0x02;
			}
			// TIMEOUT
			if (HAL_GetTick()>ticks_timeout)
			{
				stop |= 0x04;
				scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
				scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
			}
			if (HAL_GetTick()>ticks_last)
			{
				userMessage("A m1pos: %6i  m2pos: %6i  stop: %2i\r\n", m1pos, m2pos, stop);
				ticks_last = HAL_GetTick()+200;
			}
			ledring_wiper_loop();  // Do wiper effect
		}
		while(stop<3);

		if (stop & 0x04)   // There was a timeout
		{
			error(ERROR_WIPER1);
		}

		//**** Half circle from outer to center
		m1start = ENC1_CNT;
		m2start = ENC2_CNT;
	//	m1go    = ENC_VAL_MID;
	//	stop    = ENC_VAL_MID+(ENC_VAL_MAX / WIPE_LINES);
		userMessage("B (start) Enc1: %6i  Enc2: %6i\r\n", ENC1_CNT, ENC2_CNT);

		ticks_timeout = HAL_GetTick()+MOTOR_TIMEOUT;

		scara_motor_pwm(MOTOR2, SPEED_WIPE, MOTOR_CCW);
		//HAL_Delay(500);  // Move a while before waiting
		for (uint8_t i=0;i<20;i++)
		{
			ledring_wiper_loop();
			HAL_Delay(20);
		}
		stop=0;
		do {
			m1pos = constrain_pos(ENC1_CNT,m1start);
			m2pos = constrain_pos(ENC2_CNT,m2start);
			if (m2pos < ENC_VAL_MID) 		// XXX CHECK
			{
				scara_motor_pwm(MOTOR1, SPEED_WIPE, MOTOR_CW);
			}
			if (m2pos < (ENC_VAL_MID + ENC_VAL_MAX / WIPE_LINES))
			{
				scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
				stop |= 1;
			}
			if (m1pos > (ENC_VAL_MAX / WIPE_LINES))
			{
				scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
				stop |= 2;
			}
			// TIMEOUT
			if (HAL_GetTick()>ticks_timeout)
			{
				stop |=4;
				scara_motor_pwm(MOTOR1, SPEED_STOP, MOTOR_STOP);
				scara_motor_pwm(MOTOR2, SPEED_STOP, MOTOR_STOP);
			}
			if (HAL_GetTick()>ticks_last)
			{
				userMessage("B m1pos: %6i  m2pos: %6i  stop: %2i\r\n", m1pos, m2pos, stop);
				ticks_last = HAL_GetTick()+200;
			}
			ledring_wiper_loop();  // Do wiper effect
		}
		while(stop<3);

		if (stop & 0x04)   // There was a timeout
		{
			error(ERROR_WIPER2);
		}
	}
}

//void scara_wiper()
//{
//	enum statePositioning state;
//	uint16_t halfcnt=0;
//	uint16_t start=0,stop=0, val=0;
//	// Move to center
//	do
//	{
//		state=positioningScaraRobot(0.,0.); // 180°
//	}
//	while( (state == POSITIONING) );
//
//	// Set Encoders to center
//	 __HAL_TIM_SET_COUNTER(&ENC1_TMR,ENC_VAL_MID);
//	 __HAL_TIM_SET_COUNTER(&ENC2_TMR,0);
//
//	while(1)
//	{
//		// Half circle from center to outer
//		start = ENC2_CNT;
//		stop = start+ENC_VAL_MID+halfcnt;
//		userMessage("A0 Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//		motorPwmControl(MOTOR2, SPEED_WIPE, MOTOR_CW);
//		HAL_Delay(500);  // Move a while before waiting
//		do {
//			val = ENC2_CNT;
//			if (val<start) val+= ENC_VAL_MAX;	// Handle MAX->0 overflow
//			HAL_Delay(200);
//			userMessage("A1 Enc1: %6i  Enc2: %6i  start %6i   stop %6i  val %6i\r\n",
//					ENC1_CNT, ENC2_CNT, start, stop,val);
//		}
//		while(val < stop);
//		motorPwmControl(MOTOR2, SPEED_STOP, MOTOR_STOP);
//		userMessage("A Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//		// Move Motor 1 one line
//		// And also M2 to compensate
//		halfcnt += ENC_VAL_MAX / WIPE_LINES;
//		start = ENC1_CNT;
//		stop  = start + halfcnt;
//
//		motorPwmControl(MOTOR1, SPEED_WIPE/2, MOTOR_CW);
//		HAL_Delay(50);
//		do {
//			val = ENC1_CNT;
//			if (val<start) val+= ENC_VAL_MAX;	// Handle MAX->0 overflow
//		}
//		while(val < stop);
//		motorPwmControl(MOTOR1, SPEED_STOP, MOTOR_STOP);
//		userMessage("B Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//		// Half circle from outer to center
//		start = ENC2_CNT;
//		stop = start-ENC_VAL_MID+halfcnt;
//		userMessage("C0 Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//		motorPwmControl(MOTOR2, SPEED_WIPE, MOTOR_CCW);
//		HAL_Delay(500);  // Move a while before waiting
//		do {
//			val = ENC2_CNT;
//			//if (val<start) val+= ENC_VAL_MAX;	// Handle MAX->0 overflow
//			HAL_Delay(200);
//			userMessage("C1 Enc1: %6i  Enc2: %6i  start %6i   stop %6i  val %6i\r\n",
//					ENC1_CNT, ENC2_CNT, start, stop,val);
//
//		}
//		while(val > stop);
//		motorPwmControl(MOTOR2, SPEED_STOP, MOTOR_STOP);
//		userMessage("C Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//		// Move Motor 1 one line
//		halfcnt += ENC_VAL_MAX / WIPE_LINES;
//		start = ENC1_CNT;
//		stop  = start + halfcnt;
//
//		motorPwmControl(MOTOR1, SPEED_WIPE/2, MOTOR_CW);
//		HAL_Delay(50);
//		do {
//			val = ENC1_CNT;
//			if (val<start) val+= ENC_VAL_MAX;	// Handle MAX->0 overflow
//		}
//		while(val < stop);
//		motorPwmControl(MOTOR1, SPEED_STOP, MOTOR_STOP);
//		userMessage("D Enc1: %6i  Enc2: %6i  start %i   stop %i\r\n",
//				ENC1_CNT, ENC2_CNT, start, stop);
//
//	}
//}
