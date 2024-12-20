

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include <math.h>
#include "pid_controller.h"
#include "scara_robot.h"
#include "leds.h"
#define ABTASTRATE 4

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

const uint8_t M1=1;
const uint8_t M2=2;
#define RAD2DEG          (180/M_PI)
#define COUNTSPERDEGREE  (360.0/23707.0) // counts per degree OLD:23760

#define REFERENCE_SPEED_FAST  60
#define REFERENCE_SPEED_SLOW   5


pid_params_t pidParameterM1 = {
		.kp=3,
		.ki=0.1,
		.kd=0.00011,
		.Ta=ABTASTRATE,
		.threshold=0.1,
		.y_max=12,
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
		.y_max=12,
		.y_min=0,
		.esum =0,
		.eold=0
};

uint32_t lt_PID=0;

void motorPwmControl(int motor_nr, int speed, int direction){

	const int cw = 1,ccw = 0;
	switch(motor_nr)
	{
	case 1:
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,abs(speed));
		if ( direction == cw ){
			// Direction Clockwise
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_RESET);
		} else if ( direction == ccw ){
			//Direction Counter clockwise
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(_1IN_A_GPIO_Port, _1IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_1IN_B_GPIO_Port, _1IN_B_Pin, GPIO_PIN_RESET);
		}
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,abs(speed));
		if ( direction == cw ){
			// Direction Clockwise
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_SET);
		} else if ( direction == ccw ){
			//Direction Counter clockwise
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(_2IN_A_GPIO_Port, _2IN_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(_2IN_B_GPIO_Port, _2IN_B_Pin, GPIO_PIN_RESET);
		}
		break;
	default: userMessage("motorPwmControl: Motor Nr <> 1,2\r\n");
		break;
	}

}

uint8_t referenceMotors() {

	int step;

	const color_t refcolor=RGB(0,0,64);
	ledring_black();

	for (step = 1; step <= 7; step++)
	{
		ledring_set_rng_color((LEDRING_CNT/7.0+0.5)*(step-1),(LEDRING_CNT/7.0+0.5)*step-1,refcolor);
		ledring_update();

		switch (step) {

			case 1: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 1 Motor 1 fast forward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_SET)){
						motorPwmControl(1, REFERENCE_SPEED_FAST, 1);
					}
					motorPwmControl(1, 0, 1);
					break;

			case 2: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 2 Motor 1 slowly backward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_RESET)) {
						motorPwmControl(1, REFERENCE_SPEED_SLOW, 0);
					}
					HAL_Delay(1000);
					motorPwmControl(1, 0, 1);
					break;

			case 3: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 3 Motor 1 slowly forward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_1_GPIO_Port, Ref_S_1_Pin)==GPIO_PIN_SET)) {
						motorPwmControl(1, REFERENCE_SPEED_SLOW, 1);
					}
					motorPwmControl(1, 0, 1);
					__HAL_TIM_SET_COUNTER(&htim2,11854);
					debugPrint(DEBUG_ROBOT,"referenceMotors: Encoder counter 1 initialized...\r\n");
					break;

			case 4: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 4 Motor 2 fast forward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_SET)) {
						motorPwmControl(2, REFERENCE_SPEED_FAST, 1);
					}
					motorPwmControl(2, 0, 1);
					break;

			case 5: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 5 Motor 2 slowly backward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_RESET)) {
						motorPwmControl(2, REFERENCE_SPEED_SLOW, 0);
					}
					HAL_Delay(1000);
					motorPwmControl(2, 0, 1);
					break;

			case 6: debugPrint(DEBUG_ROBOT,"referenceMotors: Step 6 Motor 2 slowly forward\r\n");
					while ((HAL_GPIO_ReadPin(Ref_S_2_GPIO_Port, Ref_S_2_Pin)==GPIO_PIN_SET)) {
						motorPwmControl(2, REFERENCE_SPEED_SLOW, 1);
					}
					motorPwmControl(2, 0, 1);
					__HAL_TIM_SET_COUNTER(&htim3,0);
					debugPrint(DEBUG_ROBOT,"referenceMotors: Encoder counter 2 initialized...\r\n");
					debugPrint(DEBUG_ROBOT,"referenceMotors: Done...\r\n");
					break;

			default:break;
		}
	}
	return 0;
}

int motorTravelDirection(float actualPosition, float targetPosition ){

	const int directionClockwise = 1,directionCounterclockwise = 0;
	float positionDifference = 0.0, positionDifferenceComp=0.0;

	if(actualPosition < targetPosition){
		debugPrint(DEBUG_ROBOT,"actualPosition < targetPosition\r\n");
		positionDifference = targetPosition - actualPosition;
		positionDifferenceComp = 360 - positionDifference;
		debugPrint(DEBUG_ROBOT,"positionDifference: %f , positionDifferenceComp%f \r\n",positionDifference,positionDifferenceComp);
		if  ( positionDifference < positionDifferenceComp ) {
			return directionClockwise;
		} else {
			return directionCounterclockwise;
		}
	} else {
		debugPrint(DEBUG_ROBOT,"actualPosition > targetPosition\r\n");
		positionDifference = actualPosition - targetPosition;
		positionDifferenceComp = 360 - positionDifference;
		debugPrint(DEBUG_ROBOT,"positionDifference: %f , positionDifferenceComp%f\r\n",positionDifference,positionDifferenceComp);
		if  ( positionDifference < positionDifferenceComp ) {
			return directionCounterclockwise;
		} else {
			return directionClockwise;
		}
	}
}

enum statePositioning positioningScaraRobot(float theta, float rho){

	static enum statePositioning statePos=IN_POSITION;;
	static uint8_t startPositioning=0;
	static uint8_t directionM1=0,directionM2=0;
	static float setpointPwmM1,setpointPwmM2;

	float positionDegreeM1 =getPositionM1();
	float positionDegreeM2 =getPositionM2();

	const float positionWindowM1=0.1,positionWindowM2=0.1;

	static uint8_t positionReachedM1=0, positionReachedM2=0;

	scaraAngles_t scaraTargetAnglesDeg;

	float thetaUnitCircle = fmodf( theta, (2.*M_PI) );

	if (thetaUnitCircle < 0 ) {thetaUnitCircle = (2.*M_PI) + thetaUnitCircle;}
	debugPrint(DEBUG_ROBOT,"PSR: thetaUCir= %5.2f \r\n",thetaUnitCircle);

	scaraTargetAnglesDeg = calculateAngleScara( thetaUnitCircle , rho);
	debugPrint(DEBUG_ROBOT,"PSR: posDegM1 %6.1f, posDegM2 %6.1f\r\n",positionDegreeM1, positionDegreeM2);
	debugPrint(DEBUG_ROBOT,"PSR: targetM1 %6.1f, targetM2 %6.1f\r\n", scaraTargetAnglesDeg.phi1,scaraTargetAnglesDeg.phi2);

	if (!startPositioning){ // Richtung fuer einen Positioniervorgang festsetzen
		startPositioning = 1;
		debugPrint(DEBUG_ROBOT,"PSR: Posi start\r\n");
	}

	if(timeTick(ABTASTRATE,&lt_PID)){ // Abtastrate des PID-Reglers 4ms
		debugPrint(DEBUG_ROBOT,"PSR: timerTick\r\n");

		// Wenn Ziel erreicht neues Ziel übergeben
		if ( checkPositionWindow(positionDegreeM1,scaraTargetAnglesDeg.phi1,positionWindowM1) == 1 ){
			motorPwmControl(M1,0,0);
			positionReachedM1 = 1;
			setpointPwmM1=0;
			pidParameterM1.esum=0;
			pidParameterM1.eold=0;
			debugPrint(DEBUG_ROBOT,"PSR: PosM1 done!\r\n");
		}

		if ( checkPositionWindow(positionDegreeM2,scaraTargetAnglesDeg.phi2,positionWindowM2) == 1 ){
			motorPwmControl(M2,0,0);
			positionReachedM2 = 1;
			setpointPwmM2=0;
			pidParameterM2.esum=0;
			pidParameterM2.eold=0;
			debugPrint(DEBUG_ROBOT,"PSR: PosM2 done!\r\n");
		}

		if(!positionReachedM1){
			setpointPwmM1=digPIDControl(positionDegreeM1,scaraTargetAnglesDeg.phi1,&pidParameterM1);
			//debugPrint(DEBUG_ROBOT,"PSR: pidM1.esum = %f\r\n;",pidParameterM1.esum);
		}

		if (!positionReachedM2){
			setpointPwmM2=digPIDControl(positionDegreeM2,scaraTargetAnglesDeg.phi2,&pidParameterM2);
			//debugPrint(DEBUG_ROBOT,"PSR: pidM2.esum = %f\r\n;",pidParameterM2.esum);
		}

		debugPrint(DEBUG_ROBOT,"PSR: setPwmM1 %4.1f, setPwmM2 %4.1f\r\n", setpointPwmM1, setpointPwmM2);

		if (positionReachedM1 && positionReachedM2){
			startPositioning = 0;
			positionReachedM1 = 0;
			positionReachedM2 = 0;
		}

		if (startPositioning ==1 ){

			directionM1 = motorTravelDirection(positionDegreeM1, scaraTargetAnglesDeg.phi1 );
			directionM2 = motorTravelDirection(positionDegreeM2, scaraTargetAnglesDeg.phi2 );
			debugPrint(DEBUG_ROBOT,"PSR:dirM1 %d, dirM2 %d  \r\n", directionM1, directionM2);

			if( !positionReachedM1 ) {
				motorPwmControl(M1,setpointPwmM1,directionM1);
			}else {
				motorPwmControl(M1,0,0);
			}
			if( !positionReachedM2 ) {
				motorPwmControl(M2,setpointPwmM2,directionM2);
			}else {
				motorPwmControl(M2,0,0);
			}
			statePos = POSITIONING;
			return statePos;

		} else {

			stopScaraRobot();
			debugPrint(DEBUG_ROBOT,"PSR: ScaraRobot stopped \r\n");
			statePos = IN_POSITION;
			return statePos;
		}
	}
	debugPrint(DEBUG_ROBOT,"PSR: no timetick \r\n");
	return statePos;
}

void stopScaraRobot(){
	motorPwmControl(M1,0,0);
	motorPwmControl(M2,0,0);
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

float getPositionM1(){

	float positionDegree =(float)__HAL_TIM_GET_COUNTER(&htim3)*COUNTSPERDEGREE ;
	return positionDegree;

}

float getPositionM2(){

	float positionDegree =(float)__HAL_TIM_GET_COUNTER(&htim2)*COUNTSPERDEGREE ;
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
    scaraValues.phi1 = ( ( theta - alpha ) ) * RAD2DEG;

	if (scaraValues.phi1 < 0){scaraValues.phi1 = 360 + scaraValues.phi1;}
	if (scaraValues.phi1 > 360){scaraValues.phi1 = scaraValues.phi1 - 360;}

    debugPrint(DEBUG_ROBOT,"calculatingAngleScara: angleM1= %f \r\n", scaraValues.phi1);

	float beta = acosf( ( ( L2*L2 + L1*L1 - dist*dist ) / ( 2*L2*L1 ) ) );
	scaraValues.phi2 = ( (M_PI - beta) + ( theta - alpha ) )   * RAD2DEG;

	if (scaraValues.phi2 < 0){scaraValues.phi2 = 360 + scaraValues.phi2;}
	if (scaraValues.phi2 > 360){scaraValues.phi2 = scaraValues.phi2 - 360;}

	debugPrint(DEBUG_ROBOT,"calculatingAngleScara: angleM2= %f \r\n", scaraValues.phi2);

    if ( rho == 0 ){
    	scaraValues.phi1 = 0.0;
    	scaraValues.phi2 = 180.0;
    }
    return scaraValues;
}
