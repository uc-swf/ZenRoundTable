/*
 * motor_driver.h
 *
 *  Created on: Dec 19, 2023
 *      Author: steff
 */

#include <stdint.h>

#ifndef SRC_MOTOR_DRIVER_H_
#define SRC_MOTOR_DRIVER_H_

enum statePositioning {IN_REFERENCE,POSITIONING,IN_POSITION};
typedef struct { float phi1; float phi2;} scaraAngles_t;

void motorPwmControl( int motor_nr, int speed, int direction );
void referenceMotors();
int motorTravelDirection(float act_pos, float dest_pos );
enum statePositioning positioningScaraRobot(float theta, float rho);
void stopScaraRobot();
uint8_t checkPositionWindow(float actualPosition, float targetPosition, float positionWindow);
float getPositionM1();
float getPositionM2();
scaraAngles_t calculateAngleScara(float theta, float rho);
#endif /* SRC_MOTOR_DRIVER_H_ */
