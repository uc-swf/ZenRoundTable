/*
 * motor_driver.h
 *
 *  Created on: Dec 19, 2023
 *      Author: steff
 */

#include <stdint.h>

#ifndef SRC_MOTOR_DRIVER_H_
#define SRC_MOTOR_DRIVER_H_

#define ENC1_TMR   htim3
#define ENC2_TMR   htim2

#define ENC1_CNT   __HAL_TIM_GET_COUNTER(&ENC1_TMR)
#define ENC2_CNT   __HAL_TIM_GET_COUNTER(&ENC2_TMR)


#define ENC_VAL_MAX (23707-1)
#define ENC_VAL_MID (ENC_VAL_MAX/2)
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR_TIMEOUT 30000	    // Timeout ticks
#define SPEED_REF_FAST 70
#define SPEED_REF_SLOW  6
#define SPEED_STOP 0

// Wiper routine settings
#define SPEED_WIPE  70
#define WIPE_LINES  70		// Number of half circles

#define MOTOR_CCW	0
#define MOTOR_CW	1
#define MOTOR_STOP  100

enum statePositioning {IN_REFERENCE,POSITIONING,IN_POSITION};
typedef struct { float phi1; float phi2;} scaraAngles_t;

void scara_motor_pwm(int8_t motor_nr, int16_t speed, uint8_t direction);
uint16_t scara_reference();
int motorTravelDirection(float act_pos, float dest_pos );
enum statePositioning positioningScaraRobot(float theta, float rho);
void scara_stop_all();
uint8_t checkPositionWindow(float actualPosition, float targetPosition, float positionWindow);
float scara_get_pos_m1();
float scara_get_pos_m2();
scaraAngles_t calculateAngleScara(float theta, float rho);
void scara_wiper();

#endif /* SRC_MOTOR_DRIVER_H_ */
