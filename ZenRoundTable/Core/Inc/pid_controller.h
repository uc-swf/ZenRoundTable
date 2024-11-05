/*
 * PID.h
 *
 *  Created on: Dec 20, 2023
 *      Author: steff
 */

#include <stdint.h>

#ifndef SRC_PID_STRUCT_H_
#define SRC_PID_STRUCT_H_

typedef struct
{

	float kp;
	float ki;
	float kd;
	float Ta;
	float y_max;
	float y_min;
	float threshold;
	float esum;
	float eold;

} pid_params_t;

float digPIDControl(float r,float w, pid_params_t *pidparams);
int timeTick(uint32_t tick, uint32_t *last_time);

#endif /* SRC_PID_H_ */
