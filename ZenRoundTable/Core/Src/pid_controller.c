/*
 * PID.c
 *
 *  Created on: Dec 20, 2023
 *      Author: steff
 */
#include <pid_controller.h>
#include "main.h"
#include <stdio.h>


float digPIDControl(float r,float w, pid_params_t *pidparams){

	  float e= w - r;
	  static float y = 0;

	  if ((e >= pidparams->threshold)||(e <= (pidparams->threshold*(-1))))
	  {

	  if ((y < pidparams->y_max)&&(y > (pidparams->y_max*(-1)))) {
		  pidparams->esum = pidparams->esum + e;
		  debugPrint(DEBUG_PID,"digPIDControl: esum= %f \r\n e = %f \r\n y = %f \r\n",
				  	 pidparams->esum, e,y);
	    }

	    y = (pidparams->kp*e)+
	    	(pidparams->ki*pidparams->Ta*pidparams->esum)+
			(pidparams->kd/pidparams->Ta*(e-pidparams->eold));


	    pidparams->eold = e;
	  }
	  else {
		  y=0;
	  }

	  if (y > pidparams->y_max) {y = pidparams->y_max; }
	  if (y < (pidparams->y_max)*(-1)) { y = (pidparams->y_max)*(-1);}

	  if ((y>0)&&(y < pidparams->y_min)) {y = pidparams->y_min; }
	  if ((y<0)&&(y>(pidparams->y_min*(-1)))) { y = (pidparams->y_min)*(-1);}

	  return y;
}

int timeTick(uint32_t tick, uint32_t *last_time){

	if (*last_time==0){
		*last_time=HAL_GetTick();
	}

	if ( (HAL_GetTick()-*last_time) >= tick ){
		*last_time=HAL_GetTick();
		return 1;
	} else {
		return 0;
	}

}

