/*
 * error.h
 *
 *  Created on: Dec 3, 2024
 *      Author: tobi
 */

#ifndef ERROR_H_
#define ERROR_H_

#include <stdint.h>

#define ERROR_ENC1		  123
#define ERROR_ENC2		  132

#define ERROR_REF1_SEARCH 233
#define ERROR_REF1_FINE1  213
#define ERROR_REF1_FINE2  231

#define ERROR_REF2_SEARCH 222
#define ERROR_REF2_FINE1  212
#define ERROR_REF2_FINE2  221

#define ERROR_WIPER1	  311
#define ERROR_WIPER2	  312

#define ERROR_SDMOUNT     331
#define NO_ERROR 0

void error(uint16_t err);


#endif /* ERROR_H_ */
