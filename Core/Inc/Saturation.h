/*
 * Saturation.h
 *
 *  Created on: Jan 10, 2022
 *      Author: anraf1001
 */

#ifndef INC_SATURATION_H_
#define INC_SATURATION_H_

typedef struct {
    float lower_bound;
    float upper_bound;
} Saturation;

float calculate_saturation(float u, Saturation* saturation);

#endif /* INC_SATURATION_H_ */
