/*
 * PID_Controller.h
 *
 *  Created on: Jan 10, 2022
 *      Author: anraf1001
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include <stdbool.h>

#include "Saturation.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Tp;

    float last_error;
    float last_integral;

    Saturation* saturation;
    bool windup;
} PID_Controller;

PID_Controller PID_Create(float Kp, float Ki, float Kd, float Tp);
PID_Controller PID_Create_Saturation(float Kp, float Ki, float Kd, float Tp, Saturation* saturation);

float calculate_pid(PID_Controller* controller, float error);

#endif /* INC_PID_CONTROLLER_H_ */
