/*
 * PID_Controller.c
 *
 *  Created on: Jan 10, 2022
 *      Author: anraf1001
 */

#include "PID_Controller.h"

#include <stddef.h>
#include <math.h>

PID_Controller PID_Create(float Kp, float Ki, float Kd, float Tp) {
    PID_Controller controller = {.Kp = Kp, .Ki = Ki, .Kd = Kd, .Tp = Tp, .last_error = 0, .last_integral = 0, .saturation = NULL, .windup = false};
    return controller;
}

PID_Controller PID_Create_Saturation(float Kp, float Ki, float Kd, float Tp, Saturation* saturation) {
    PID_Controller controller = PID_Create(Kp, Ki, Kd, Tp);
    controller.saturation = saturation;
    return controller;
}

float calculate_pid(PID_Controller* controller, float error) {
    float P = controller->Kp * error;

    if (!controller->windup) {
        controller->last_integral += error + controller->last_error;
    }
    float I = controller->Ki * controller->last_integral * (controller->Tp / 2.f);

    float D = controller->Kd * (error - controller->last_error) / controller->Tp;
    controller->last_error = error;

    float u = P + I + D;
    float u_saturation = calculate_saturation(u, controller->saturation);
    if (fabs(u) > fabs(u_saturation)) {
        controller->windup = true;
    } else {
        controller->windup = false;
    }
    return u_saturation;
}
