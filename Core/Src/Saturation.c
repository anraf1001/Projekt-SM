/*
 * Saturation.c
 *
 *  Created on: Jan 10, 2022
 *      Author: anraf1001
 */

#include "Saturation.h"

#include <stddef.h>

float calculate_saturation(float u, Saturation* saturation) {
    if (saturation != NULL) {
        if (u > saturation->upper_bound) {
            u = saturation->upper_bound;
        } else if (u < saturation->lower_bound) {
            u = saturation->lower_bound;
        }
    }

    return u;
}
