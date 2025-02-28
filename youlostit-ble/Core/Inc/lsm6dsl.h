/*
 * lsm6dsl.c
 *
 *  Created on: Feb 4, 2025
 *      Author: dhruvsusheelkar
 */
#ifndef __LSM6DSL_H
#define __LSM6DSL_H

#include <stdint.h>

void lsm6dsl_init(void);
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif  // __LSM6DSL_H

