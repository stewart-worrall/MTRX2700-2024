/*
 * ptu_i2c.h
 *
 *  Created on: Apr 14, 2023
 *      Author: stew
 */

#ifndef INC_PTU_I2C_H_
#define INC_PTU_I2C_H_

#include "stm32f3xx_hal.h"

void initialise_ptu_i2c(I2C_HandleTypeDef *i2c);

#endif /* INC_PTU_I2C_H_ */
