/*
 * devices.h
 *
 *  Created on: 30.7.2014
 *      Author: horinek
 */

#ifndef DEVICES_H_
#define DEVICES_H_

#include <drivers/sensors/lsm6ds.h>
#include "../../skydrop.h"


#include "ms5611.h"
#include "lsm6ds.h"
#include "gnss_ublox_m8.h"

extern I2c mems_i2c;
extern Lsm6ds lsm6ds;
extern MS5611 ms5611;


int32_t to_dec_3(int64_t c);
int16_t to_dec_2(int32_t c);
int8_t to_dec_1(int8_t c);

bool mems_i2c_init();
bool mems_i2c_selftest();


#endif /* DEVICES_H_ */
