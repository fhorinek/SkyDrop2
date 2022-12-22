/*
 * lsm303d.h
 *
 *  Created on: 11.8.2022
 *      Author: horinek
 */


#ifndef LSM6DS_H_
#define LSM6DS_H_

class Lsm6ds;

#include "common.h"
#include "devices.h"



class Lsm6ds
{
//private:
public:
	I2c * i2c;

	void Init(I2c *i2c);
	void Deinit();
	void Reset();

	bool SelfTest();

	void Write(uint8_t adr, uint8_t data);
	uint8_t Read(uint8_t adr);

	void StartReadFIFOStream();
	void ReadFIFOStreamAvg(volatile vector_i16_t * gyro, volatile vector_i16_t * acc);
};


#endif /* LSM6DS_H_ */
