/*
 * lsm303d.cc
 *
 *  Created on: 11.8.2022
 *      Author: horinek
 */

/*

#include <drivers/sensors/lsm6ds.h>
#include "debug_off.h"

#define LSM_ADDRESS		0x6A
#define LSM_ID			0x6A
#define	LSM_FIFO_SIZE	16

void Lsm6ds::Init(I2c *i2c)
{
	this->i2c = i2c;

	//need time to boot up
	_delay_ms(10);

	//WHO_AM_I
	uint8_t id = this->Read(0x0F);
	DEBUG("lsm id %02X\n", id);
	assert(id == LSM_ID);

	//FIFO_CTRL1 - fifo size
	this->Write(0x06, LSM_FIFO_SIZE * 2 * 3);
	//FIFO_CTRL3 - Store GYRO an XL data
	this->Write(0x08, 0b00001001);
	//FIFO_CTRL4 - STOP_ON_FTH
	this->Write(0x09, 0b10000000);
	//FIFO_CTRL5 - ODR166 kHz,
	this->Write(0x0A, 0b01000110);

	//CTRL1_XL - 1.66 kHz, +/-8g
	this->Write(0x10, 0b10001100);
	//CTRL2_G - 1.66 kHz, 2000 dps, Gyro enabled
	this->Write(0x11, 0b10001100);
	//CTRL3_C - BDU, Auto increment
	this->Write(0x12, 0b01000100);
	//CTRL4_C - STOP_ON_FTH
	//CTRL5_C
	//CTRL6_C

	for (uint8_t i = 0x06; i < 0x14; i++)
		DEBUG("CFG %02x == %02X\n", i, this->Read(i));
}

void Lsm6ds::Deinit()
{
	//CTRL1_XL - Power down
	this->Write(0x10, 0b00000000);
	//CTRL2_G - Power down
	this->Write(0x11, 0b00000000);
}

void Lsm6ds::Write(uint8_t adr, uint8_t data)
{
	this->i2c->Wait();

	this->i2c->Write(adr);
	this->i2c->Write(data);
	this->i2c->StartTransmittion(LSM_ADDRESS, 0);
}

uint8_t Lsm6ds::Read(uint8_t adr)
{
	this->i2c->Wait();

	this->i2c->Write(adr);
	this->i2c->StartTransmittion(LSM_ADDRESS, 1);
	this->i2c->Wait();

	return this->i2c->Read();
}

void Lsm6ds::Reset()
{
	this->Write(0x12, 0b11000100); //CTRL3_C <- BOOT

	_delay_ms(1);
	this->Write(0x12, 0b01000100); //CTRL3_C -> BOOT
	_delay_ms(1);
}

bool Lsm6ds::SelfTest()
{
	uint8_t id = this->Read(0x0F);

	return (id == LSM_ID);
}

void Lsm6ds::StartReadFIFOStream()
{
//	DEBUG("ST1 %02X, ST2 %02X\n", this->Read(0x3A), this->Read(0x3B));
//	DEBUG("ST3 %02X, ST4 %02X\n", this->Read(0x3C), this->Read(0x3D));


//	for (uint8_t i = 0x22; i <= 0x2C; i += 2)
//	{
//		byte2 tmp;
//
//		this->i2c->Wait();
//		this->i2c->Write(i);// | 0b10000000);
//		this->i2c->StartTransmittion(LSM_ADDRESS, 2);
//
//		this->i2c->Wait();
//		tmp.uint8[0] = this->i2c->Read();
//		tmp.uint8[1] = this->i2c->Read();
//
//		DEBUG("VAL %02x == %d\n", i, tmp.int16);
//	}
//	DEBUG("-----\n\n\n");

	this->i2c->Wait();
	this->i2c->Write(0x3A);// | 0b10000000);
	this->i2c->StartTransmittion(LSM_ADDRESS, 4 + (LSM_FIFO_SIZE * 6) * 2);
}


void Lsm6ds::ReadFIFOStreamAvg(volatile vector_i16_t * gyro, volatile vector_i16_t * acc)
{
	this->i2c->Wait();

	byte2 tmp;
	uint8_t status[4];

	for (uint8_t i = 0; i < 4; i++)
	{
		status[i] = this->i2c->Read();
//		DEBUG("status%u = %02X %u\n", i+1, status[i], status[i]);
	}

	uint8_t size = min(status[0], (LSM_FIFO_SIZE * 3) * 2);

	uint32_t acu[6] = {0};
	uint8_t acu_cnt[6] = {0};

	uint8_t index = status[2];

	DEBUG("SZ/PT: %u %u\n", size, index);
	for (uint8_t i = 0; i < size; i++)
	{
		tmp.uint8[0] = this->i2c->Read();
		tmp.uint8[1] = this->i2c->Read();

		acu[index] += tmp.int16;
//		DEBUG(" %u %u = %d\n", i, index, tmp.int16);
		acu_cnt[index] += 1;

		index = (index + 1) % 6;
	}

//	for (uint8_t i = 0; i < 6; i++)
//	{
//		DEBUG("%u %ld %u\n", i, acu[i], acu_cnt[i]);
//	}

	gyro->x = (int32_t)acu[0] / acu_cnt[0];
	gyro->y = (int32_t)acu[1] / acu_cnt[1];
	gyro->z = (int32_t)acu[2] / acu_cnt[2];
	DEBUG("G: %d %d %d\n", gyro->x, gyro->y, gyro->z);

	acc->x = -(int32_t)acu[3] / acu_cnt[3];
	acc->y = -(int32_t)acu[4] / acu_cnt[4];
	acc->z = -(int32_t)acu[5] / acu_cnt[5];
	DEBUG("A: %d %d %d\n", acc->x, acc->y, acc->z);
}

*/
