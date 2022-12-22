/*
 * lsm303d.cc
 *
 *  Created on: 11.8.2022
 *      Author: horinek
 */

#include <drivers/sensors/lsm6ds.h>
#include "debug_on.h"

#define LSM_ADDRESS		0x6A
#define LSM_ID			0x6C
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
	this->Write(0x07, LSM_FIFO_SIZE * 2);
	//FIFO_CTRL2 - STOP_ON_WTM
	this->Write(0x08, 0b00000000);
	//FIFO_CTRL3 - Store GYRO an XL data @ 1667Hz
	this->Write(0x09, 0b10001000);
	//FIFO_CTRL4 - Continuous mode
	this->Write(0x0A, 0b00000110);

	//CTRL1_XL - 1.66 kHz, +/-8g
	this->Write(0x10, 0b10001100);
	//CTRL2_G - 1.66 kHz, 2000 dps, Gyro enabled
	this->Write(0x11, 0b10001100);
	//CTRL3_C - BDU, Auto increment
	this->Write(0x12, 0b01000100);
	//CTRL4_C - STOP_ON_FTH
	//CTRL5_C
	//CTRL6_C

//	for (uint8_t i = 0x06; i < 0x14; i++)
//		DEBUG("CFG %02x == %02X\n", i, this->Read(i));
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
	this->i2c->Write(0x78);// | 0b10000000);
	this->i2c->StartTransmittion(LSM_ADDRESS, (LSM_FIFO_SIZE * 7) * 2);
}

void Lsm6ds::ReadFIFOStreamAvg(volatile vector_i16_t * gyro, volatile vector_i16_t * acc)
{
//	return;

	vector_i32_t gyro_acu = {0};
	vector_i32_t acc_acu = {0};

	uint8_t acc_cnt = 0;
	uint8_t gyro_cnt = 0;

	this->i2c->Wait();

	byte2 tmp;

//	DEBUG("-----FIFO READ-----\n");
	for (uint8_t i = 0; i < LSM_FIFO_SIZE * 2; i++)
	{
		uint8_t tag = this->i2c->Read() & 0b11111000;
		vector_i32_t * acu;

		if (tag == 0b00001000)
		{
			acu = &gyro_acu;
//			DEBUG("G:");
			gyro_cnt++;
		}
		else if (tag == 0b00010000)
		{
			acu = &acc_acu;
//			DEBUG("A:");
			acc_cnt++;
		}
		else
		{
//			DEBUG("WTF? %02X\n", tag);

			for (uint8_t i = 0 ; i<6; i++)
				this->i2c->Read();

			continue;
		}

		tmp.uint8[0] = this->i2c->Read();
		tmp.uint8[1] = this->i2c->Read();
		acu->x += tmp.int16;
//		DEBUG("%d;", tmp.int16);

		tmp.uint8[0] = this->i2c->Read();
		tmp.uint8[1] = this->i2c->Read();
		acu->y += tmp.int16;
//		DEBUG("%d;", tmp.int16);

		tmp.uint8[0] = this->i2c->Read();
		tmp.uint8[1] = this->i2c->Read();
		acu->z += tmp.int16;
//		DEBUG("%d\n", tmp.int16);
	}


	gyro->x = (int32_t)gyro_acu.x / gyro_cnt;
	gyro->y = (int32_t)gyro_acu.y / gyro_cnt;
	gyro->z = (int32_t)gyro_acu.z / gyro_cnt;
//	DEBUG("G: %d %d %d\n", gyro->x, gyro->y, gyro->z);

	acc->x = -(int32_t)acc_acu.x / acc_cnt;
	acc->y = -(int32_t)acc_acu.y / acc_cnt;
	acc->z = -(int32_t)acc_acu.z / acc_cnt;
//	DEBUG("A: %d %d %d\n", acc->x, acc->y, acc->z);
}

