/*
 * gnss_ublox_m8.cc
 *
 *  Created on: 4. 5. 2020
 *      Author: horinek
 */
//#define DEBUG_LEVEL DEBUG_DEBUG
#include "gnss_ublox_m8.h"

#include "../../fc/fc.h"
#include "../uart.h"
#include "../../debug_off.h"

//Pins
//#define	GPS_SW_EN
//#define	GPS_PPS
//#define	GPS_RESET

#define GPS_UART_RX_SIZE	250
#define GPS_UART_TX_SIZE	40

uint8_t gps_uart_rx_buffer[GPS_UART_RX_SIZE];
uint8_t gps_uart_tx_buffer[GPS_UART_TX_SIZE];

Usart gps_uart(GPS_UART_RX_SIZE, gps_uart_rx_buffer, GPS_UART_TX_SIZE, gps_uart_tx_buffer);
CreateStdOut(gps_out, gps_uart.Write);

volatile bool ublox_init_ok = false;

//DMA buffer
static uint8_t ublox_last_command;
static uint32_t ublox_last_command_time;
static uint32_t ublox_start_time;
static uint16_t ublox_read_index = 0;

static enum {
	PM_INIT,
	PM_IDLE,
	PM_SYNC,
	PM_CLASS,
	PM_ID,
	PM_LEN_LO,
	PM_LEN_HI,
	PM_PAYLOAD,
	PM_CK_A,
	PM_CK_B,
	PM_RESET
} ublox_parse_mode;

void ublox_deinit()
{
	GpioSetDirection(GPS_EN, INPUT);
	GpioSetDirection(GPS_RESET, INPUT);

	fc.gps_data.valid = false;
	fc.gps_data.fix = 0;
	fc.gps_data.fix_cnt = 0;

	GpioSetPull(GPS_TIMER, gpio_totem);

	ublox_init_ok = false;
	gps_uart.Stop();
	GPS_UART_PWR_OFF;
}

bool ublox_selftest()
{
	return ublox_init_ok;
}

void ublox_init()
{
	DEBUG("UBX init\n");

	GPS_UART_PWR_ON;
	gps_uart.Init(GPS_UART, 9600);
	gps_uart.SetInterruptPriority(MEDIUM);

	gps_uart.SetupRxDMA(GPS_UART_DMA_CH, GPS_UART_DMA_TRIGGER);

	GpioSetDirection(GPS_EN, OUTPUT);	 //active high
	GpioWrite(GPS_EN, LOW);

	GpioSetDirection(GPS_TIMER, INPUT);  //active low, otherwise open-drain
	GpioSetPull(GPS_TIMER, gpio_pull_up);
	GpioSetInterrupt(GPS_TIMER, gpio_interrupt1, gpio_falling);

	GpioSetDirection(GPS_RESET, OUTPUT); //active low
	GpioWrite(GPS_RESET, LOW);

	GpioWrite(GPS_EN, HIGH);
	_delay_ms(10);
	GpioWrite(GPS_RESET, LOW);
	_delay_ms(20);
	GpioWrite(GPS_RESET, HIGH);

	fc.gps_data.valid = false;
	fc.gps_data.new_sample = 0x00;
	fc.gps_data.fix = 0;
	fc.gps_data.fix_cnt = 0;

	for (uint8_t i = 0; i < GPS_SAT_CNT; i++)
	{
		fc.gps_data.sat_id[i] = 0;
		fc.gps_data.sat_snr[i] = 0;
	}

	//reset cache
	strcpy_P((char *)fc.gps_data.cache_igc_latitude, PSTR("0000000N"));
	strcpy_P((char *)fc.gps_data.cache_igc_longtitude, PSTR("00000000E"));
	strcpy_P((char *)fc.gps_data.cache_gui_latitude, PSTR("---"));
	strcpy_P((char *)fc.gps_data.cache_gui_longtitude, PSTR("---"));
	fc.gps_data.altitude = 0;
	fc.gps_data.vacc = fc.gps_data.hacc = 0.0;

	ublox_read_index = 0;
}

static void gnss_set_baudrate(uint32_t baud)
{
	gps_uart.Stop();
	gps_uart.Init(GPS_UART, baud);

	gps_uart.SetInterruptPriority(MEDIUM);
	gps_uart.SetupRxDMA(GPS_UART_DMA_CH, GPS_UART_DMA_TRIGGER);
}

static void ublox_send(uint8_t cls, uint8_t id, uint8_t len, uint8_t * payload)
{
	uint8_t buff[len + 8];
	byte2 blen;

	//sync word
	buff[0] = 0xB5;
	buff[1] = 0x62;

	//class
	buff[2] = cls;

	//id
	buff[3] = id;

	//length
	blen.uint16 = len;
	buff[4] = blen.uint8[0];
	buff[5] = blen.uint8[1];

	//payload
	memcpy(buff + 6, payload, len);

	//checksum
	uint8_t ck_a = 0;
	uint8_t ck_b = 0;
	for (uint8_t i = 2; i < len + 6; i++)
	{
		ck_a += buff[i];
		ck_b += ck_a;
	}
	buff[len + 6] = ck_a;
	buff[len + 7] = ck_b;

	DEBUG("UBX >>> %02X, %02X len %u\n", cls, id, len);
	for (uint8_t i = 0; i < len; i++)
		DEBUG("%02X ", payload[i]);
	DEBUG("\n");

	for (uint8_t i = 0; i < len + 8; i++)
	{
		gps_uart.Write(buff[i]);
		DEBUG("%02X ", buff[i]);
	}
	DEBUG("\n");

	gps_uart.FlushTxBuffer();
	_delay_ms(10);
}

static void ublox_command(uint8_t command)
{
	ublox_last_command = command;
	ublox_last_command_time = task_get_ms_tick();

	DEBUG("sending command %u\n", command);

	switch (command)
	{
		case(0):
		{
			struct {
				uint8_t portID;
				uint8_t reserved1;
				uint16_t txReady;
				uint32_t mode;
				uint32_t baudRate;
				uint16_t inProtoMask;
				uint16_t outProtoMask;
				uint16_t flags;
				uint16_t reserved2;
			} ubx_cfg_prt;

			ubx_cfg_prt.portID = 1;
			ubx_cfg_prt.reserved1 = 0;
			ubx_cfg_prt.txReady = 0;
			ubx_cfg_prt.mode = (0b00100011000000); //1 stop, No parity, 8Bit;
			ubx_cfg_prt.baudRate = 115200;
			ubx_cfg_prt.inProtoMask = 0x0001;
			ubx_cfg_prt.outProtoMask = 0x0001;
			ubx_cfg_prt.flags = 0;
			ubx_cfg_prt.reserved2 = 0;

			DEBUG("ubx_cfg_prt\n");
			ublox_send(0x06, 0x00, sizeof(ubx_cfg_prt), (uint8_t *)&ubx_cfg_prt);
			gnss_set_baudrate(115200);
			break;
		}

		case(1):
		{
			struct {
				uint16_t measRate;
				uint16_t navRate;
				uint16_t timeRef;
			} ubx_cfg_rate;

			ubx_cfg_rate.measRate = 1000;
			ubx_cfg_rate.navRate = 1;
			ubx_cfg_rate.timeRef = 0;

			DEBUG("ubx_cfg_rate\n");
			ublox_send(0x06, 0x08, sizeof(ubx_cfg_rate), (uint8_t *)&ubx_cfg_rate);
			break;
		}

		case(2):
		case(3):
		case(4):
		case(5):
		case(6):
		{
			struct {
				uint8_t msgClass;
				uint8_t msgID;
				uint8_t rate;
			} ubx_cfg_msg;

			//output sat info
			ubx_cfg_msg.msgClass = 0x01;
			ubx_cfg_msg.rate = 1;

			switch (command)
			{
			case(2):
				ubx_cfg_msg.msgID = 0x35; //sat info
			break;
			case(3):
				ubx_cfg_msg.msgID = 0x03; //status
			break;
			case(4):
				ubx_cfg_msg.msgID = 0x21; //timeutc
			break;
			case(5):
				ubx_cfg_msg.msgID = 0x02; //POSLLH
			break;
			case(6):
				ubx_cfg_msg.msgID = 0x12; //velned
			break;
			}

			DEBUG("ubx_cfg_msg\n");
			ublox_send(0x06, 0x01, sizeof(ubx_cfg_msg), (uint8_t *)&ubx_cfg_msg);
			break;
		}
	}


}


void ublox_format_coords()
{
	uint32_t alat = abs(fc.gps_data.latitude);
	uint32_t alon = abs(fc.gps_data.longtitude);

	switch (config.connectivity.gps_format_flags & GPS_FORMAT_MASK)
	{
		case(GPS_DDdddddd):
			sprintf((char *)fc.gps_data.cache_gui_latitude, "+%0.6f", fc.gps_data.latitude / (float)GNSS_MUL);
			sprintf((char *)fc.gps_data.cache_gui_longtitude, "+%0.6f", fc.gps_data.longtitude / (float)GNSS_MUL);
		break;

		case(GPS_DDMMmmm):
			sprintf((char *)fc.gps_data.cache_gui_latitude, "%lu°%6.3f' %c", alat / GNSS_MUL, ((alat % GNSS_MUL) * 60) / (float)GNSS_MUL, fc.gps_data.latitude > 0 ? 'N' : 'S');
			sprintf((char *)fc.gps_data.cache_gui_longtitude, "%lu°%6.3f' %c", alon / GNSS_MUL, ((alon % GNSS_MUL) * 60) / (float)GNSS_MUL, fc.gps_data.longtitude > 0 ? 'E' : 'W');
		break;

		case(GPS_DDMMSS):
			sprintf((char *)fc.gps_data.cache_gui_latitude, "%lu°%02lu'%02.4f\" %c", alat / GNSS_MUL, ((alat % GNSS_MUL) * 60) / GNSS_MUL, ((alat % 100000) * 60) / 100000.0, fc.gps_data.latitude > 0 ? 'N' : 'S');
			sprintf((char *)fc.gps_data.cache_gui_longtitude, "%lu°%02lu'%02.4f\" %c", alon / GNSS_MUL, ((alon % GNSS_MUL) * 60) / GNSS_MUL, ((alon % 100000) * 60) / 100000.0, fc.gps_data.longtitude > 0 ? 'E' : 'W');
		break;
	}

	uint32_t mlat = ((alat % GNSS_MUL) * 60);
	uint32_t mlon = ((alon % GNSS_MUL) * 60);
	uint32_t mlat1 = mlat / GNSS_MUL;
	uint32_t mlon1 = mlon / GNSS_MUL;
	uint32_t mlat2 = (mlat % GNSS_MUL) / 10000;
	uint32_t mlon2 = (mlon % GNSS_MUL) / 10000;

	snprintf((char *)fc.gps_data.cache_igc_latitude, sizeof(fc.gps_data.cache_igc_latitude), "%02lu%02lu%03lu%c", alat / GNSS_MUL, mlat1, mlat2, fc.gps_data.latitude > 0 ? 'N' : 'S');
	snprintf((char *)fc.gps_data.cache_igc_longtitude, sizeof(fc.gps_data.cache_igc_longtitude), "%03lu%02lu%03lu%c", alon / GNSS_MUL, mlon1, mlon2, fc.gps_data.longtitude  > 0 ? 'E' : 'W');
}



bool ublox_handle_nav(uint8_t msg_id, uint8_t * msg_payload, uint16_t msg_len)
{
	DEBUG("Handle NAV\n");

	if (msg_id == 0x02)
	{
		assert(msg_len == 28);

		typedef struct {
			uint32_t iTOW;
			int32_t lon;
			int32_t lat;
			int32_t height; //above elipsoid [mm]
			int32_t hMSL;  //above mean sea level [mm]
			uint32_t hAcc; //horizontal accuracy estimate [mm]
			uint32_t vAcc; //vertical accuracy estimate [mm]
		} ubx_nav_posllh_t;

		ubx_nav_posllh_t * ubx_nav_posllh = (ubx_nav_posllh_t *)msg_payload;

		//fc.gnss.longtitude = ubx_nav_posllh->lon;
		fc.gps_data.longtitude = ubx_nav_posllh->lon;

		//fc.gnss.latitude = ubx_nav_posllh->lat;
		fc.gps_data.latitude = ubx_nav_posllh->lat;

		float altitude_above_ellipsiod = ubx_nav_posllh->height / 1000.0;
		float altitude_above_msl= ubx_nav_posllh->hMSL / 1000.0;
		fc.gps_data.separation = altitude_above_ellipsiod - altitude_above_msl;
		fc.gps_data.altitude = ubx_nav_posllh->hMSL / 1000.0;

		//DEBUG("hacc %lX vacc %lX\n", ubx_nav_posllh->hAcc, ubx_nav_posllh->vAcc);
		fc.gps_data.hacc = (float)ubx_nav_posllh->hAcc / 100.0;
		fc.gps_data.vacc = (float)ubx_nav_posllh->vAcc / 100.0;

		fc.gps_data.new_sample = 0xFF;

		if (fc.gps_data.fix > 0)
		{
			ublox_format_coords();
		}

		return true;
	}

	if (msg_id == 0x12)
	{
		assert(msg_len == 36);

		typedef struct {
			uint32_t iTOW;
			int32_t velN;
			int32_t velE;
			int32_t velD;
			uint32_t speed; //3D speed [cm/s]
			uint32_t gSpeed; //ground speed [cm/s]
			int32_t heading; //degrees //1*10^-5
			uint32_t sAcc;
			uint32_t cAcc;
		} ubx_nav_velned_t;

		ubx_nav_velned_t * ubx_nav_velned = (ubx_nav_velned_t *)msg_payload;

		fc.gps_data.ground_speed = FC_MPS_TO_KNOTS * ubx_nav_velned->gSpeed / 100.0;
		fc.gps_data.heading = ubx_nav_velned->heading / 100000;

		return true;
	}

	if (msg_id == 0x03)
	{
		assert(msg_len == 16);

		typedef struct {
			uint32_t iTOW;
			uint8_t gpsFix;
			uint8_t flags;
			uint8_t fixStat;
			uint8_t flags2;
			uint32_t ttff;
			uint32_t msss;
		} ubx_nav_status_t;

		ubx_nav_status_t * ubx_nav_status = (ubx_nav_status_t *)msg_payload;

//        ublox_handle_itow(ubx_nav_status->iTOW);

		switch (ubx_nav_status->gpsFix)
		{
			case(2):
				fc.gps_data.fix = 2;
				fc.gps_data.ttf = task_get_ms_tick() - ublox_start_time;

			break;
			case(3):
				fc.gps_data.fix = 3;
				fc.gps_data.ttf = ubx_nav_status->ttff;

			break;
			default:
				fc.gps_data.fix = 0;
				fc.gps_data.ttf = task_get_ms_tick() - ublox_start_time;
			break;
		}

		fc.gps_data.valid = fc.gps_data.fix > 0;

		if (fc.gps_data.valid)
		{
			if (fc.gps_data.fix_cnt < GPS_FIX_CNT_MAX)
				fc.gps_data.fix_cnt++;
		}
		else
		{
			fc.gps_data.fix_cnt = 0;
		}

		return true;
	}

	if (msg_id == 0x21)//UBX-NAV-TIMEUTC
	{
		assert(msg_len == 20);

		typedef struct {
			uint32_t iTOW;
			uint32_t tAcc;
			int32_t nano;
			uint16_t year;
			uint8_t month;
			uint8_t day;
			uint8_t hour;
			uint8_t min;
			uint8_t sec;
			uint8_t valid;
		} ubx_nav_timeutc_t;

		ubx_nav_timeutc_t * ubx_nav_timeutc = (ubx_nav_timeutc_t *)msg_payload;

//        ublox_handle_itow(ubx_nav_timeutc->iTOW);

		if ((ubx_nav_timeutc->valid & 0b00000111) == 0b00000111)
		{
			fc.gps_data.utc_time = datetime_to_epoch(ubx_nav_timeutc->sec, ubx_nav_timeutc->min, ubx_nav_timeutc->hour,
					ubx_nav_timeutc->day, ubx_nav_timeutc->month, ubx_nav_timeutc->year);

			DEBUG("DATE %u.%u.%u\n", ubx_nav_timeutc->day, ubx_nav_timeutc->month, ubx_nav_timeutc->year);
			DEBUG("TIME %02u:%02u.%02u\n", ubx_nav_timeutc->hour, ubx_nav_timeutc->min, ubx_nav_timeutc->sec);
			DEBUG("UTC %lu\n", fc.gps_data.utc_time);
		}

		return true;
	}

	if (msg_id == 0x35)
	{
		assert((msg_len - 8) % 12 == 0);

		typedef struct {
			uint32_t iTOW;
			uint8_t version;
			uint8_t numSvs;
			uint8_t reserved1[2];
		} ubx_nav_sat_t;

		ubx_nav_sat_t * ubx_nav_sat = (ubx_nav_sat_t *)msg_payload;

		DEBUG("iTOW %lu\n", ubx_nav_sat->iTOW);
		DEBUG("numSvs %u\n", ubx_nav_sat->numSvs);

		assert(ubx_nav_sat->version == 0x01);

		typedef struct {
			uint8_t gnssId;
			uint8_t svId;
			uint8_t cno;
			int8_t elev;
			int16_t azim;
			int16_t prRes;
			uint32_t flags;
		} ubx_nav_sat_info_t;


		uint8_t used = 0;
		for (uint8_t i = 0; i < ubx_nav_sat->numSvs; i++)
		{
			ubx_nav_sat_info_t * ubx_nav_sat_info = (ubx_nav_sat_info_t *)(msg_payload + 8 + 12 * i);
			fc.gps_data.sat_id[i] = ubx_nav_sat_info->svId;
//			fc.gnss.sat_info.sats[i].flags = GNSS_SAT_SYSTEM_MASK & ubx_nav_sat_info->gnssId;
			fc.gps_data.sat_snr[i] = ubx_nav_sat_info->cno;
//			fc.gnss.sat_info.sats[i].elevation = ubx_nav_sat_info->elev;
//			fc.gnss.sat_info.sats[i].azimuth = ubx_nav_sat_info->azim / 2;
			if (ubx_nav_sat_info->flags & (1 << 3))
			{
				used++;
			}
		}

		for (uint8_t i = ubx_nav_sat->numSvs; i < GPS_SAT_CNT; i++)
		{
			fc.gps_data.sat_id[i] = 0;
		}

		fc.gps_data.sat_total = ubx_nav_sat->numSvs;
		fc.gps_data.sat_used = used;

		return true;
	}

	return false;
}

bool ublox_handle_ack(uint8_t msg_id, uint8_t * msg_payload, uint16_t msg_len)
{
	static uint8_t ubx_cfg_msg_cnt;

	DEBUG("Handle ACK\n");
	assert(msg_len == 2);
	assert(msg_id == 0x01);

	ublox_last_command_time = false;

	if (msg_payload[0] == 0x06 && msg_payload[1] == 0x00) //ubx_cfg_prt
	{
		ublox_command(1);
		return true;
	}

	if (msg_payload[0] == 0x06 && msg_payload[1] == 0x08) //ubx_cfg_rate
	{
		ublox_command(2);
		ubx_cfg_msg_cnt = 0;
		return true;
	}

	if (msg_payload[0] == 0x06 && msg_payload[1] == 0x01) //ubx_cfg_msg
	{
		if (ubx_cfg_msg_cnt < 4)
		{
			ublox_command(3 + ubx_cfg_msg_cnt);
		}
		else
		{
			ublox_init_ok = true;
		}
		ubx_cfg_msg_cnt++;
		return true;
	}

	return false;
}

static char ublox_start_word[] = "$GNRMC";


void ublox_parse(uint8_t b)
{
	//DEBUG(">> [%02X %c]\n", b, b);

	static uint8_t msg_payload[1024];

    static uint16_t index = 0;
    static uint16_t msg_len;

	static uint8_t msg_class;
	static uint8_t msg_id;
	static uint8_t msg_ck_a;
	static uint8_t msg_ck_b;

	if (ublox_parse_mode > PM_SYNC && ublox_parse_mode < PM_CK_A)
	{
		msg_ck_a += b;
		msg_ck_b += msg_ck_a;
	}

	switch (ublox_parse_mode)
	{
		case(PM_RESET):
			index = 0;
			ublox_parse_mode = PM_INIT;
		break;

		case(PM_INIT):
			{

				if (b == ublox_start_word[index])
				{
					index++;
					if (index == strlen(ublox_start_word))
					{
						ublox_command(0);
						ublox_parse_mode = PM_IDLE;
					}
				}
				else
				{
					index = 0;
				}

				break;
			}

		case(PM_IDLE):
			if (b == 0xB5)
			{
				ublox_parse_mode = PM_SYNC;
			}
			break;

		case(PM_SYNC):
			if (b == 0x62)
			{
				ublox_parse_mode = PM_CLASS;
				msg_ck_a = 0;
				msg_ck_b = 0;
			}
			else
			{
				ublox_parse_mode = PM_IDLE;
			}
			break;

		case(PM_CLASS):
			msg_class = b;
			ublox_parse_mode = PM_ID;
			break;

		case(PM_ID):
			msg_id = b;
			ublox_parse_mode = PM_LEN_LO;
			break;

		case(PM_LEN_LO):
			msg_len = b;
			ublox_parse_mode = PM_LEN_HI;
			break;

		case(PM_LEN_HI):
			msg_len |= (b << 8);
			index = 0;
			if (msg_len > 0)
				ublox_parse_mode = PM_PAYLOAD;
			else
				ublox_parse_mode = PM_CK_A;

			if (msg_len > sizeof(msg_payload))
			{
				DEBUG("payload too long\n");
				ublox_parse_mode = PM_IDLE;
			}

			break;
			
		case(PM_PAYLOAD):
			msg_payload[index] = b;
			index++;
			if (index == msg_len)
				ublox_parse_mode = PM_CK_A;
			break;
			
		case(PM_CK_A):
			if (msg_ck_a == b)
			{
				ublox_parse_mode = PM_CK_B;
			}
			else
			{
				DEBUG("Checksum A failed\n");
				ublox_parse_mode = PM_IDLE;
			}
			break;
			
		case(PM_CK_B):
			if (msg_ck_b == b)
			{
				//execute
				bool parsed;

				switch (msg_class)
				{
					case(0x05): //UBX-ACK
						parsed = ublox_handle_ack(msg_id, msg_payload, msg_len);
					break;

					case(0x01): //UBX-NAV
						parsed = ublox_handle_nav(msg_id, msg_payload, msg_len);
					break;

					default:
						parsed = false;
				}

				if (!parsed)
				{
					DEBUG("Message not parsed: \n");
					DEBUG("UBX <<< %02X, %02X len %u\n", msg_class, msg_id, msg_len);
					for (uint8_t i = 0; i < msg_len; i++)
						DEBUG("%02X ", msg_payload[i]);
					DEBUG("\n");
				}
			}
			else
			{
				DEBUG("Checksum B failed\n");
			}
			ublox_parse_mode = PM_IDLE;

			break;
	}
}

void ublox_step()
{
	while (!gps_uart.isRxBufferEmpty())
		ublox_parse(gps_uart.Read());
}
