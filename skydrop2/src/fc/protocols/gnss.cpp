/*
 * gnss.c
 *
 *  Created on: Oct 7, 2021
 *      Author: horinek
 */



#include "gnss.h"

#include "fc/fc.h"


bool gnss_rmc_msg(char * buff, uint16_t len)
{
	if (fc.gps_data.fix <= 1)
		return false;

	uint8_t sec, min, hour, day, month, wday;
	uint16_t year;

	datetime_from_epoch(fc.gps_data.utc_time, &sec, &min, &hour, &day, &wday, &month, &year);
	year = min(year - 2000, 99);

	char tmp[128];

	char slat[16], clat, slon[16], clon;

    uint32_t alat = abs(fc.gps_data.latitude);
    uint32_t alon = abs(fc.gps_data.longtitude);
    uint32_t mlat = ((alat % GNSS_MUL) * 60);
    uint32_t mlon = ((alon % GNSS_MUL) * 60);
    uint32_t mlat1 = mlat / GNSS_MUL;
    uint32_t mlon1 = mlon / GNSS_MUL;
    uint32_t mlat2 = (mlat % GNSS_MUL) / 1000;
    uint32_t mlon2 = (mlon % GNSS_MUL) / 1000;

    snprintf(slat, sizeof(slat), "%02lu%02lu.%04lu", alat / GNSS_MUL, mlat1, mlat2);
    snprintf(slon, sizeof(slon), "%03lu%02lu.%04lu", alon / GNSS_MUL, mlon1, mlon2);
    clat = fc.gps_data.latitude > 0 ? 'N' : 'S';
    clon = fc.gps_data.longtitude > 0 ? 'E' : 'W';

	snprintf(tmp, sizeof(tmp), "GPRMC,%02u%02u%02u,A,%s,%c,%s,%c,%0.1f,%u,%02u%02u%02u,0.0,E,A",
			hour, min, sec,
			slat, clat, slon, clon,
			fc.gps_data.ground_speed, fc.gps_data.heading,
			day, month, year);


	snprintf(buff, len, "$%s*%02X\r\n", tmp, nmea_checksum(tmp));

	return true;
}

bool gnss_gga_msg(char * buff, uint16_t len)
{
	if (fc.gps_data.fix <= 1)
		return false;

	uint8_t sec, min, hour;

	time_from_epoch(fc.gps_data.utc_time, &sec, &min, &hour);

	char tmp[128];

	char slat[16], clat, slon[16], clon;

    uint32_t alat = abs(fc.gps_data.latitude);
    uint32_t alon = abs(fc.gps_data.longtitude);
    uint32_t mlat = ((alat % GNSS_MUL) * 60);
    uint32_t mlon = ((alon % GNSS_MUL) * 60);
    uint32_t mlat1 = mlat / GNSS_MUL;
    uint32_t mlon1 = mlon / GNSS_MUL;
    uint32_t mlat2 = (mlat % GNSS_MUL) / 1000;
    uint32_t mlon2 = (mlon % GNSS_MUL) / 1000;

    snprintf(slat, sizeof(slat), "%02lu%02lu.%04lu", alat / GNSS_MUL, mlat1, mlat2);
    snprintf(slon, sizeof(slon), "%03lu%02lu.%04lu", alon / GNSS_MUL, mlon1, mlon2);
	clat = fc.gps_data.latitude > 0 ? 'N' : 'S';
	clon = fc.gps_data.longtitude > 0 ? 'E' : 'W';

	snprintf(tmp, sizeof(tmp), "GPGGA,%02u%02u%02u,%s,%c,%s,%c,1,%u,1.5,%0.1f,M,%0.1f,M,,,",
			hour, min, sec,
			slat, clat, slon, clon,
			fc.gps_data.sat_used,
			fc.gps_data.altitude,
			fc.gps_data.separation);

	snprintf(buff, len, "$%s*%02X\r\n", tmp, nmea_checksum(tmp));

	return true;
}
