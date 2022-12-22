#include "igc.h"
#include "logger.h"
#include "../../drivers/storage/storage.h"

/** 
 * Here we save the filepointer at the position right before the
 * <TimeStamp>...<end>". So we can rewind to set the end date, when
 * we close the log.
 */
uint32_t filepos_for_end;

/**
 * printf-like function to send output to the KML file and append a CRLF.
 *
 * \param format a printf-like format string that must reside in PROGMEM.
 *
 */
void kml_sprintf_P(const char *format, ...)
{
	va_list arp;
	char line[79+2];
	uint8_t l;
	uint16_t wl;

	va_start(arp, format);
	vsnprintf_P(line, sizeof(line) - 2, format, arp);
	va_end(arp);

	l = strlen(line);
	strcpy_P(line + l, PSTR("\r\n"));
	l += 2;

	assert(f_write(&log_file, line, l, &wl) == FR_OK);
	assert(wl == l);
	assert(f_sync(&log_file) == FR_OK);
}

/**
 * Insert a line into the log containing a XML element and "now" as the content, 
 * e.g. "<end>2016-12-24T18:00:00Z</end>".
 * 
 * \param tag the name of the XML element, in the above example, this would be "end".
 */
void kml_now(const char *tag) {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t wday;
	uint8_t month;
	uint16_t year;

	datetime_from_epoch(time_get_utc(), &sec, &min, &hour, &day, &wday, &month, &year);
	kml_sprintf_P(PSTR("<%s>%04d-%02d-%02dT%02d:%02d:%02dZ</%s>"), tag, year, month, day, hour, min, sec, tag);
}

uint8_t kml_start(char * path)
{
	char filename[128];

	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t wday;
	uint8_t month;
	uint16_t year;

	char id[32];

	datetime_from_epoch(time_get_utc(), &sec, &min, &hour, &day, &wday, &month, &year);

	sprintf_P(filename, PSTR("%sKML"), path);
	DEBUG("KML filename %s\n", filename);

	uint8_t res = f_open(&log_file, filename, FA_WRITE | FA_CREATE_ALWAYS);
	assert(res == FR_OK);

	//cannot create file
	if (res != FR_OK)
		return false;

	//header
	GetID_str(id);

	//body
	kml_sprintf_P(PSTR("<kml xmlns=\"http://www.opengis.net/kml/2.2\">"));
	kml_sprintf_P(PSTR("<Document>"));
	kml_sprintf_P(PSTR("<name>Flight %02d.%02d.%04d @ %02d:%02d</name>"), day, month, year, hour, min);
	kml_sprintf_P(PSTR("<Placemark id=\"%s-%ld\">"), id, time_get_utc());
	kml_sprintf_P(PSTR("<name>Flight</name>"));
	kml_sprintf_P(PSTR("<visibility>1</visibility>"));
	kml_sprintf_P(PSTR("<open>1</open>"));

	kml_sprintf_P(PSTR("<TimeSpan>"));
	kml_now("begin");
	// Save position of end date, so that we can overwrite on close:
	filepos_for_end = f_tell(&log_file);
	kml_now("end");
	kml_sprintf_P(PSTR("</TimeSpan>"));

	kml_sprintf_P(PSTR("<Style>"));
	kml_sprintf_P(PSTR("<LineStyle><color>ff00ffff</color></LineStyle>"));
	kml_sprintf_P(PSTR("<PolyStyle><color>7f0000ff</color></PolyStyle>"));
	kml_sprintf_P(PSTR("</Style>"));
	kml_sprintf_P(PSTR("<LineString>"));
	kml_sprintf_P(PSTR("<extrude>1</extrude>"));
	kml_sprintf_P(PSTR("<altitudeMode>absolute</altitudeMode>"));
	kml_sprintf_P(PSTR("<coordinates>"));

	return (fc.gps_data.valid) ? LOGGER_ACTIVE : LOGGER_WAIT_FOR_GPS;
}

void kml_step()
{
	if (fc.gps_data.valid)
	{
		char tmp1[16];
		char tmp2[16];

		fc.logger_state = LOGGER_ACTIVE;

		sprintf_P(tmp1, PSTR(" %+011ld"), fc.gps_data.longtitude);
		memcpy((void *)tmp1, (void *)(tmp1 + 1), 4);
		tmp1[4] = '.';

		sprintf_P(tmp2, PSTR(" %+010ld"), fc.gps_data.latitude);
		memcpy((void *)tmp2, (void *)(tmp2 + 1), 3);
		tmp2[3] = '.';

		kml_sprintf_P(PSTR("%s,%s,%0.0f"), tmp1, tmp2, fc.altitude1);
	}
	else
		fc.logger_state = LOGGER_WAIT_FOR_GPS;
}

void kml_comment(char * text)
{
	kml_sprintf_P(PSTR("<!-- %s -->"), text);
}


void kml_stop()
{
	kml_sprintf_P(PSTR("</coordinates>"));
	kml_sprintf_P(PSTR("</LineString>"));
	kml_sprintf_P(PSTR("</Placemark>"));
	kml_sprintf_P(PSTR("</Document>"));
	kml_sprintf_P(PSTR("</kml>"));

	// Overwrite previous entry of <end> with "now":
	f_lseek(&log_file, filepos_for_end);
	kml_now("end");

	assert(f_close(&log_file) == FR_OK);
}
