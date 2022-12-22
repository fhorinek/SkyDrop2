#include "common.h"
#include "drivers/uart.h"
#include "drivers/storage/storage.h"
#include <string.h>
#include "gui/gui.h"
#include "fc/conf.h"

#include "debug_on.h"

struct app_info ee_fw_info __attribute__ ((section(".fw_info")));
struct app_info fw_info;

void print_reset_reason()
{
	//Print reset reason
	DEBUG("Rst:");

	if (system_rst & 0b00100000)
		DEBUG("Soft");
	else
	if (system_rst & 0b00010000)
		DEBUG("Prog");
	else
	if (system_rst & 0b00001000)
		DEBUG("Wdt");
	else
	if (system_rst & 0b00000100)
		DEBUG("Brwn");
	else
	if (system_rst & 0b00000010)
		DEBUG("Ext");
	else
	if (system_rst & 0b00000001)
		DEBUG("Pwr");
	else
		DEBUG("%02X", system_rst);

	DEBUG("\n");
}


void print_fw_info()
{
	
	ee_read_block(&fw_info, &ee_fw_info, sizeof(fw_info));

	DEBUG("App:");
	for (uint8_t i = 0; i < APP_INFO_NAME_len; i++)
	{
		uint8_t c = fw_info.app_name[i];
		if (c < 32 || c > 126)
			c = '_';
		DEBUG("%c", c);
	}
	DEBUG("\n");
}

//----------------------------------------------------------

void turnoff_subsystems()
{
	//PORTA
	PR.PRPA = 0b00000111;
	//PORTB
	PR.PRPB = 0b00000111;
	//PORTC
	PR.PRPC = 0b01111111;
	//PORTD
	PR.PRPD = 0b01111111;
	//PORTE
	PR.PRPE = 0b01111111;
	//PORTF
	PR.PRPF = 0b01111111;
	//PRGEN - RTC must stay on
	PR.PRGEN = 0b01011011;
}

//----------------------------------------------------------

DataBuffer::DataBuffer(uint16_t size, uint8_t * buffer)
{
	this->size = size;
	this->length = 0;
	this->write_index = 0;
	this->read_index = 0;

	this->data = buffer;
	if (this->data == NULL)
	{
		this->size = 0;
	}
}

DataBuffer::~DataBuffer()
{

}

uint16_t DataBuffer::Read(uint16_t len, uint8_t * * data)
{
	(*data) = this->data + this->read_index;

	if (len > this->length)
		len = this->length;

	if (this->read_index + len > this->size)
		len = this->size - this->read_index;

	this->read_index = (this->read_index + len) % this->size;
	this->length -= len;

	return len;
}

bool DataBuffer::Write(uint16_t len, uint8_t * data)
{
	if (this->size - this->length < len)
		return false;

	if (this->write_index + len > this->size)
	{
		uint16_t first_len = this->size - this->write_index;
		memcpy(this->data + this->write_index, data, first_len);
		memcpy(this->data, data + first_len, len - first_len);
	}
	else
	{
		memcpy(this->data + this->write_index, data, len);
	}

	this->write_index = (this->write_index + len) % this->size;
	this->length += len;

	return true;
}

uint16_t DataBuffer::Length()
{
	return this->length;
}

void DataBuffer::Clear()
{
	this->length = 0;
	this->write_index = 0;
	this->read_index = 0;
}

//***************************************************

volatile uint8_t bat_en_mask = 0;

void bat_en_high(uint8_t mask)
{
	bat_en_mask |= mask;
	GpioWrite(BAT_EN, HIGH);
}

void bat_en_low(uint8_t mask)
{
	bat_en_mask &= ~mask;

	if (bat_en_mask == 0)
		GpioWrite(BAT_EN, LOW);
}

//***************************************************

uint8_t device_id[11];

void GetID() //11 b
{
	enum
	{
		LOTNUM0=8,  // Lot Number Byte 0, ASCII
		LOTNUM1,    // Lot Number Byte 1, ASCII
		LOTNUM2,    // Lot Number Byte 2, ASCII
		LOTNUM3,    // Lot Number Byte 3, ASCII
		LOTNUM4,    // Lot Number Byte 4, ASCII
		LOTNUM5,    // Lot Number Byte 5, ASCII
		WAFNUM =16, // Wafer Number
		COORDX0=18, // Wafer Coordinate X Byte 0
		COORDX1,    // Wafer Coordinate X Byte 1
		COORDY0,    // Wafer Coordinate Y Byte 0
		COORDY1,    // Wafer Coordinate Y Byte 1
	};

	NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
	device_id[0] = pgm_read_byte(LOTNUM0);
	device_id[1] = pgm_read_byte(LOTNUM1);
	device_id[2] = pgm_read_byte(LOTNUM2);
	device_id[3] = pgm_read_byte(LOTNUM3);
	device_id[4] = pgm_read_byte(LOTNUM4);
	device_id[5] = pgm_read_byte(LOTNUM5);
	device_id[6] = pgm_read_byte(WAFNUM);
	device_id[7] = pgm_read_byte(COORDX0);
	device_id[8] = pgm_read_byte(COORDX1);
	device_id[9] = pgm_read_byte(COORDY0);
	device_id[10] = pgm_read_byte(COORDY1);
	NVM.CMD = NVM_CMD_NO_OPERATION_gc;
}


void GetID_str(char * id) //23 b
{
	uint8_t * b = device_id;

	for (uint8_t i = 0; i < 11; i++)
		sprintf_P(id + (2 * i), PSTR("%02X"), b[i]);
}

//***************************************************


bool cmpn(char * s1, const char * s2, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++)
	{
		if (s1[i] != s2[i])
			return false;
	}
	return true;
}

bool cmpn_p(char * s1, const char * s2, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++)
	{
		if (s1[i] != pgm_read_byte(&s2[i]))
			return false;
	}
	return true;
}

int freeRam()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

bool LoadEEPROM()
{
	FILINFO fno;

	if (storage_file_exist_P(PSTR("DROP2.EE")))
	{
		DEBUG("EE update found.\n");

		f_stat("DROP2.EE", &fno);

		FIL ee_file;

		f_open(&ee_file, "DROP2.EE", FA_READ);
		uint16_t rd = 0;

		byte4 tmp;

		f_read(&ee_file, tmp.uint8, sizeof(tmp), &rd);
		if (tmp.uint32 != BUILD_NUMBER || fno.fsize != sizeof(cfg_t))
		{
			gui_showmessage_P(PSTR("DROP2.EE is not\ncompatibile!"));
			DEBUG("Rejecting update file %lu != %lu\n", tmp.uint32, BUILD_NUMBER);
			return false;
		}

		//rewind the file
		f_lseek(&ee_file, 0);

		for (uint16_t i = 0; i < fno.fsize; i += rd)
		{
			uint8_t buf[256];

			FRESULT res = f_read(&ee_file, buf, sizeof(buf), &rd);

			if (res != FR_OK || rd == 0)
				break;

			ee_update_block(buf, (uint8_t *)(APP_INFO_EE_offset + i), rd);
		}

		gui_showmessage_P(PSTR("Configuration\napplied."));
		return true;
	}
	return false;
}

bool StoreEEPROM()
{
	ewdt_reset();
//	DEBUG("Storing settings\n");

	if (!storage_ready())
	{
//		DEBUG("Error: Storage not available\n");
		return false;
	}

	FIL ee_file;

	if (f_open(&ee_file, "CFG2.EE", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
	{
//		DEBUG("Unable to create file\n");

		return false;
	}
	uint16_t wd = 0;

	uint32_t tmp = BUILD_NUMBER;
	ee_update_dword(&config_ee.build_number, tmp);

	uint16_t i = 0;
	do
	{
		uint8_t buf[256];
		uint16_t rwd;

		if (i + sizeof(buf) < sizeof(cfg_t))
			wd = sizeof(buf);
		else
			wd = sizeof(cfg_t) - i;


		
		ee_read_block(buf, (uint8_t *)(APP_INFO_EE_offset + i), wd);

		assert(f_write(&ee_file, buf, wd, &rwd) == FR_OK);

		i += wd;
	} while (i < sizeof(cfg_t));

	f_close(&ee_file);
//	DEBUG("File closed\n");

	return true;
}

uint8_t flip_table[] = {0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15};

uint8_t fast_flip(uint8_t in)
{
	uint8_t out = flip_table[0x0F & in] << 4;
	out |= flip_table[(0xF0 & in) >> 4];
	return out;
}

void mems_power_init()
{
	GpioSetDirection(MEMS_EN, OUTPUT);
}

void mems_power_on()
{
	GpioWrite(MEMS_EN, HIGH);
}

void mems_power_off()
{
	GpioWrite(MEMS_EN, LOW);
}

float mul_to_sec(float mul)
{
	if (mul == 0)
		return 1;
	else
		return 1.0 / (mul * 100.0);
}

float sec_to_mul(float sec)
{
	if (sec <= 0.01)
		return 1;
	else
		return 1.0 / (sec * 100.0);
}

uint8_t hex_to_num(uint8_t c)
{
	if (c >= 'A')
		return c - 'A' + 10;
	else
		return c - '0';
}

/**
 * Return 10^pow.
 *
 * @param pow the number of times, that 10 will be powered.
 *
 * @return 10^pow
 */
uint32_t pow_ten(uint8_t pow)
{
	uint32_t ret = 1;
	uint8_t i;

	for (i = 0; i < pow; i++)
		ret *= 10;

	return ret;
}

/**
 * Build an integer from "str" expected to have "n" digits.
 * Hitting ',' will stop scanning and return the current result.
 * Hitting '.' will skip this character and continue scanning,
 * this does not count as a digit for "n".
 *
 * If there are less than "n" digits found, then the result
 * will still be an integer with "n" digits. E.g. if "n" is 5
 * but the string is "12", then 12000 will be returned.
 *
 * This is usefull to read fixed decimal numbers fractions.
 *
 * @param str the string to scan through
 * @param n the number of digits the integer has
 *
 * @return an integer
 */
uint32_t atoi_n(char * str, uint8_t n)
{
	uint32_t tmp = 0;
    uint8_t i;

	for (i=0; i < n; i++)
	{
		if (str[i] == ',')
			return tmp;
		if (str[i] == '.')
		{
			n++;
			continue;
		}

		tmp += (uint32_t)(str[i] - '0') * pow_ten(n - i - 1);
	}

	return tmp;
}

/**
 * Build an integer from "str" until either ',' or '*' or '\0'.
 *
 * @param str the string to scan through
 *
 * @return an integer
 */
uint8_t atoi_c(char * str)
{
	uint8_t tmp = 0;
	uint8_t i = 0;

	while(str[i] != ',' && str[i] != '*' && str[i] != 0)
	{
		tmp *= 10;
		tmp += str[i] - '0';
		i++;
	}

	return tmp;
}

float atoi_f(char * str)
{
	float tmp = 0;
	uint8_t dot = 0;
	uint8_t i = 0;

	if (str[0] == '-')
		i++;

	while(ISDIGIT(str[i]) || str[i] == '.')
	{
		if (str[i] == '.')
		{
			dot = i;
			i++;
		}

		if (dot == 0)
		{
			tmp *= 10;
			tmp += str[i] - '0';
		}
		else
		{
			tmp += (str[i] - '0') / pow(10, i - dot);
		}

		i++;
	}

	if (str[0] == '-')
		tmp *= -1;

	return tmp;
}

/**
 * Returns a pointer to the first occurrence of the character c in the string s.
 *
 * @param s the string to look through
 * @param c the character to search
 *
 * @return pointer to found character or NULL if not found.
 */
char *index(char * s, char c)
{
	while ((*s) != c) {
		if (*s == 0) return NULL;
		s++;
	}

	return s;
}

/**
 * Returns a pointer to the last occurrence of the character c in the string s.
 *
 * @param s the string to look through
 * @param c the character to search
 *
 * @return pointer to found character or NULL if not found.
 */
char *rindex(char * s, char c)
{
	char *s2 = s + strlen(s);

	while ((*s2) != c) {
		s2--;
		if (s2 < s) return NULL;
	}

	return s2;
}

/**
 * Return a pointer to the filename part of a full pathname.
 * e.g. "/directory1/tilmann" will return "tilmann".
 *
 * @param a string with a '/' in it.
 *
 * @returns the filename part.
 */
char *basename(char *s) {
	char *s2;

	s2 = rindex(s, '/');
	if ( s2 != NULL ) {
		return s2 + 1;
	} else {
		return NULL;
	}
}


/**
 * Advance s until the first non-blank character. This skips over all blanks.
 *
 * @param s a pointer to a string
 *
 * @return the next character in this string, that is not a blank.
 */
char *skip_ws(char *s)
{
	while( ISBLANK(*s) ) s++;
	return s;
}

/**
 * Find the first character after the ','.
 *
 * @return a pointer to the first character after ','.
 */
char * find_comma(char * str)
{
	return index(str, ',') + 1;
}

uint8_t nmea_checksum(char *s)
{
	uint8_t c = 0;

    while(*s)
        c ^= *s++;

    return c;
}
