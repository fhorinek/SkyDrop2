#include "conf.h"
#include "../gui/widgets/layouts/layouts.h"

cfg_ro_t config_ro __attribute__ ((section(".cfg_ro")));

#define log_default_text	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

#define bt_link_key_blank	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define mac_invalid			{0, 0, 0, 0, 0, 0}

#define empty10				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define empty20				{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

//*** Default configuration placeholder ***
const cfg_t config_defaults PROGMEM = {0xAABBCCDD, '*', '*', '*', ' ', 'D', 'e', 'f', 'a', 'u', 'l', 't', ' ', 'c', 'o', 'n', 'f', 'i', 'g', 'u', 'r', 'a', 't', 'i', 'o', 'n', ' ', 'p', 'l', 'a', 'c', 'e', 'h', 'o', 'l', 'd', 'e', 'r', ' ', '*', '*', '*'};

volatile cfg_t config;

EEMEM cfg_t config_ee = {
	//build_number
	BUILD_NUMBER,
	//gui
	{
		//contrast
		6,
		//brightness
		20,
		//brightness_timeout
		10,
		//display_flags
		CFG_DISP_ANIM,
		//last_page
		0,
		//menu_audio_flags
		CFG_AUDIO_MENU_SPLASH | CFG_AUDIO_MENU_PAGES | CFG_AUDIO_MENU_BUTTONS | CFG_AUDIO_MENU_GPS,
		//menu_volume
		40,
		//vario_volume
		100,
		//vario_mute
		false,
		//alert_volume
		100,
		//number_of_pages
		5,
		//silent
		0b00000000,
		//hide_label
		0b00000000,
	    //page_acro_thold; //*10
		-50,
	    //page_circling_timeout; //in  s
		15,
		//page mode
		{
            //prepare
            0,
            //circling
            3,
            //normal
            1,
            //acro
            4,
            //landed
            0
		},
		//pages
		{
			//0
			{
				//type
				LAYOUT_222,
				{WIDGET_TIME, WIDGET_FTIME, WIDGET_ALT1, WIDGET_ODO_DISTANCE, WIDGET_CTRL_WLIFT, WIDGET_BATTERY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//1
			{
				//type
				LAYOUT_122,
				{WIDGET_VARIO_BAR, WIDGET_ALT1, WIDGET_VARIO, WIDGET_GROUND_SPD, WIDGET_GHEADING_POINTS, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//2
			{
				//type
				LAYOUT_233,
				{WIDGET_ALT1, WIDGET_GLIDE_RATIO, WIDGET_GHEADING_POINTS, WIDGET_WIND_DIR_POINTS, WIDGET_WAYPOINT_ARROW, WIDGET_GROUND_SPD, WIDGET_WIND_SPD, WIDGET_WAYPOINT_DISTANCE, WIDGET_EMPTY}
			},
			//3
			{
				//type
				LAYOUT_31,
				{WIDGET_AVG_VARIO, WIDGET_ALT1, WIDGET_THERMAL_GAIN, WIDGET_THERMAL_ASS, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//4
			{
				//type
				LAYOUT_21a,
				{WIDGET_ALT1, WIDGET_AGL_HEIGHT, WIDGET_ACC_TOT, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//5
			{
				//type
				LAYOUT_21a,
				{WIDGET_ALT1, WIDGET_ALT3, WIDGET_ACC_TOT, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//6
			{
				//type
				LAYOUT_22,
				{WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
			//7
			{
				//type
				LAYOUT_22,
				{WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY, WIDGET_EMPTY}
			},
		},

	},
	//vario
	{
		//digital_vario_dampening
		1.0 / 100.0 / 0.3, // << last 0.3 sec
		//avg_vario_dampening
		1.0 / 100.0 / 10.0, // << last 10 sec
		//flags
		VARIO_UNITS_M | VARIO_USE_ACC,
	},
	//altitude
	{
		//QNH1
		103000,
		//QNH2
		101325,
		//atl1_flags
		ALT_UNIT_M | ALT_AUTO_GPS,
		//altimeter
		{
			//altimeter2
			{
				//flags
				ALT_ABS_GPS,
				//diff
				0,
			},
			//altimeter3
			{
				//flags
				ALT_DIFF | ALT_AUTO_ZERO | 0,
				//diff
				0,
			},
			//altimeter4
			{
				//flags
				ALT_ABS_QNH2 | ALT_UNIT_I,
				//diff
				0,
			},
			//altimeter5
			{
				//flags
				ALT_ABS_QNH2,
				//diff
				0,
			},
		},
		//alarm_enabled
		false,
		// alarm_confirm_secs
		10,
		//alarm_1
		500,
		//alarm_2
		300,
		//alarm_h1
		2450,
		//alarm_offset
		50,
	},
	//audio_profile
	{
		//freq
		{200, 202, 204, 206, 210, 214, 220, 225, 230, 235, 242, 250, 263, 282, 305, 330, 358, 390, 424, 462, 500, 540, 600, 680, 800, 920, 1010, 1075, 1120, 1160, 1200, 1240, 1280, 1320, 1360, 1400, 1440, 1480, 1520, 1560, 1600},
		//pause
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 50, 200, 600, 1000, 1500, 2000, 2000, 1000, 600, 420, 320, 265, 230, 215, 200, 185, 172, 160, 150, 142, 135, 130, 125, 120, 115, 110, 105, 100, 95},
		//length
		{2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 1950, 1800, 1400, 1000, 600, 380, 220, 130, 100, 100, 100, 100, 150, 200, 225, 230, 215, 200, 185, 172, 160, 150, 142, 135, 130, 125, 120, 115, 110, 105, 100, 95},
		//lift
		10,
		//sink
		-246,
		//weak
		57,
		//flags
		AUDIO_FLUID | AUDIO_SINK_CONT,// | AUDIO_WEAK,
		//prebeep_offset in Hz
		150,
		//prebeep_length in ms
		150,
		//weak_low_freq
		100,
		//weak_high_freq
		400
	},
	//System
	{
		//time flags
		TIME_SYNC,
		//timezone
		+1 * 2,
		//debug_log
		false,//DEBUG_MAGIC_ON,
		//debug_gps
		false,
		//record_screen
		false,
		//auto_power_off
		30,
	},
	//Autostart
	{
		//start_sensititvity
		6,
		//land_sensitivity
		1,
		//gps_speed
		10,
		//timeout
		60,
		//flags
		AUTOSTART_SUPRESS_AUDIO,
	},
	//Logger
	{
		//enabled
		true,
		//format
		LOGGER_IGC,
		//pilot
		log_default_text,
		//glider_type
		log_default_text,
		//glider_id
		log_default_text
	},
	//Connectivity
	{
		//usb_mode
		USB_MODE_NONE,
		//use_gps
		true,
		//gps_format_flags
		GPS_DDdddddd | GPS_SPD_KPH,
		//use_bt
		false,
		//bt_link_partner
		mac_invalid,
		//bt_link_key
		bt_link_key_blank,
		//btle_mac
		mac_invalid,
		//forward_gps
		false,
		//protocol
		PROTOCOL_LK8EX1,
		//uart_function
		UART_FORWARD_DEBUG,
	},
	//Home
	{
		//flags
		HOME_TAKEOFF,
		//lat
		0,
		//lon
		0,
		//name
		empty20,
	},
	//Airspaces
	{
		//class_enabled
		0b1111111111111111,
		//warning_m
		500,
		//alert_on
		true,
		//alarm_confirm_secs
		30,
	},
	//Tasks
	{
		//task name
		empty10,
	}
};

void cfg_acc_write_defaults()
{
	vector_i16_t tmp;
	//bias
	tmp.x = 38;
	tmp.y = -66;
	tmp.z = -86;
	
	eeprom_write_block((void *)&tmp, (void *)&config_ro.calibration.acc_bias, sizeof(vector_i16_t));


	//sensitivity
	tmp.x = 4090;
	tmp.y = 4120;
	tmp.z = 4114;
	
	eeprom_write_block((void *)&tmp, (void *)&config_ro.calibration.acc_sensitivity, sizeof(vector_i16_t));
}

void cfg_gyro_write_defaults()
{
	vector_i16_t tmp;

	//bias
	tmp.x = 79;
	tmp.y = -53;
	tmp.z = -8;
	
	eeprom_write_block((void *)&tmp, (void *)&config_ro.gyro_bias, sizeof(vector_i16_t));
}

void cfg_baro_write_defaults()
{
	int16_t tmp = 0;
	
	ee_update_block(&tmp, &config_ro.baro_offset, sizeof(config_ro.baro_offset));
}

void cfg_bat_write_defaults()
{
	uint16_t tmp = 0xffff;
	
	eeprom_write_word(&config_ro.bat_runtime_minutes, tmp);
}


void cfg_check_floats()
{
	if (isnan(config.altitude.QNH1))
	{
	    config.altitude.QNH1 = 103000;
	    
	    ee_update_float(&config_ee.altitude.QNH1, config.altitude.QNH1);
	}

    if (isnan(config.altitude.QNH2))
    {
    	config.altitude.QNH2 = 101325;
        
        ee_update_float(&config_ee.altitude.QNH2, config.altitude.QNH2);
    }

    if (isnan(config.vario.digital_vario_dampening))
    {
    	config.vario.digital_vario_dampening = 1.0 / 100.0 / 0.3;
        
        ee_update_float(&config_ee.vario.digital_vario_dampening, config.vario.digital_vario_dampening);
    }

    if (isnan(config.vario.avg_vario_dampening))
    {
    	config.vario.avg_vario_dampening = 1.0 / 100.0 / 10.3;
        
        ee_update_float(&config_ee.vario.avg_vario_dampening, config.vario.avg_vario_dampening);
    }
}

void cfg_load()
{
	//check and reload default calibration data (if needed)
	uint8_t calib_flags;
	ee_read_byte(&config_ro.calibration_flags, calib_flags);

	if (calib_flags != CALIB_DEFAULT_LOADED)
	{
		cfg_acc_write_defaults();
		cfg_gyro_write_defaults();
		cfg_baro_write_defaults();
		cfg_bat_write_defaults();

		uint8_t tmp = CALIB_DEFAULT_LOADED;
		ee_update_byte(&config_ro.calibration_flags, tmp);
	}


	
	ee_read_block((void *)&config, &config_ee, sizeof(cfg_t));

	//prevent freezing if QNH or int. interval is corrupted
	cfg_check_floats();
}

void cfg_restore_defaults()
{
	#define EE_PAGE_SIZE            255

	for (uint16_t i = 0; i < sizeof(cfg_t); i += EE_PAGE_SIZE)
	{
		uint8_t buff[EE_PAGE_SIZE];
		uint8_t size = min(sizeof(cfg_t) - i, EE_PAGE_SIZE);

		void * progmem_adr = (void *) ((uint16_t) (&config_defaults) + i);
		void * eeprom_adr = (void *) ((uint16_t) (&config_ee) + i);

		memcpy_P(buff, progmem_adr, size);
		
		ee_update_block(buff, eeprom_adr, size);

		ewdt_reset();
	}

	

	cfg_load();
	gui_load_eeprom();
}

