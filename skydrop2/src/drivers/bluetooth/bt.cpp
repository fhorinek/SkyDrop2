

#include "bt.h"
#include "debug_off.h"

#include "../../fc/conf.h"
#include <stdint.h>

#define BT_OUTPUT_SIZE	512
uint8_t bt_output_buffer[BT_OUTPUT_SIZE];
RingBuffer bt_output(BT_OUTPUT_SIZE, bt_output_buffer);

#define BT_UART_RX_SIZE	128
#define BT_UART_TX_SIZE	64
uint8_t bt_uart_rx_buffer[BT_UART_RX_SIZE];
uint8_t bt_uart_tx_buffer[BT_UART_TX_SIZE];
Usart bt_uart(BT_UART_RX_SIZE, bt_uart_rx_buffer, BT_UART_TX_SIZE, bt_uart_tx_buffer);

volatile bool bt_device_connected = false;

volatile uint8_t bt_module_state = BT_MOD_STATE_OFF;
uint32_t bt_start_timer = 0;
uint32_t bt_timer = 0;

bool bt_set_name = false;

void bt_module_deinit()
{
	GpioWrite(BT_EN, LOW);
	GpioWrite(BT_WKP, HIGH);
	GpioWrite(BT_PDN, LOW);

	bt_irqh(BT_IRQ_DEINIT, 0);
	bt_uart.Stop();
	BT_UART_PWR_OFF;
}

//void bt_module_reset()
//{
//	GpioWrite(BT_EN, LOW);
//
//	bt_uart.Stop();
//	BT_UART_PWR_OFF;
//
//	//clear output buffer
//	bt_output.Clear();
//}


#define BSEL_TUNE_STEP 	10
#define BSEL_TUNE_RANGE	400

static bool bsel_tune_active = false;
int16_t bt_bsel_tune = -BSEL_TUNE_RANGE;
static int16_t bsel_tune_min = BSEL_TUNE_RANGE;
static int16_t bsel_tune_max = -BSEL_TUNE_RANGE;


void bt_module_init()
{
	DEBUG("bt_bsel_tune %d\n", bt_bsel_tune);

	//init uart
	BT_UART_PWR_ON;
	bt_uart.Init(BT_UART, 115200, bt_bsel_tune);
	bt_uart.SetInterruptPriority(MEDIUM);
	bt_uart.SetupRxDMA(BT_UART_DMA_CH, BT_UART_DMA_TRIGGER);

	GpioWrite(BT_WKP, LOW);

	//power is on
	GpioWrite(BT_EN, HIGH);
	bt_start_timer = task_get_ms_tick() + 100;
	bt_irqh(BT_IRQ_INIT, 0);

	bt_set_name = true;
}

void bt_init()
{
	DEBUG("bt_init\n");

	eeprom_busy_wait();
	ee_read_word(&config_ro.bt_bsel_tune, bt_bsel_tune);

	//pin init
	GpioSetDirection(BT_EN, OUTPUT);
	GpioSetDirection(BT_WKP, OUTPUT);
	GpioSetDirection(BT_PDN, OUTPUT);

	//power is off
	GpioWrite(BT_EN, LOW);
	GpioWrite(BT_WKP, LOW);
	GpioWrite(BT_PDN, LOW);
}

void bt_stop()
{
	bt_module_deinit();
}

bool bt_device_active()
{
	return bt_device_connected;
}

static void bsel_tune_next(bool failed = false)
{
	bt_bsel_tune += BSEL_TUNE_STEP;
	if (bt_bsel_tune > BSEL_TUNE_RANGE || (failed && (bsel_tune_max != -BSEL_TUNE_RANGE && bt_bsel_tune > bsel_tune_max)))
	{
		bt_bsel_tune = (bsel_tune_max + bsel_tune_min) / 2;
		DEBUG("BSEL tune = %d (%d, %d)\n", bt_bsel_tune, bsel_tune_min, bsel_tune_max);

		eeprom_busy_wait();
		ee_update_word(&config_ro.bt_bsel_tune, bt_bsel_tune);
		bsel_tune_active = false;
	}
	bt_irqh(BT_IRQ_REINIT, NULL);
}


void bt_parse(char c)
{
	static uint8_t parse_state = 0;
	char prefix[] = "TTM:";

	DEBUG(">> %02X %c %u\n", c, c, parse_state);

	if (parse_state < 4)
	{
		if (c == prefix[parse_state])
		{
			parse_state++;
		}
		else
		{
			parse_state = 0;
		}
	}
	else
	{
		if (c == 'O' && bt_module_state == BT_MOD_STATE_INIT)
		{
			bsel_tune_max = max(bsel_tune_max, bt_bsel_tune);
			bsel_tune_min = min(bsel_tune_min, bt_bsel_tune);

			if (bsel_tune_active)
			{
				bsel_tune_next();
			}
			else
			{
				//start advertising and go to sleep
				GpioWrite(BT_PDN, LOW);
				bt_irqh(BT_IRQ_INIT_OK, 0);
				GpioWrite(BT_WKP, HIGH);
			}
		}

		if (c == 'C')
		{
			bt_irqh(BT_IRQ_CONNECTED, NULL);

			uint8_t data[] = "TTM:CIT-20ms\r\n";
			bt_send(sizeof(data), data);

			//wake up
			bt_timer = task_get_ms_tick() + 1;
			GpioWrite(BT_WKP, LOW);
		}

		if (c == 'D')
		{
			bt_irqh(BT_IRQ_DISCONNECTED, NULL);

			//sleep
			GpioWrite(BT_WKP, HIGH);
		}

		parse_state = 0;
	}
}

extern int16_t xtal_calib_stable;

void bt_step()
{
	if (bt_module_state == BT_MOD_STATE_OFF)
		return;

	if (bt_module_state == BT_MOD_STATE_INIT)
	{
		if (bt_start_timer < task_get_ms_tick())
		{
			if (bt_set_name)
			{
				if (fc_xtal_calib_stable())
				{
					DEBUG("bt_set_name\n");

					bt_set_name = false;
					uint8_t data[] = "TTM:REN-SkyDrop2\r\n";

					bt_send(sizeof(data), data, true);
					bt_start_timer = task_get_ms_tick() + 100;
				}
				else
				{
					bt_start_timer = task_get_ms_tick() + 250;
				}
			}
			else
			{
				//failed
				if (bsel_tune_active)
				{
					bsel_tune_next(true);
				}
				else
				{
					bt_bsel_tune = -BSEL_TUNE_RANGE;
					bsel_tune_active = true;
					bt_irqh(BT_IRQ_REINIT, NULL);
				}
			}
		}
	}


	while (!bt_uart.isRxBufferEmpty())
	{
		bt_parse(bt_uart.Read());
	}

	if (bt_timer >= task_get_ms_tick())
	{
		DEBUG("Too fast!\n");
		return;
	}

	uint16_t len = bt_output.Length();

//	len = min(len, 40);

	for (uint16_t i = 0; i < len; i++)
	{
		char c = bt_output.Read();
		bt_uart.Write(c);
//		DEBUG("<< %02X '%c'\n", c, c);
	}
}

bool bt_full_warning = true;

void bt_send(uint16_t len, uint8_t * data, bool cmd)
{
	if (!bt_device_connected && !cmd)
		return;

	if (len + bt_output.Length() > bt_output.size)
	{
		if (bt_full_warning)
		{
			DEBUG("bt output buffer full!\n");
			bt_full_warning = false;
		}
		return;
	}

	bt_full_warning = true;
	bt_output.Write(len, data);
//
//	DEBUG("\n--------\n");
//	DEBUG("WI %d\n", bt_output.write_index);
//	DEBUG("SI %d\n", bt_output.length);
//
//
//	for (uint16_t i = 0; i < bt_output.Length(); i++)
//	{
//		uint16_t index = (i + bt_output.read_index) % bt_output.size;
//
//		DEBUG("%02X ", bt_output.buffer[index]);
//	}
//	DEBUG("\n--------\n");
}

void bt_send(char * str)
{
	uint16_t len = strlen(str);
	bt_send(len, (uint8_t *)str);
}

void bt_irqh(uint8_t type, uint8_t * buf)
{
	DEBUG("BT IRQ %u\n", type);

	switch(type)
	{
		case(BT_IRQ_INIT):
		DEBUG("BT_MOD_STATE_INIT\n");
			bt_module_state = BT_MOD_STATE_INIT;
		break;
		case(BT_IRQ_INIT_OK):
			DEBUG("BT_MOD_STATE_OK\n");
			bt_module_state = BT_MOD_STATE_OK;
		break;
		case(BT_IRQ_DEINIT):
			DEBUG("BT_MOD_STATE_OFF\n");
			bt_module_state = BT_MOD_STATE_OFF;
			bt_device_connected = false;
		break;
		case(BT_IRQ_CONNECTED):
			DEBUG("BT_IRQ_CONNECTED\n");
			gui_showmessage_P(PSTR("Bluetooth\nconnected"));
			bt_device_connected = true;
		break;
		case(BT_IRQ_DISCONNECTED):
			DEBUG("BT_IRQ_DISCONNECTED\n");
			gui_showmessage_P(PSTR("Bluetooth\ndisconnected"));
			bt_device_connected = false;
			bt_output.Clear();
		break;
		case(BT_IRQ_RESET):
			DEBUG("BT_IRQ_RESET\n");
			bt_device_connected = false;
		break;
		case(BT_IRQ_REINIT):
			DEBUG("BT_IRQ_REINIT!\n");
			bt_module_deinit();
			bt_module_init();
		break;
	}
}

bool bt_ready()
{
	return (bt_module_state == BT_MOD_STATE_OK);
}

