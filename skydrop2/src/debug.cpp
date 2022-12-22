#include "debug.h"

#include <xlib/core/watchdog.h>

#include "drivers/storage/storage.h"
#include "fc/conf.h"

#include "debug_on.h"

//also used for FW update
FIL debug_file;

#ifndef DISABLE_DEBUG

volatile uint8_t debug_file_open;
Timer debug_timer;
uint32_t debug_last_pc;

uint8_t debug_log_buffer[DEBUG_LOG_BUFFER_SIZE];

DataBuffer debug_log_storage(DEBUG_LOG_BUFFER_SIZE, debug_log_buffer);

volatile uint16_t debug_return_pointer;
volatile uint16_t debug_min_stack_pointer = 0xFFFF;
volatile uint16_t debug_max_heap_pointer = 0x0000;
bool debug_done = false;


bool debug_disabled()
{
	if (config.system.debug_log == DEBUG_MAGIC_ON && storage_ready())
		return false;
	if (config.connectivity.uart_function == UART_FORWARD_DEBUG)
		return false;
	return true;
}

void debug_uart_send(char * msg)
{
	if (config.connectivity.uart_function == UART_FORWARD_DEBUG)
	{
		uart_send(msg);
		uart.FlushTxBuffer();
	}
}

void debug_print_ram()
{

	DEBUG("\noffset ");
	for (uint8_t i = 0; i < 16; i++)
		DEBUG(" +%01X", i);

	for (uint16_t i=0x2000; i<0x5FFF; i++)
	{
		ewdt_reset();
		uint8_t * tmp = (uint8_t *)(i);
		if (i % 0x10 == 0)
			DEBUG("\n0x%04X: ", i);
		DEBUG("%02X ", *tmp);

		if (i == 7)
			DEBUG(" ");
	}

}

void debug_last_dump()
{
	DEBUG("\nWDT info\n");

	
	ee_read_dword(&config_ro.debug.program_counter, debug_last_pc);

	uint32_t tmpl;
	uint16_t tmp;
	//time of the event
	ee_read_dword(&config_ro.debug.time, tmpl);
	DEBUG(" time:%lu\n", tmpl);
	//build number
	ee_read_word(&config_ro.debug.build_number, tmp);
	DEBUG(" build:%d\n", tmp);
	//program counter byte address (same as in lss)
	DEBUG(" PC:%06lX\n", debug_last_pc);
	//smallest stack address
	ee_read_word(config_ro.debug.min_stack, tmp);
	DEBUG(" stack::%04X\n", tmp);
	//biggest heap address
	ee_read_word(&config_ro.debug.max_heap, tmp);
	DEBUG(" heap:%04X\n\n", tmp);
}

void debug_timeout_handler()
{
	uint8_t r0 = *((uint8_t *)(debug_return_pointer + 0));
	uint8_t r1 = *((uint8_t *)(debug_return_pointer + 1));
	uint8_t r2 = *((uint8_t *)(debug_return_pointer + 2));

	//push decrement PC
	uint32_t ra = (((uint32_t)r0 << 16) & 0x00FF0000) | (((uint16_t)r1 <<  8) & 0xFF00) | (r2);
	//word to byte
	ra = ra * 2;

	//store this info
	uint32_t tmpl = time_get_local();
	uint16_t tmp = BUILD_NUMBER;
	ee_update_dword(&config_ro.debug.time, tmpl);
	ee_update_word(&config_ro.debug.build_number, tmp);
	ee_update_dword(&config_ro.debug.program_counter, ra);
	ee_update_word(&config_ro.debug.min_stack, debug_min_stack_pointer);
	ee_update_word(&config_ro.debug.max_heap, debug_max_heap_pointer);

	debug_last_dump();
}

/**
 * printf-like function to send output to the debug log or UART if debug is enabled.
 *
 * \param format a printf-like format string that must reside in PROGMEM.
 *
 */
void debug(const char *format, ...)
{
	va_list arp;
	char msg_buff[256];

	if (debug_disabled())
		return;

	va_start(arp, format);
	vsnprintf_P(msg_buff, sizeof(msg_buff), format, arp);
	va_end(arp);

	debug_uart_send(msg_buff);
	debug_log(msg_buff);

}

ISR(DEBUG_TIMER_OVF, ISR_NAKED)
{
	//saving all call clobbered
	asm volatile(
		"push r1" "\n\t"			//save "zero" register
		"push r0" "\n\t"			//save tmp register

		"in	r0, __SREG__" "\n\t" 	//save SREG to R0
		"cli" "\n\t"				//disable interrupts (ASAP, so SP will not be changed, hopefully)
		"push r0" "\n\t" 			//store SREG to STACK

		"eor r1, r1" "\n\t" 		//zero "zero" register

		"in	r0, 0x3B" "\n\t" 		//save RAMPZ to R0
		"push r0" "\n\t" 			//store RAMPZ to STACK

		"push r18" "\n\t" 			//store call clobbered
		"push r19" "\n\t"
		"push r20" "\n\t"
		"push r21" "\n\t"
		"push r22" "\n\t"
		"push r23" "\n\t"
		"push r24" "\n\t"
		"push r25" "\n\t"
		"push r26" "\n\t"
		"push r27" "\n\t"
		"push r30" "\n\t"
		"push r31" "\n\t"
		::);

	debug_return_pointer = SP + 17; // 16x push (IRQ prologue) + 1 (SP is pointing to next available location)

	sei(); //enable interrupts since handler is using uart and spi

	debug_timeout_handler();

	asm volatile(
		"pop r31" "\n\t"			//pop call clobbered
		"pop r30" "\n\t"
		"pop r27" "\n\t"
		"pop r26" "\n\t"
		"pop r25" "\n\t"
		"pop r24" "\n\t"
		"pop r23" "\n\t"
		"pop r22" "\n\t"
		"pop r21" "\n\t"
		"pop r20" "\n\t"
		"pop r19" "\n\t"
		"pop r18" "\n\t"

		"pop r0" "\n\t"				//RAMPZ STACK->R0
		"out 0x3B, r0" "\n\t"		//restore RAMPZ

		"pop r0" "\n\t"				//RAMPZ SREG->R0
		"out __SREG__, r0" "\n\t"	//restore SREG

		"pop r0" "\n\t"				//restore tmp
		"pop r1" "\n\t"				//restore zero
		"reti" "\n\t"
	::);

}

void debug_timer_init()
{
	DEBUG_TIMER_PWR_ON;
	debug_timer.Init(DEBUG_TIMER, timer_div1024);
	debug_timer.SetTop(0xEE09); //1.95s
	debug_timer.EnableInterrupts(timer_overflow);
	debug_timer.SetInterruptPriority(MEDIUM);
	debug_timer.Start();
}

void debug_log(char * msg)
{
	if (config.system.debug_log == DEBUG_MAGIC_ON && storage_ready())
		debug_log_storage.Write(strlen(msg), (uint8_t *)msg);
}

void debug_flush()
{
	debug_done = true;
	while(debug_log_storage.Length())
		debug_step();
	debug_done = false;
}


void debug_end()
{

	DEBUG("=== END ===\n");
	debug_flush();
}

void debug_step()
{

	if (config.system.debug_log != DEBUG_MAGIC_ON)
		return;

	uint8_t res;
	uint16_t wt;
	uint16_t len;

	if (!storage_ready())
		return;

	if (debug_log_storage.Length() < DEBUG_LOG_BUFFER_CHUNK && !debug_done)
		return;

	//Append file if not opened
	if (!debug_file_open)
	{
		//open or create new
		res = f_open(&debug_file, DEBUG_FILE, FA_WRITE | FA_OPEN_ALWAYS);
		if (res != FR_OK)
			return;

		//seek to end
		res = f_lseek(&debug_file, f_size(&debug_file));
		if (res != FR_OK)
		{
			f_close(&debug_file);
			return;
		}

		//Timestamp
		uint8_t sec, min, hour, day, wday, month;
		uint16_t year;
		char tmp[64];

		datetime_from_epoch(time_get_local(), &sec, &min, &hour, &day, &wday, &month, &year);

		sprintf_P(tmp, PSTR("\n\n === APPEND %02d.%02d.%04d %02d:%02d.%02d ===\n"), day, month, year, hour, min, sec);

		len = strlen(tmp);
		res = f_write(&debug_file, tmp, len, &wt);
		if (res != FR_OK || wt != len)
		{
			f_close(&debug_file);
			return;
		}

		//file is ready
		debug_file_open = true;
	}

	uint8_t * tmp = 0;

	//write content
	len = debug_log_storage.Read(DEBUG_LOG_BUFFER_CHUNK, &tmp);
	res = f_write(&debug_file, tmp, len, &wt);
	if (res != FR_OK || wt != len)
	{
		f_close(&debug_file);
		debug_file_open = false;

		return;
	}

	//sync file
	res = f_sync(&debug_file);
	if (res != FR_OK || wt != len)
	{
		f_close(&debug_file);
		debug_file_open = false;
	}
}
#endif

void ewdt_init()
{
	wdt_init(wdt_2s);
#ifndef DISABLE_DEBUG
	debug_timer_init();
#endif
}

void ewdt_reset()
{
	wdt_reset();
#ifndef DISABLE_DEBUG
	debug_timer.SetValue(0);
#endif
}
