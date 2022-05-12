#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "m68k.h"

#include "uart.h"
#include "machine.h"
#include "wordops.h"

#include "main.h"


// Define this to log unhandled memory accesses
#define LOG_UNHANDLED

// Log state changes of the UART output port
// #define LOG_UART_OUTPORT

// Suppress logging from noisy unimplemented devices
#define LOG_SILENCE_240800

// System ROM
uint8_t rom[ROM_LENGTH];

// System RAM
uint8_t ram[RAM_LENGTH];

// Active interrupts
volatile InterruptFlags_s InterruptFlags;

// Interrupt priority levels
#define IPL_UART 2
#define IPL_PHASE 4
#define IPL_NMI 7


const char *GetDevFromAddr(const uint32_t address)
{
	if ((address >= 0x240000) && (address <= 0x24FFFF)) {
		switch (address & 0xFFFF00) {
			case 0x240000:
				return "ADC";
				break;

			case 0x240100:
				return "EEPROM RDIO";		// read 240101 from pc=0001FC90
				break;

			case 0x240200:
				return "RF Phase";
				break;

			case 0x240300:
				return "UART";
				break;

			case 0x240500:
				return "UNK 24:05";
				break;

			case 0x240700:
				return "ADCON CHSEL";		// write 240701
				break;

			case 0x240800:
				return "EEPROM WRIO";		// write 240801 from pc=0001FC22, pc=0001FC2C, pc=0001FCB6, pc=0001FC44, pc=0001FC52, pc=0001FC6C
				break;

			default:
				return "UNK 24:??";
				break;
		}
	}

	return "?";
}


// Disassembler: can only access ROM and RAM
uint32_t m68k_read_disassembler_32(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return DWORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return DWORD_READ(ram, (address - RAM_BASE) & (RAM_LENGTH - 1));
	} else {
		// ye cannae read empty space, cap'n!
		return 0;
	}
}
/*}}}*/

uint32_t m68k_read_disassembler_16(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return WORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return WORD_READ(ram, (address - RAM_BASE) & (RAM_LENGTH - 1));
	} else {
		return 0;
	}
}
/*}}}*/

uint32_t m68k_read_memory_32(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return DWORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return DWORD_READ(ram, (address - RAM_BASE) & (RAM_LENGTH - 1));
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		fprintf(stderr, "RD32 %s <%s> 0x%08x ignored, pc=%08X\n",
				GetDevFromAddr(address), GetUartRegFromAddr(address, true),
				address, m68k_get_reg(NULL, M68K_REG_PPC));
		return UNIMPLEMENTED_VALUE;
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD32 UNHANDLED [%-9s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
		return UNIMPLEMENTED_VALUE;
	}
}
/*}}}*/

uint32_t m68k_read_memory_16(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return WORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return WORD_READ(ram, (address - RAM_BASE) & (RAM_LENGTH - 1));
	} else if ((address == 0x240200) || (address == 0x240201)) {
		// phase register
		return 0xff;		// FIXME PHASE REGISTER
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		fprintf(stderr, "RD16 %s <%s> 0x%08x UNIMPLEMENTED_RWSIZE, pc=%08X\n",
				GetDevFromAddr(address), GetUartRegFromAddr(address, true),
				address, m68k_get_reg(NULL, M68K_REG_PPC));
		return UNIMPLEMENTED_VALUE & 0xFFFF;
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD16 UNHANDLED [%-9s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
		return UNIMPLEMENTED_VALUE & 0xFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_memory_8(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return rom[address];
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return ram[(address - RAM_BASE) & (RAM_LENGTH - 1)];
	} else if ((address == 0x240100) || (address == 0x240101)) {
		return 0xff;		// FIXME 240101 EEPROM DATA READ REG
	} else if ((address == 0x240200) || (address == 0x240201)) {
		// phase register
		return 0x00;		// FIXME PHASE REGISTER
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		return UartRegRead(address);
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD-8 UNHANDLED [%-9s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
		return UNIMPLEMENTED_VALUE & 0xFF;
	}
}
/*}}}*/

void m68k_write_memory_32(unsigned int address, unsigned int value)/*{{{*/
{
	if (address < ROM_LENGTH) {
		// WRITE TO ROM
#ifdef LOG_UNHANDLED_ROM
		fprintf(stderr, "WR32 to ROM 0x%08x => 0x%08X ignored, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		// write to RAM
		DWORD_WRITE(ram, (address - RAM_BASE) & (RAM_LENGTH - 1), value);
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		fprintf(stderr, "WR32 %s <%s> 0x%08x => 0x%08x ignored, pc=%08X\n",
				GetDevFromAddr(address), GetUartRegFromAddr(address, false),
				address, value, m68k_get_reg(NULL, M68K_REG_PPC));
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR32 UNHANDLED [%-9s] 0x%08x => 0x%08X ignored, pc=%08X\n", GetDevFromAddr(address), address, value, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	}
}
/*}}}*/

void m68k_write_memory_16(unsigned int address, unsigned int value)/*{{{*/
{
	assert(value <= 0xFFFF);

	if (address < ROM_LENGTH) {
		// WRITE TO ROM
#ifdef LOG_UNHANDLED_ROM
		fprintf(stderr, "WR16 to ROM 0x%08x => 0x%04X ignored, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		// write to RAM
		WORD_WRITE(ram, (address - RAM_BASE) & (RAM_LENGTH - 1), value);
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		fprintf(stderr, "WR16 %s <%s> 0x%08x => 0x%04x ignored, pc=%08X\n",
				GetDevFromAddr(address), GetUartRegFromAddr(address, false),
				address, value, m68k_get_reg(NULL, M68K_REG_PPC));
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR16 UNHANDLED [%-9s] 0x%08x => 0x%04X ignored, pc=%08X\n", GetDevFromAddr(address), address, value, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	}
}
/*}}}*/

void m68k_write_memory_8(unsigned int address, unsigned int value)/*{{{*/
{
	assert(value <= 0xFF);

	if (address < ROM_LENGTH) {
		// WRITE TO ROM
#ifdef LOG_UNHANDLED_ROM
		fprintf(stderr, "WR-8 to ROM 0x%08x => 0x%02X ignored, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		// write to RAM
		ram[(address - RAM_BASE) & (RAM_LENGTH - 1)] = value;
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		// UART -- SCC68692
		UartRegWrite(address, value);
#ifdef LOG_SILENCE_240800
	} else if ((address == 0x240800) || (address == 0x240801)) {
		// FIXME UNHANDLED 2408xx
#endif
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR-8 UNHANDLED [%-9s] 0x%08x => 0x%02X '%c' ignored, pc=%08X\n",
				GetDevFromAddr(address), address, value,
				isprint(value) ? value : '.', m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	}
}
/*}}}*/


// Get current IPL with priority encoding
void m68k_update_ipl(void)
{
       // Start with IPL=0, no interrupts
       int ipl = 0;

       if ((InterruptFlags.phase_tick) && (ipl < IPL_PHASE)) {
               ipl = IPL_PHASE;
       }

       if ((InterruptFlags.uart) && (ipl < IPL_UART)) {
               ipl = IPL_UART;
       }

       m68k_set_irq(ipl);
}


int m68k_irq_callback(int int_level)
{
	uint8_t vector = 0;

	// raise 1ms tick interrupt if needed
	if (InterruptFlags.phase_tick)
	{
		InterruptFlags.phase_tick = 0;
		vector = 0x55;		// TODO check this interrupt number
	}

	// raise UART interrupt if needed
	if (InterruptFlags.uart)
	{
		InterruptFlags.uart = 0;
		vector = Uart.IVR;
	}

	// if no further pending interrupts, clear the IRQ level
	m68k_update_ipl();

	fprintf(stderr, "IVEC: %02X\n", vector);
	return vector;
}

int main(int argc, char **argv)
{
	// Load ROM. Order is: A byte from IC2, then a byte from IC1.
	{
		FILE *oddf, *evenf;
		uint8_t *evenbuf, *oddbuf;

		// Allocate odd and even buffers
		oddbuf = malloc(ROM_LENGTH / 2);
		if (oddbuf == NULL) {
			fprintf(stderr, "Error allocating memory.\n");
			return EXIT_FAILURE;
		}
		evenbuf = malloc(ROM_LENGTH / 2);
		if (evenbuf == NULL) {
			free(oddbuf);
			fprintf(stderr, "Error allocating memory.\n");
			return EXIT_FAILURE;
		}

		// Open odd ROM
		oddf = fopen("ic2.bin", "rb");
		if (oddf == NULL) {
			fprintf(stderr, "Error: can't open ic2.bin\n");
			return EXIT_FAILURE;
		}

		// Open even ROM
		evenf = fopen("ic1.bin", "rb");
		if (evenf == NULL) {
			fclose(oddf);
			fprintf(stderr, "Error: can't open ic1.bin\n");
			return EXIT_FAILURE;
		}

		// Read odd ROM
		if (fread(oddbuf, 1, ROM_LENGTH / 2, oddf) != ROM_LENGTH / 2) {
			fprintf(stderr, "Error reading Odd ROM\n");
			fclose(oddf);
			fclose(evenf);
			free(oddbuf);
			free(evenbuf);
		}

		// Read even ROM
		if (fread(evenbuf, 1, ROM_LENGTH / 2, evenf) != ROM_LENGTH / 2) {
			fprintf(stderr, "Error reading Even ROM\n");
			fclose(oddf);
			fclose(evenf);
			free(oddbuf);
			free(evenbuf);
		}

		// Close file pointers
		fclose(oddf);
		fclose(evenf);

		// Sort odd and even bytes into the correct order
		for (uint32_t i=0, j=0; i<ROM_LENGTH; i+=2, j++) {
			rom[i+0] = oddbuf[j];
			rom[i+1] = evenbuf[j];
		}

		// Free the read buffers
		free(oddbuf);
		free(evenbuf);
	}

	// Init the debug UART
	UartInit();

	// Boot the 68000
	//
#define SYSTEM_CLOCK 20e6 /*Hz*/
#define INTERRUPT_RATE 1000 /* Hz */
#define CLOCKS_PER_INTERRUPT  (SYSTEM_CLOCK / INTERRUPT_RATE)

	m68k_init();
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_set_int_ack_callback(&m68k_irq_callback);
	m68k_pulse_reset();

	uint32_t clock_cycles = 0;
	uint32_t n=0;

	for (;;) {
		// Run one tick interrupt worth of instructions
		uint32_t tmp = m68k_execute(CLOCKS_PER_INTERRUPT);
		clock_cycles += tmp;

		if (n > 10) {
			// Trigger a tick interrupt
			//fprintf(stderr, "***TICK***\n");
			InterruptFlags.phase_tick = true;
		} else {
			n++;
		}

		m68k_update_ipl();

		// TODO: Delay 1/TIMESLOT_FREQUENCY to make this run at real time
		//
		//return 0;
	}

	// Shut down the UART
	UartDone();

	return 0;
}
