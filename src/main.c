#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "musashi/m68k.h"

#include "wordops.h"



/// ROM length
#define ROM_LENGTH (256*1024)

/// RAM base
#define RAM_BASE 0x200000

/// RAM window (limit of address space allocated to RAM)
#define RAM_WINDOW ((128*1024)-1)

/// RAM physical length
#define RAM_LENGTH (128*1024)



// System ROM
uint8_t rom[ROM_LENGTH];

// System RAM
uint8_t ram[RAM_LENGTH];


// Value to return if the CPU reads from unimplemented memory
#define UNIMPLEMENTED_VALUE (0xFFFFFFFF)
//#define UNIMPLEMENTED_VALUE (0)


#define LOG_UNHANDLED


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
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD32 from UNHANDLED 0x%08x ignored, pc=%08X\n", address, m68k_get_reg(NULL, M68K_REG_PPC));
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
		return 0;		// FIXME PHASE REGISTER
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD16 from UNHANDLED 0x%08x ignored, pc=%08X\n", address, m68k_get_reg(NULL, M68K_REG_PPC));
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
	} else if ((address == 0x240200) || (address == 0x240201)) {
		// phase register
		return 0xff;		// FIXME PHASE REGISTER
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD-8 from UNHANDLED 0x%08x ignored, pc=%08X\n", address, m68k_get_reg(NULL, M68K_REG_PPC));
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
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR32 to UNHANDLED 0x%08x => 0x%08X ignored, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
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
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR16 to UNHANDLED 0x%08x => 0x%04X ignored, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
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
//		printf("UART Write reg %d: 0x%08X\n", (address / 2) & 0x0F, value);
//		printf("%c", (char)value);
		uint8_t reg = (address / 2) & 0x0F;
		if (reg == 0x0C) { printf("UART IVR -> 0x%02X, pc=%08X\n", value, m68k_get_reg(NULL, M68K_REG_PPC)); }
//		if (reg == 0x03) { printf("%c", value); }
		if (reg == 0x0B) { printf("UART THRB -> %c\n", value); }
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR-8 to UNHANDLED 0x%08x => 0x%02X '%c' ignored, pc=%08X\n",
				address, value, isprint(value) ? value : '.', m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	}
}
/*}}}*/

volatile struct {
	bool phase_tick;
	bool uart;
} InterruptFlags;

int m68k_irq_callback(int int_level)
{
	uint8_t intr_num = 0;

	if (InterruptFlags.phase_tick)
	{
		InterruptFlags.phase_tick = 0;
		intr_num |= 0x55;
	}

	if (InterruptFlags.uart)
	{
		InterruptFlags.uart = 0;
		intr_num = 0x00;
	}

	m68k_set_irq(0);
	return intr_num;
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

	// Boot the 68000
	//
#define SYSTEM_CLOCK 20e6 /*Hz*/
#define INTERRUPT_RATE 1000 /* Hz */
#define CLOCKS_PER_INTERRUPT  (SYSTEM_CLOCK / INTERRUPT_RATE)

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
			InterruptFlags.phase_tick = true;
			m68k_set_irq(7);
		} else {
			n++;
		}

		// TODO: Delay 1/TIMESLOT_FREQUENCY to make this run at real time
		//
		//return 0;
	}

	return 0;
}
