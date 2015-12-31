#include <malloc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "musashi/m68k.h"

#include "wordops.h"



/// ROM length
#define ROM_LENGTH 256*1024

/// RAM base
#define RAM_BASE 0x200000

/// RAM length
#define RAM_LENGTH 64*1024


// System ROM
uint8_t rom[ROM_LENGTH];

// System RAM
uint8_t ram[RAM_LENGTH];



// Disassembler: can only access ROM and RAM
uint32_t m68k_read_disassembler_32(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return DWORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		return DWORD_READ(ram, address - RAM_BASE);
	} else {
		// ye cannae read empty space, cap'n!
		return 0xFFFFFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_disassembler_16(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return WORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		return WORD_READ(ram, address - RAM_BASE);
	} else {
		return 0xFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_memory_32(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return DWORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		return DWORD_READ(ram, address - RAM_BASE);
	} else {
		// FIXME log access to unimplemented peripheral
		return 0xFFFFFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_memory_16(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return WORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		return WORD_READ(ram, address - RAM_BASE);
	} else {
		// FIXME log access to unimplemented peripheral
		return 0xFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_memory_8(uint32_t address)/*{{{*/
{
	if (address < ROM_LENGTH) {
		return WORD_READ(rom, address);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		return WORD_READ(ram, address - RAM_BASE);
	} else {
		// FIXME log access to unimplemented peripheral
		return 0xFFFF;
	}
}
/*}}}*/

void m68k_write_memory_32(uint32_t address, uint32_t value)/*{{{*/
{
	if (address < ROM_LENGTH) {
		// WRITE TO ROM
		fprintf(stderr, "WR32 to ROM 0x%08x => 0x%08X ignored\n", address, value);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		// write to RAM
		DWORD_WRITE(ram, address - RAM_BASE, value);
	} else {
		// FIXME log
	}
}
/*}}}*/

void m68k_write_memory_16(uint32_t address, uint32_t value)/*{{{*/
{
	if (address < ROM_LENGTH) {
		// WRITE TO ROM
		fprintf(stderr, "WR16 to ROM 0x%08x => 0x%04X ignored\n", address, value);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		// write to RAM
		WORD_WRITE(ram, address - RAM_BASE, value);
	} else {
		// FIXME log
	}
}
/*}}}*/

void m68k_write_memory_8(uint32_t address, uint32_t value)/*{{{*/
{
	if (address < ROM_LENGTH) {
		// WRITE TO ROM
		fprintf(stderr, "WR-8 to ROM 0x%08x => 0x%02X ignored\n", address, value);
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_LENGTH))) {
		// write to RAM
		ram[address - RAM_BASE] = value;
	} else {
		// FIXME log
	}
}
/*}}}*/


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

	// Blank RAM (or fill it with crap?) who cares?

	// Boot the 68000

	return 0;
}
