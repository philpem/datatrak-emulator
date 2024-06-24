#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <math.h>
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

// Define this to log interrupt vector numbers when an interrupt is triggered
// #define LOG_INTERRPUT_VECTOR

// Suppress logging from noisy unimplemented devices
#define LOG_SILENCE_240800
#define LOG_SILENCE_ADC

// Debug: write modulated (audible) phase data to a file. Data format is 16bit signed, mono, 44100 Hz.
//#define WRITE_PHASEDATA_MODULATED
// Debug: write raw phase data to a file. Data format is 16bit signed, mono, 44100 Hz.
//#define WRITE_PHASEDATA

// System ROM
uint8_t rom[ROM_LENGTH];

// System RAM
uint8_t ram[RAM_LENGTH];

// Active interrupts
volatile InterruptFlags_s InterruptFlags;

// Interrupt priority levels
#define IPL_UART	2
#define IPL_PHASE	5
#define IPL_NMI		7

// Interrupt vector numbers
// Phase tick could be interrupt 85, 170 or 255 -- all go to the same handler
#define	IVEC_PHASE_TICK		255


// Phase-mod data generation

#define PHASE_PER_CYCLE 1680

void fillPhasebuf(void);
void writeSynthPhasebuf(void);

uint16_t phasebuf[PHASE_PER_CYCLE];
size_t phasebuf_rpos = 0;


#define PHASE_ZERO 499
#define PHASE_AMPL 499

// Trigger templates
uint16_t trig_50_template[40];
uint16_t trig_375_template[40];


// Trigger templates from the Datatrak firmware
int16_t DT_TRIG50_TEMPLATE[40] = {
	  54,    124,    181,    218,
	 232,    221,    185,    129,
	  59,    -21,    -99,   -169,
	-223,   -257,   -265,   -250,
	-210,   -150,    -76,      6,
	  87,    159,    215,    249,
	 260,    245,    206,    147,
	  74,     -8,    -89,   -160,
	-216,   -251,   -261,   -245,
	-207,   -148,    -74,      8
};
int16_t DT_TRIG375_TEMPLATE[40] = {
	 -43,    -98,   -144,   -181,
	-203,   -212,   -204,   -183,
	-149,   -106,    -53,      4,
	  62,    118,    168,    210,
 	 240,    258,    263,    253,
	 229,    193,    147,     93,
	  33,    -28,    -88,   -143,
	-189,   -225,   -248,   -258,
	-254,   -236,   -204,   -162,
	-110,    -53,      9,     69
};

void initPhasegen(void)
{
#ifdef GENERATE_TRIGGER
	// 40ms at 1kHz sample rate = 40 samples
	const double nSamples = 40.0; // 1000.0 * 40.0e-3;
# warning "Generating Trigger from scratch"
	for (int i=0; i < 40; i++) {
		// 50Hz trigger is 2 cycles of 50Hz
		trig_50_template[i] = trunc(sin((double)(i) / nSamples * M_PI * 2.0 * 2.0) * PHASE_AMPL + PHASE_ZERO);
		// 37.5Hz trigger is 1.5 cycles of 37.5Hz with 180-degree phase offset
		trig_375_template[i] = trunc(sin(((double)(i) / nSamples * M_PI * 2.0 * 1.5) + M_PI) PHASE_AMPL + PHASE_ZERO);
	}
#else
# warning "Generating Trigger from rescaled firmware values"
	const double scale = 1.73;	// 1.73 gives the best trigger match quality (705)
								// 500 peak = 289 after scaling, what's going on?
	
	for (int i=0; i < 40; i++) {
		// Re-scale the firmware templates - convert from signed-around-zero to unsigned 0-1000
		trig_50_template[i] = trunc((double)(DT_TRIG50_TEMPLATE[i]) * scale + PHASE_ZERO);
		trig_375_template[i] = trunc((double)(DT_TRIG375_TEMPLATE[i]) * scale + PHASE_ZERO);
	}
#endif

	// do the initial buffer fill
	fillPhasebuf(); 
}

const uint32_t GOLDCODE[] = {0xFA9B8700, 0xAE32BD97};
size_t goldcode_n = 0;
size_t clock_n = 12345;

void fillPhasebuf(void)
{
	uint8_t goldcode_word = (goldcode_n / 32);
	uint8_t goldcode_bit = (goldcode_n % 32);

	for (size_t i=0; i<PHASE_PER_CYCLE; i++) {
		if (i < 45) {							// AA1 and S - 40ms
			phasebuf[i] = PHASE_ZERO;
		} else if ((i >= 45) && (i < 85)) {		// TRIGGER
			// -- Trigger (Gold Code) --
			if (GOLDCODE[goldcode_word] & (1<<goldcode_bit)) {
				// bit is a '1'
				phasebuf[i] = roundf(trig_375_template[i-45] * 1);
			} else {
				// bit is a '0'
				phasebuf[i] = roundf(trig_50_template[i-45] * 1);
			}
		} else if ((i >= 95) && (i < 115)) {
			const float CLOCK_AMPL = 0.5;

			// -- Clock --
			int bit_n = (goldcode_n % 8) * 2;
			int bits = (clock_n >> bit_n) & 3;
			if (goldcode_n >= 32) bits = bits ^ 3;
			int pha;
			switch(bits) {
				case 0: pha = 0; break;
				case 1: pha = 5; break;
				case 2: pha = 15; break;
				case 3: pha = 10; break;
			}
			phasebuf[i] = roundf((trig_50_template[((i-95)+pha) % 20] * CLOCK_AMPL) + (PHASE_ZERO * (1.0 - CLOCK_AMPL)));
		} else {
			phasebuf[i] = PHASE_ZERO;
		}
	}

	// advance to next period
	goldcode_n++;
	if (goldcode_n == 64) {
		goldcode_n = 0;
		clock_n++;
	}

#if defined(WRITE_PHASEDATA_MODULATED) || defined(WRITE_PHASEDATA)
	writeSynthPhasebuf();
#endif
}

#if defined(WRITE_PHASEDATA_MODULATED) || defined(WRITE_PHASEDATA)
float phi = 0;
void writeSynthPhasebuf(void)
{
	FILE *fp = fopen("phasedata_synth.raw", "ab");

	const double SAMPLERATE = 44100;
	const double FREQUENCY  = 1000;
	const double AMPLITUDE  = 32767 * 0.25;

	const size_t SAMPLES_PER_MS = SAMPLERATE/1000;

	int16_t samp[SAMPLES_PER_MS];
	double theta = (2.0 * M_PI) * FREQUENCY / SAMPLERATE;
	int last_ph = PHASE_ZERO;
	for (size_t msec = 0; msec < PHASE_PER_CYCLE; msec++) {
		for (size_t s = 0; s < SAMPLES_PER_MS; s++) {
			// calculate phase shift from last cycle to this
			double ph_sh = (((int)phasebuf[msec] - last_ph) / (double)PHASE_AMPL) * (2.0 * M_PI);
			last_ph = phasebuf[msec];

			// update phase
			phi = phi + theta + ph_sh;

			// generate sine point
			samp[s] = roundf(AMPLITUDE * sin(phi));
#ifndef WRITE_PHASEDATA_MODULATED
			// output raw phase data instead
			samp[s] = ((int)phasebuf[msec] - PHASE_ZERO) * 32;
#endif
		}
		fwrite(samp, sizeof(int16_t), SAMPLES_PER_MS, fp);
	}
	fclose(fp);
}
#endif

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

			case 0x240400:
				return "8051 I/O";
				break;

			case 0x240500:
				return "F1/F2 FREQ SET";
				break;

			case 0x240600:
				return "F1+/F2+ FREQ SET";
				break;

			case 0x240700:
				return "ADCON CHSEL (DIGOP1)";		// write 240701
				break;

			case 0x240800:
				return "EEPROM WRIO (DIGOP2)";		// write 240801 from pc=0001FC22, pc=0001FC2C, pc=0001FCB6, pc=0001FC44, pc=0001FC52, pc=0001FC6C
				break;

			case 0x240900:
				return "DUSC";
				break;

			case 0x240A00:
				return "UPDOWN CNT 1";
				break;

			case 0x240B00:
				return "UPDOWN CNT 2";
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
		fprintf(stderr, "RD32 UNHANDLED [%-12s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
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
	} else if (address == 0x240200) {
#ifdef LOG_PHASE_REG
		printf("\nPHASE_L RD16\n");
#endif
		// phase register low
		// the firmware usually does a 16bit read of this

		// this causes an autoincrement

		uint8_t val = phasebuf[phasebuf_rpos] >> 8; 
		phasebuf_rpos++;

		// emptied the buffer
		if (phasebuf_rpos >= PHASE_PER_CYCLE) {
			phasebuf_rpos = 0;
			fillPhasebuf();
		}
		
		return val;
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		fprintf(stderr, "RD16 %s <%s> 0x%08x UNIMPLEMENTED_RWSIZE, pc=%08X\n",
				GetDevFromAddr(address), GetUartRegFromAddr(address, true),
				address, m68k_get_reg(NULL, M68K_REG_PPC));
		return UNIMPLEMENTED_VALUE & 0xFFFF;
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD16 UNHANDLED [%-12s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
		return UNIMPLEMENTED_VALUE & 0xFFFF;
	}
}
/*}}}*/

uint32_t m68k_read_memory_8(uint32_t address)/*{{{*/
{
	if (address == 0x2CC96) {
		printf("*** 2cc96 trap -> pc = %08X\n", m68k_get_reg(NULL, M68K_REG_PC));
	}

	if (address < ROM_LENGTH) {
		return rom[address];
	} else if ((address >= RAM_BASE) && (address < (RAM_BASE + RAM_WINDOW))) {
		return ram[(address - RAM_BASE) & (RAM_LENGTH - 1)];
	} else if ((address == 0x240100) || (address == 0x240101)) {
		return 0xff;		// FIXME 240101 EEPROM DATA READ REG
	} else if (address == 0x240200) {
#ifdef LOG_PHASE_REG
		printf("\nPHASE_L RD8\n");
#endif
		// phase register low
		// this causes an autoincrement

		uint8_t val = phasebuf[phasebuf_rpos] >> 8; 
		phasebuf_rpos++;

		// emptied the buffer
		if (phasebuf_rpos >= PHASE_PER_CYCLE) {
			phasebuf_rpos = 0;
			fillPhasebuf();
		}
		
		return val;
	} else if (address == 0x240201) {
#ifdef LOG_PHASE_REG
		// phase register high -- this is read first
		printf("\nPHASE_H RD8\n");
#endif
		return phasebuf[phasebuf_rpos] & 0xFF;
	} else if ((address >= 0x240300) && (address <= 0x2403FF)) {
		return UartRegRead(address);

		// 240401 -- Alarm port
#ifdef LOG_SILENCE_ADC
	} else if ((address == 0x240000) || (address == 0x240001)) {
		// FIXME UNHANDLED 2400xx ADC
		return UNIMPLEMENTED_VALUE & 0xFF;
	} else if ((address == 0x240700) || (address == 0x240701)) {
		// FIXME UNHANDLED 2407xx ADC CHANNEL SELECT
		return UNIMPLEMENTED_VALUE & 0xFF;
#endif
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "RD-8 UNHANDLED [%-12s] 0x%08x ignored, pc=%08X\n", GetDevFromAddr(address), address, m68k_get_reg(NULL, M68K_REG_PPC));
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
		fprintf(stderr, "WR32 UNHANDLED [%-12s] 0x%08x => 0x%08X ignored, pc=%08X\n", GetDevFromAddr(address), address, value, m68k_get_reg(NULL, M68K_REG_PPC));
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
		fprintf(stderr, "WR16 UNHANDLED [%-12s] 0x%08x => 0x%04X ignored, pc=%08X\n", GetDevFromAddr(address), address, value, m68k_get_reg(NULL, M68K_REG_PPC));
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
#ifdef LOG_SILENCE_ADC
	} else if ((address == 0x240000) || (address == 0x240001)) {
		// FIXME UNHANDLED 2400xx ADC
	} else if ((address == 0x240700) || (address == 0x240701)) {
		// FIXME UNHANDLED 2407xx ADC CHANNEL SELECT
#endif
#ifdef LOG_SILENCE_240800
	} else if ((address == 0x240800) || (address == 0x240801)) {
		// FIXME UNHANDLED 2408xx
#endif
	} else if ((address >= 0x240000) && (address <= 0x24FFFF)) {
		fprintf(stderr, "WR-8 to ASIC 0x%08X => 0x%02X, pc=%08X\n", address, value, m68k_get_reg(NULL, M68K_REG_PPC));
	} else {
		// log unhandled access
#ifdef LOG_UNHANDLED
		fprintf(stderr, "WR-8 UNHANDLED [%-12s] 0x%08x => 0x%02X '%c' ignored, pc=%08X\n",
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

	else if ((InterruptFlags.uart) && (ipl < IPL_UART)) {
		ipl = IPL_UART;
	}

	m68k_set_irq(ipl);
}


int m68k_irq_callback(int int_level)
{
	int vector = M68K_INT_ACK_SPURIOUS;

	// raise 1ms tick interrupt if needed
	if (InterruptFlags.phase_tick)
	{
		InterruptFlags.phase_tick = false;
		vector = IVEC_PHASE_TICK;
	}

	// raise UART interrupt if needed
	else if (InterruptFlags.uart)
	{
		InterruptFlags.uart = false;
		vector = Uart.IVR;
	}

	m68k_update_ipl();

#ifdef LOG_INTERRPUT_VECTOR
	if (vector != Uart.IVR) {
		fprintf(stderr, "IVEC: %02X\n", vector);
	}
#endif
	return vector;
}

int main(int argc, char **argv)
{
	// Load ROM. Order is: A byte from IC2, then a byte from IC1.
#if 1
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
#else
	// load TUTOR monitor rom
	{
		FILE *f = fopen("./monitor_rom/tutor13.bin", "rb");
		fread(rom, 1, sizeof(rom), f);
		fclose(f);
	}
#endif

	// Init the debug UART
	UartInit();

	// Init the phase modulation engine
	initPhasegen();

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

	for (;;) {
		// Run one tick interrupt worth of instructions
		uint32_t tmp = m68k_execute(CLOCKS_PER_INTERRUPT);
		clock_cycles += tmp;

		// Trigger a tick interrupt
		InterruptFlags.phase_tick = true;

		m68k_update_ipl();

		// TODO: Delay 1/TIMESLOT_FREQUENCY to make this run at real time
		//
		//return 0;
	}

	// Shut down the UART
	UartDone();

	return 0;
}
