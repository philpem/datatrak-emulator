#include <assert.h>
#include <ctype.h>
#include <malloc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

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

// Value to return if the CPU reads from unimplemented memory
#ifdef UNIMPL_READS_AS_FF
#  define UNIMPLEMENTED_VALUE (0xFFFFFFFF)
#else
#  define UNIMPLEMENTED_VALUE (0)
#endif

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
volatile struct {
	bool phase_tick;
	bool uart;
} InterruptFlags;


const char *GetDevFromAddr(const uint32_t address)
{
	if ((address >= 0x240000) && (address <= 0x24FFFF)) {
		switch (address & 0xFFFF00) {
			case 0x240000:
				return "ADC";
				break;

			case 0x240100:
				return "UNK 24:01";		// read 240101 from pc=0001FC90
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
				return "UNK 24:07";		// write 240701
				break;

			case 0x240800:
				return "UNK 24:08";		// write 240801 from pc=0001FC22, pc=0001FC2C, pc=0001FCB6, pc=0001FC44, pc=0001FC52, pc=0001FC6C
				break;

			default:
				return "UNK 24:??";
				break;
		}
	}

	return "?";
}

///////
// UART EMULATION
///////

#define UART_PORT 8888

struct {
	int SocketA, SocketB;
	bool TxEnA, TxEnB;
	bool RxEnA, RxEnB;
	bool MRnA, MRnB;
	uint8_t MRA[2];
	uint8_t MRB[2];
	uint8_t IMR;
	uint8_t IVR;
	uint8_t OutPort;
} Uart;

void die(char *s)
{
    perror(s);
    exit(1);
}

static int UartInit(void)
{
	struct sockaddr_in si_Uart;

	// initialise UART registers
	memset(&Uart, '\0', sizeof(Uart));
	Uart.TxEnA = Uart.TxEnB = false;
	Uart.RxEnA = Uart.RxEnB = false;
	Uart.MRnA  = Uart.MRnB  = false;

	// Default interrupt vector is 0x0F on reset
	Uart.IVR = 0x0F;

	// Create a TCP socket
	if ((Uart.SocketA = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	{
		die("Failed to create socket for UARTA");
	}
	if ((Uart.SocketB = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	{
		die("Failed to create socket for UARTB");
	}

	// Construct the server's sockaddr_in structure -- this is for UART_A
	// The UART A Interface connects to port UART_PORT
	memset(&si_Uart, 0, sizeof(si_Uart));
	si_Uart.sin_family		= AF_INET;
	si_Uart.sin_addr.s_addr	= inet_addr("127.0.0.1");
	si_Uart.sin_port		= htons(UART_PORT);

	// Establish connection
	if (connect(Uart.SocketA, (struct sockaddr *)&si_Uart, sizeof(si_Uart)) < 0) {
		fprintf(stderr, "Failed to connect to UART_A terminal (port %d)\n", UART_PORT);
		close(Uart.SocketA);
		Uart.SocketA = -1;
	}

	// Construct the server's sockaddr_in structure -- this is for UART_A
	// The UART A Interface connects to port UART_PORT
	memset(&si_Uart, 0, sizeof(si_Uart));
	si_Uart.sin_family		= AF_INET;
	si_Uart.sin_addr.s_addr	= inet_addr("127.0.0.1");
	si_Uart.sin_port		= htons(UART_PORT + 1);

	// Establish connection
	if (connect(Uart.SocketB, (struct sockaddr *)&si_Uart, sizeof(si_Uart)) < 0) {
		fprintf(stderr, "Failed to connect to UART_B terminal (port %d)\n", UART_PORT+1);
		close(Uart.SocketB);
		Uart.SocketB = -1;
	}

	// TODO set nonblocking

	return 0;
}

static void UartDone(void)
{
	if (Uart.SocketA > 0) {
		close(Uart.SocketA);
	}

	if (Uart.SocketB > 0) {
		close(Uart.SocketB);
	}
}

static uint8_t UartRx(void)
{
	uint8_t buf;

	// TODO select()

	if (recv(Uart.SocketA, &buf, 1, 0) < 1) {
		die("Failed to receive byte from TTY");
	}

	return buf;
}


static const char *GetUartRegFromAddr(const uint32_t addr, const bool reading)
{
	const char *RA[16][2] = {
		{ "MR1A/MR2A",	"MR1A/MR2A"	},
		{ "SRA",		"CSRA"		},
		{ "BRG Test",	"CRA"		},
		{ "RHRA",		"THRA"		},
		{ "IPCR",		"ACR"		},
		{ "ISR",		"IMR"		},
		{ "CUR",		"CTUR"		},
		{ "CLR",		"CTLR"		},

		{ "MR1B/MR2B",	"MR1B/MR2B"	},
		{ "SRB",		"CSRB"		},
		{ "1x/16x Test","CRB"		},
		{ "RHRB",		"THRB"		},
		{ "IVR",		"IVR"		},
		{ "IP0-6",		"OPCR"		},
		{ "START COUNTER", "SET OUT BITS" },
		{ "STOP  COUNTER", "RESET OUT BITS" }
	};

	if (reading) {
		return RA[(addr >> 1) & 0x0F][0];
	} else {
		return RA[(addr >> 1) & 0x0F][1];
	}
}

void UartRegWrite(uint32_t address, uint8_t value)
{
	// enable/disable states
	const char *ENDIS[4] = { "unch", "ENA ", "DIS ", "??? " };
	// command codes
	const char *CMDS[16] = {
		"Null",
		"Reset MRn Pointer",
		"Reset Receiver",
		"Reset Transmitter",
		"Reset Error Status",
		"Reset Break Change interrupt",
		"Start Break",
		"Stop Break",
		"Set   Rx BRG Select Extend bit",
		"Clear Rx BRG Select Extend bit",
		"Set   Tx BRG Select Extend bit",
		"Clear Tx BRG Select Extend bit",
		"Set Standby mode",
		"Set Active mode",
		"rsvd 14",
		"rsvd 15"
	};

	fprintf(stderr, "WR-8 %s <%s> 0x%08x => 0x%02x, pc=%08X\n",
			GetDevFromAddr(address), GetUartRegFromAddr(address, false),
			address, value, m68k_get_reg(NULL, M68K_REG_PPC));

	switch ((address / 2) & 0x0F) {
		case 2:		// Command Register A
			printf("UART CRA -->  RxEn %s  TxEn %s  Cmd:%s\n",
					ENDIS[value & 0x03],
					ENDIS[(value >> 2) & 0x03],
					CMDS[(value >> 4) & 0x0F]);

			switch (value & 0x03) {
				case 0:	// RX unchanged
					break;
				case 1:	// enable RX
					Uart.RxEnA = true; break;
				case 2: // disable RX
					Uart.RxEnA = false; break;
			}

			switch ((value >> 2) & 0x03) {
				case 0:	// TX unchanged
					break;
				case 1:	// enable TX
					Uart.TxEnA = true; break;
				case 2: // disable TX
					Uart.TxEnA = false; break;
			}

			switch ((value >> 4) & 0x0F) {
				case 0:			// null command
					break;

				case 1:			// Reset MRn pointer
					Uart.MRnA = false;
					break;

				case 2:			// Reset receiver
					Uart.RxEnA = false;
					break;

				case 3:			// Reset transmitter
					Uart.TxEnA = false;

				case 4:			// Reset error status
				case 5:			// Reset break change interrupt
				case 6:			// Start break
				case 7:			// Stop break
					break;

				case 8:			// Set RX BRG select extend bit
				case 9:			// Clear RX BRG select extend bit
				case 10:		// Set TX BRG select extend bit
				case 11:		// Clear TX BRG select extend bit
					break;

				case 12:		// Set standby mode
				case 13:		// Set active mode
					break;
			}
			break;

		case 3:		// Transmit holding register A
			printf("UARTA --> %c  [%02x]\n", value, value);
			if (Uart.SocketA > 0) {
				if (send(Uart.SocketA, &value, 1, 0) != 1) {
					die("Mismatch in number of sent bytes (UARTA)");
				}
			}

			// If IMR transmit interrupt is enabled, pend a TX IRQ
			if ((Uart.IMR & 0x01) || (Uart.IMR & 0x10)) {
				InterruptFlags.uart = true;
			}

			break;


		case 5:			// Interrupt mask register
			fprintf(stderr, "UART IMR = %02X  --> ", value);
			Uart.IMR = value;
			if (Uart.IMR & 0x80) fprintf(stderr, "InPortChng ");
			if (Uart.IMR & 0x40) fprintf(stderr, "DeltaBrkB ");
			if (Uart.IMR & 0x20) fprintf(stderr, "RxRdy/FFullB ");
			if (Uart.IMR & 0x10) fprintf(stderr, "TxRdyB ");
			if (Uart.IMR & 0x08) fprintf(stderr, "CounterReady ");
			if (Uart.IMR & 0x04) fprintf(stderr, "DeltaBrkA ");
			if (Uart.IMR & 0x02) fprintf(stderr, "RxRdy/FFullA ");
			if (Uart.IMR & 0x01) fprintf(stderr, "TxRdyA ");
			fprintf(stderr, "\n");

			// If TX interrupt is enabled, pend one (transmit buffer clear)
			if ((Uart.IMR & 0x01) || (Uart.IMR & 0x10)) {
				InterruptFlags.uart = true;
			}
			break;


		case 10:		// Command Register B
			printf("UART CRB -->  RxEn %s  TxEn %s  Cmd:%s\n",
					ENDIS[value & 0x03],
					ENDIS[(value >> 2) & 0x03],
					CMDS[(value >> 4) & 0x0F]);

			switch (value & 0x03) {
				case 0:	// RX unchanged
					break;
				case 1:	// enable RX
					Uart.RxEnB = true; break;
				case 2: // disable RX
					Uart.RxEnB = false; break;
			}

			switch ((value >> 2) & 0x03) {
				case 0:	// TX unchanged
					break;
				case 1:	// enable TX
					Uart.TxEnB = true; break;
				case 2: // disable TX
					Uart.TxEnB = false; break;
			}

			switch ((value >> 4) & 0x0F) {
				case 0:			// null command
					break;

				case 1:			// Reset MRn pointer
					Uart.MRnB = false;
					break;

				case 2:			// Reset receiver
					Uart.RxEnB = false;
					break;

				case 3:			// Reset transmitter
					Uart.TxEnB = false;

				case 4:			// Reset error status
				case 5:			// Reset break change interrupt
				case 6:			// Start break
				case 7:			// Stop break
					break;

				case 8:			// Set RX BRG select extend bit
				case 9:			// Clear RX BRG select extend bit
				case 10:		// Set TX BRG select extend bit
				case 11:		// Clear TX BRG select extend bit
					break;

				case 12:		// Set standby mode
				case 13:		// Set active mode
					break;
			}
			break;

		case 11:	// Transmit holding register B
			printf("UARTB --> %c  [%02x]\n", value, value);
			if (Uart.SocketB > 0) {
				if (send(Uart.SocketB, &value, 1, 0) != 1) {
					die("Mismatch in number of sent bytes (UARTB)");
				}
			}
			break;


		case 12:	// Interrupt vector register
			Uart.IVR = value;
			printf("UART Int Vec = 0x%02X\n", Uart.IVR);
			break;


		case 14:	// Set Output Port Bits command
			Uart.OutPort |= (uint8_t)value;
#ifdef LOG_UART_OUTPORT
			fprintf(stderr, "UART OutPort state change --> now 0x%02X\n", Uart.OutPort);
#endif
			break;

		case 15:	// Reset Output Port Bits command
			Uart.OutPort &= ~(uint8_t)value;
#ifdef LOG_UART_OUTPORT
			fprintf(stderr, "UART OutPort state change --> now 0x%02X\n", Uart.OutPort);
#endif
			break;
	}
}

uint8_t UartRegRead(uint32_t address)
{
	fprintf(stderr, "RD-8 %s <%s> 0x%08x ignored, pc=%08X\n",
			GetDevFromAddr(address), GetUartRegFromAddr(address, true),
			address, m68k_get_reg(NULL, M68K_REG_PPC));

	switch ((address >> 1) & 0x0F) {
		case 1:		// Status Register A
		case 9:		// Status Register B
			return 0xC0;		// TxRDY on, TxEMT on

		case 5:		// Interrupt status register
			return 0x11;	// Channel A TXRDY, Channel B TXRDY

		default:
			return UNIMPLEMENTED_VALUE & 0xFF;
	}
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
		fprintf(stderr, "RD16 %s <%s> 0x%08x ignored, pc=%08X\n",
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
//	} else if ((address == 0x240100) || (address == 0x240101)) {
//		return 0xff;		// FIXME 240101
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


int m68k_irq_callback(int int_level)
{
	// raise 1ms tick interrupt if needed
	if (InterruptFlags.phase_tick)
	{
		InterruptFlags.phase_tick = 0;
		return 0x55;		// TODO check this interrupt number
	}

	// raise UART interrupt if needed
	if (InterruptFlags.uart)
	{
		InterruptFlags.uart = 0;
		return Uart.IVR;
	}

	// if no further pending interrupts, clear the IRQ level
	if (!(InterruptFlags.phase_tick || InterruptFlags.uart)) {
		m68k_set_irq(0);
	}
	return 0;
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

		InterruptFlags.phase_tick = false;		// FIXME DEBUG - phase tick causes crash on boot
		if (InterruptFlags.uart || InterruptFlags.phase_tick) {
			m68k_set_irq(7);
		}

		// TODO: Delay 1/TIMESLOT_FREQUENCY to make this run at real time
		//
		//return 0;
	}

	// Shut down the UART
	UartDone();

	return 0;
}

/***
 * Code gets stuck at pc=23E2 if UART status register returns zero
 *
 * also weirdly if m68k_set_irq(7) is called, we get a spurious IRQ error. herpaderp.
 *
 * TODO's for the UART - when irq mask (IMR) gets TxRdy bit set, make sure to pend an IRQ indicating TX is ready.
 *   uart driver should "pretend" data is sent instantly, UART is always ready
 *   need to use select() to figure out if data is available in the socket before sending RX interrupts though
 *
 * uart terminal cmds:
 *   stty -icanon && ncat -k -l 8888
 *   stty -icanon && ncat -k -l 8889
 */
