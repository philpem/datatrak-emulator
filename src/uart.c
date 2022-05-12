/***
 * UART Emulation
 *
 * TODO's for the UART - when irq mask (IMR) gets TxRdy bit set, make sure to pend an IRQ indicating TX is ready.
 *   uart driver should "pretend" data is sent instantly, UART is always ready
 *   need to use select() to figure out if data is available in the socket before sending RX interrupts though
 *
 * uart terminal cmds:
 *   stty -icanon && ncat -k -l 8888
 *   stty -icanon && ncat -k -l 8889
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "m68k.h"

#include "machine.h"
#include "main.h"

#include "uart.h"

#define UART_PORT 10000

uart_s Uart;


void die(char *s)
{
    perror(s);
    exit(1);
}


int UartInit(void)
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

void UartDone(void)
{
	if (Uart.SocketA > 0) {
		close(Uart.SocketA);
	}

	if (Uart.SocketB > 0) {
		close(Uart.SocketB);
	}
}

uint8_t UartRx(void)
{
	uint8_t buf;

	// TODO select()

	if (recv(Uart.SocketA, &buf, 1, 0) < 1) {
		die("Failed to receive byte from TTY");
	}

	return buf;
}


const char *GetUartRegFromAddr(const uint32_t addr, const bool reading)
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

	fprintf(stderr, "[UART WR-8] <%s> 0x%08x => 0x%02x, pc=%08X\n",
			GetUartRegFromAddr(address, false),
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
	uint8_t val;

	switch ((address >> 1) & 0x0F) {
		case 1:		// Status Register A
		case 9:		// Status Register B
			val = 0x0C;		// TxRDY on, TxEMT on, RxRDY off
			break;

		case 5:		// Interrupt status register
			val = 0x11;	// Channel A TXRDY, Channel B TXRDY
			break;

		default:
			val = UNIMPLEMENTED_VALUE & 0xFF;
			break;
	}

	fprintf(stderr, "[UART RD-8] <%s> 0x%08x => 0x%02x, pc=%08X\n",
			GetUartRegFromAddr(address, true),
			address, val, m68k_get_reg(NULL, M68K_REG_PPC));
	return val;
}
