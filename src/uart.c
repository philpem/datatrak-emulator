/***
 * UART Emulation
 *
 * SCC68692 dual UART emulation. Each channel listens on a TCP port;
 * connect with 'nc localhost 10000' or 'telnet localhost 10000'.
 * Telnet IAC negotiation bytes are stripped automatically.
 */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "m68k.h"

#include "machine.h"
#include "main.h"

#include "uart.h"


// TCP ports for the two UART channels
#define UART_PORT_A 10000
#define UART_PORT_B 10001

// Define to enable full register r/w debug messages (very verbose during TX)
// #define UART_DEBUG_MSGS

// Define to enable key state-change messages only (RX arrivals, IMR/CRA changes,
// ISR reads) -- much less noisy than UART_DEBUG_MSGS, good for diagnosing hangs
// #define UART_DEBUG_KEY

// Log state changes of the UART output port
// #define LOG_UART_OUTPORT



uart_s Uart;


static void die(char *s)
{
    perror(s);
    exit(1);
}

// Close a client socket and reset to -1
static void UartClientClose(int *sockfd)
{
	if (*sockfd >= 0) {
		close(*sockfd);
		*sockfd = -1;
	}
}

// Create a non-blocking TCP listening socket bound to the given port.
static int make_listen_socket(int port)
{
	int fd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (fd < 0) die("socket");

	int one = 1;
	if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0)
		die("setsockopt SO_REUSEADDR");

	if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
		die("fcntl O_NONBLOCK");

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family      = AF_INET;
	sa.sin_addr.s_addr = INADDR_ANY;
	sa.sin_port        = htons(port);

	if (bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0)
		die("bind");

	if (listen(fd, 1) < 0)
		die("listen");

	return fd;
}


int UartInit(void)
{
	memset(&Uart, '\0', sizeof(Uart));

	Uart.TxEnA = Uart.TxEnB = false;
	Uart.RxEnA = Uart.RxEnB = false;
	Uart.MRnA  = Uart.MRnB  = false;

	// Default interrupt vector on reset
	Uart.IVR = 0x0F;

	// No clients connected yet
	Uart.SocketA = Uart.SocketB = -1;
	Uart.RxReadyA = Uart.RxReadyB = false;
	Uart.IacStateA = Uart.IacStateB = IAC_NORMAL;

	// Input port: IP4 = Ignition Sense (1 = ignition on)
	Uart.InPort = (1 << 4);

	// Counter/Timer: starts not-ready; only begins ticking once the firmware
	// issues its first START COUNTER read (UartRegRead case 14).
	Uart.CounterTick    = 0;
	Uart.CounterReady   = false;
	Uart.CounterStarted = false;

	// Create listening sockets
	Uart.ListenA = make_listen_socket(UART_PORT_A);
	fprintf(stderr, "UART_A listening on port %d\n", UART_PORT_A);

	Uart.ListenB = make_listen_socket(UART_PORT_B);
	fprintf(stderr, "UART_B listening on port %d\n", UART_PORT_B);

	return 0;
}


void UartDone(void)
{
	UartClientClose(&Uart.SocketA);
	UartClientClose(&Uart.SocketB);
	if (Uart.ListenA >= 0) close(Uart.ListenA);
	if (Uart.ListenB >= 0) close(Uart.ListenB);
}


// Filter one incoming byte through the telnet IAC state machine.
// Returns true and writes *out if the byte should be passed to the firmware.
// Returns false if the byte is part of an IAC sequence (discard it).
// Also sends WONT/DONT responses when the telnet client offers options.
static bool UartFilterByte(int sockfd, uint8_t byte, IacState *state,
                            uint8_t *pending_cmd, uint8_t *out)
{
	switch (*state) {
		case IAC_NORMAL:
			if (byte == 0xFF) {
				*state = IAC_AFTER_FF;
				return false;
			}
			if (byte == '\r') {
				// Hold off: CR NUL -> output CR; CR LF -> output LF
				// (matching nc's single-LF behavior the firmware already accepts)
				*state = IAC_AFTER_CR;
				return false;  // swallow CR for now; decide on next byte
			}
			*out = byte;
			return true;

		case IAC_AFTER_CR:
			// Resolve the held CR based on the following byte:
			//   CR NUL  -> output CR  (bare CR on Telnet NVT)
			//   CR LF   -> output LF  (matches nc's Enter; firmware expects LF)
			//   CR CR   -> output CR, stay in IAC_AFTER_CR for the second CR
			//   CR IAC  -> output CR, begin IAC sequence
			//   CR else -> output CR  (edge case)
			*state = IAC_NORMAL;
			if (byte == '\0') { *out = '\r'; return true; }  // CR NUL -> CR
			if (byte == '\n') { *out = '\n'; return true; }  // CR LF  -> LF
			if (byte == '\r') { *state = IAC_AFTER_CR; *out = '\r'; return true; }
			if (byte == 0xFF) { *state = IAC_AFTER_FF; return false; }
			// CR followed by something else: output CR, discard the other byte
			*out = '\r';
			return true;

		case IAC_AFTER_FF:
			if (byte == 0xFF) {
				// Escaped literal 0xFF
				*state = IAC_NORMAL;
				*out = 0xFF;
				return true;
			}
			if (byte == 0xFB || byte == 0xFD) {
				// WILL or DO — we will respond WONT/DONT after seeing the option
				*pending_cmd = byte;
				*state = IAC_AFTER_CMD;
				return false;
			}
			if (byte == 0xFC || byte == 0xFE) {
				// WONT or DONT — no response needed, just eat the option byte
				*pending_cmd = 0;
				*state = IAC_AFTER_CMD;
				return false;
			}
			// SE, NOP, or other single-byte commands
			*state = IAC_NORMAL;
			return false;

		case IAC_AFTER_CMD:
			// This byte is the option code
			if (*pending_cmd == 0xFB) {
				// Client sent WILL <opt> — respond IAC DONT <opt>
				uint8_t resp[3] = { 0xFF, 0xFE, byte };
				send(sockfd, resp, 3, MSG_NOSIGNAL);
			} else if (*pending_cmd == 0xFD) {
				// Client sent DO <opt> — respond IAC WONT <opt>
				uint8_t resp[3] = { 0xFF, 0xFC, byte };
				send(sockfd, resp, 3, MSG_NOSIGNAL);
			}
			*state = IAC_NORMAL;
			return false;
	}

	// unreachable
	return false;
}


// Try to accept a new client on the given listening socket.
// Sets *client_sock, *iac_state on success.
static void try_accept(int listen_sock, int *client_sock,
                        IacState *iac_state, const char *name)
{
	if (*client_sock >= 0) return;  // already connected

	int fd = accept(listen_sock, NULL, NULL);
	if (fd < 0) return;  // EAGAIN/EWOULDBLOCK — no pending connection

	// Do NOT set O_NONBLOCK on the accepted socket — send() must remain
	// blocking so TX never sees EAGAIN and inadvertently closes the connection
	// when the firmware bursts output and fills the TCP send buffer.
	// recv() is kept non-blocking via MSG_DONTWAIT in UartPollRx().

	*client_sock = fd;
	*iac_state   = IAC_NORMAL;
	fprintf(stderr, "%s: client connected\n", name);
}


void UartPollRx(void)
{
	// --- Channel A ---
	try_accept(Uart.ListenA, &Uart.SocketA, &Uart.IacStateA, "UART_A");

	if (Uart.SocketA >= 0 && Uart.RxEnA && !Uart.RxReadyA) {
		uint8_t raw;
		int n = recv(Uart.SocketA, &raw, 1, MSG_DONTWAIT);
		if (n == 1) {
			uint8_t filtered;
			if (UartFilterByte(Uart.SocketA, raw, &Uart.IacStateA,
			                   &Uart.IacPendingCmdA, &filtered)) {
				Uart.RxBufA   = filtered;
				Uart.RxReadyA = true;
#ifdef UART_DEBUG_KEY
				fprintf(stderr, "[UART_A RX] byte=0x%02X '%c'  IMR=0x%02X\n",
				        filtered, (filtered >= 0x20 && filtered < 0x7F) ? filtered : '.',
				        Uart.IMR);
#endif
				if (Uart.IMR & 0x02)   // RxRdy/FFullA
					InterruptFlags.uart = true;
			}
		} else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
			UartClientClose(&Uart.SocketA);
			fprintf(stderr, "UART_A: client disconnected\n");
		}
	}

	// --- Channel B ---
	try_accept(Uart.ListenB, &Uart.SocketB, &Uart.IacStateB, "UART_B");

	if (Uart.SocketB >= 0 && Uart.RxEnB && !Uart.RxReadyB) {
		uint8_t raw;
		int n = recv(Uart.SocketB, &raw, 1, MSG_DONTWAIT);
		if (n == 1) {
			uint8_t filtered;
			if (UartFilterByte(Uart.SocketB, raw, &Uart.IacStateB,
			                   &Uart.IacPendingCmdB, &filtered)) {
				Uart.RxBufB   = filtered;
				Uart.RxReadyB = true;
				if (Uart.IMR & 0x20)   // RxRdy/FFullB
					InterruptFlags.uart = true;
			}
		} else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
			UartClientClose(&Uart.SocketB);
			fprintf(stderr, "UART_B: client disconnected\n");
		}
	}

	// --- Counter/Timer ---
	// Fire CounterReady (ISR bit 3) every ~10ms (~10 UartPollRx calls).
	// Only counts after the firmware has issued its first START COUNTER read,
	// so we never fire a spurious interrupt before the firmware sets up the timer.
	if (Uart.CounterStarted && !Uart.CounterReady) {
		Uart.CounterTick++;
		if (Uart.CounterTick >= 10) {
			Uart.CounterTick  = 0;
			Uart.CounterReady = true;
			if (Uart.IMR & 0x08)   // CounterReady interrupt enabled
				InterruptFlags.uart = true;
		}
	}
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
#ifdef UART_DEBUG_MSGS
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
#endif

	switch ((address / 2) & 0x0F) {
		case 0:		// Mode Register 1A / 2A (pointer auto-advances)
			Uart.MRA[Uart.MRnA ? 1 : 0] = value;
			Uart.MRnA = !Uart.MRnA;
			break;

		case 8:		// Mode Register 1B / 2B
			Uart.MRB[Uart.MRnB ? 1 : 0] = value;
			Uart.MRnB = !Uart.MRnB;
			break;

		case 2:		// Command Register A
#ifdef UART_DEBUG_MSGS
			printf("UART CRA -->  RxEn %s  TxEn %s  Cmd:%s\n",
					ENDIS[value & 0x03],
					ENDIS[(value >> 2) & 0x03],
					CMDS[(value >> 4) & 0x0F]);
#endif
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

			#ifdef UART_DEBUG_KEY
			if ((value >> 4) & 0x0F)
				fprintf(stderr, "[UART CRA] cmd=0x%X  rx_bits=%d tx_bits=%d  RxEn:%d TxEn:%d  pc=%08X\n",
				        (value >> 4) & 0x0F, value & 0x03, (value >> 2) & 0x03,
						Uart.RxEnA, Uart.TxEnA,
						m68k_get_reg(NULL, M68K_REG_PPC));
#endif
			switch ((value >> 4) & 0x0F) {
				case 0:			// null command
					break;

				case 1:			// Reset MRn pointer
					Uart.MRnA = false;
					break;

				case 2:			// Reset receiver
					// Flushes RX FIFO and clears status bits.
					// Does NOT change receiver enabled/disabled state.
					Uart.RxReadyA = false;
					break;

				case 3:			// Reset transmitter
					// Flushes TX FIFO. Does NOT change transmitter enabled state.
					// Fall through -- no state to clear in this emulation.

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
#ifdef UART_DEBUG_KEY
			fprintf(stderr, "[UART THRA write] byte=0x%02X '%c'  pc=%08X\n",
			        value, (value >= 0x20 && value < 0x7F) ? value : '.',
			        m68k_get_reg(NULL, M68K_REG_PPC));
#endif
#ifdef UART_DEBUG_MSGS
			printf("UARTA --> %c  [%02x]\n", value, value);
#endif
			if (Uart.SocketA >= 0) {
				if (send(Uart.SocketA, &value, 1, MSG_NOSIGNAL) != 1)
					UartClientClose(&Uart.SocketA);
			}

			// If TxRdyA interrupt is enabled, pend a TX IRQ and
			// immediately update the CPU IPL so it fires promptly.
			if (Uart.IMR & 0x01) {
				InterruptFlags.uart = true;
				m68k_update_ipl();
			}
			break;


		case 5:			// Interrupt mask register
			Uart.IMR = value;
#ifdef UART_DEBUG_KEY
			fprintf(stderr, "[UART IMR write] IMR=0x%02X  RxA=%d RxB=%d  pc=%08X\n",
			        Uart.IMR, Uart.RxReadyA, Uart.RxReadyB,
			        m68k_get_reg(NULL, M68K_REG_PPC));
#endif

#ifdef UART_DEBUG_MSGS
			fprintf(stderr, "UART IMR = %02X  --> ", value);
			if (Uart.IMR & 0x80) fprintf(stderr, "InPortChng ");
			if (Uart.IMR & 0x40) fprintf(stderr, "DeltaBrkB ");
			if (Uart.IMR & 0x20) fprintf(stderr, "RxRdy/FFullB ");
			if (Uart.IMR & 0x10) fprintf(stderr, "TxRdyB ");
			if (Uart.IMR & 0x08) fprintf(stderr, "CounterReady ");
			if (Uart.IMR & 0x04) fprintf(stderr, "DeltaBrkA ");
			if (Uart.IMR & 0x02) fprintf(stderr, "RxRdy/FFullA ");
			if (Uart.IMR & 0x01) fprintf(stderr, "TxRdyA ");
			fprintf(stderr, "\n");
#endif

			// Pend interrupt if any enabled condition is already asserted:
			// TX always ready (buffer empty), RX has data, or CounterReady still set.
			if ((Uart.IMR & 0x01) || (Uart.IMR & 0x10) ||
			    ((Uart.IMR & 0x02) && Uart.RxReadyA) ||
			    ((Uart.IMR & 0x20) && Uart.RxReadyB) ||
			    ((Uart.IMR & 0x08) && Uart.CounterReady)) {
				InterruptFlags.uart = true;
			}
			break;


		case 10:		// Command Register B
#ifdef UART_DEBUG_MSGS
			printf("UART CRB -->  RxEn %s  TxEn %s  Cmd:%s\n",
					ENDIS[value & 0x03],
					ENDIS[(value >> 2) & 0x03],
					CMDS[(value >> 4) & 0x0F]);
#endif
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
					Uart.RxReadyB = false;
					break;

				case 3:			// Reset transmitter
					// Fall through -- no state to clear in this emulation.

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
#ifdef UART_DEBUG_MSGS
			printf("UARTB --> %c  [%02x]\n", value, value);
#endif
			if (Uart.SocketB >= 0) {
				if (send(Uart.SocketB, &value, 1, MSG_NOSIGNAL) != 1)
					UartClientClose(&Uart.SocketB);
			}

			// If TxRdyB interrupt is enabled, pend a TX IRQ and
			// immediately update the CPU IPL so it fires promptly.
			if (Uart.IMR & 0x10) {
				InterruptFlags.uart = true;
				m68k_update_ipl();
			}
			break;


		case 12:	// Interrupt vector register
			Uart.IVR = value;
#ifdef UART_DEBUG_MSGS
			printf("UART Int Vec = 0x%02X\n", Uart.IVR);
#endif
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
		case 0:		// Mode Register 1A / 2A (pointer auto-advances on read)
			val = Uart.MRA[Uart.MRnA ? 1 : 0];
			Uart.MRnA = !Uart.MRnA;
			break;

		case 1:		// Status Register A: bit0=RxRDY, bit2=TxEMT, bit3=TxRDY
			val = 0x0C | (Uart.RxReadyA ? 0x01 : 0);
			break;

		case 3:		// Receive Holding Register A
			val = Uart.RxBufA;
			Uart.RxReadyA = false;
#ifdef UART_DEBUG_KEY
			fprintf(stderr, "[UART RHRA read] byte=0x%02X '%c'  pc=%08X\n",
			        val, (val >= 0x20 && val < 0x7F) ? val : '.',
			        m68k_get_reg(NULL, M68K_REG_PPC));
#endif
			break;

		case 4:		// IPCR — Input Port Change Register (no pending change events)
			val = 0x00;
			break;

		case 5:		// Interrupt Status Register
			// TxRdyA (bit 0) and TxRdyB (bit 4) are unconditionally set: the TX
			// holding register is always empty in this emulation (we send immediately
			// via TCP), matching real SCC68692 hardware where ISR is NOT masked by IMR.
			// RxRdy bits reflect actual receive buffer state.
			val = 0x11;                           // TxRdyA | TxRdyB always set
			if (Uart.RxReadyA)    val |= 0x02;    // RxRdy/FFullA
			if (Uart.CounterReady) val |= 0x08;   // CounterReady
			if (Uart.RxReadyB)    val |= 0x20;    // RxRdy/FFullB
#ifdef UART_DEBUG_KEY
			fprintf(stderr, "[UART ISR read] ISR=0x%02X  IMR=0x%02X  RxA=%d RxB=%d  pc=%08X\n",
			        val, Uart.IMR, Uart.RxReadyA, Uart.RxReadyB,
			        m68k_get_reg(NULL, M68K_REG_PPC));
#endif
			break;

		case 8:		// Mode Register 1B / 2B
			val = Uart.MRB[Uart.MRnB ? 1 : 0];
			Uart.MRnB = !Uart.MRnB;
			break;

		case 9:		// Status Register B
			val = 0x0C | (Uart.RxReadyB ? 0x01 : 0);
			break;

		case 11:	// Receive Holding Register B
			val = Uart.RxBufB;
			Uart.RxReadyB = false;
			break;

		case 13:	// IP0-6 — Input Port register
			val = Uart.InPort;
			break;

		case 14:	// START COUNTER — arms/re-arms the counter/timer, clears CounterReady
			Uart.CounterStarted = true;
			Uart.CounterReady   = false;
			Uart.CounterTick    = 0;
			val = 0x00;
#ifdef UART_DEBUG_KEY
			fprintf(stderr, "[UART START COUNTER] CounterReady cleared, re-armed  pc=%08X\n",
			        m68k_get_reg(NULL, M68K_REG_PPC));
#endif
			break;

	default:
			val = UNIMPLEMENTED_VALUE & 0xFF;
			break;
	}

#ifdef UART_DEBUG_MSGS
	fprintf(stderr, "[UART RD-8] <%s> 0x%08x => 0x%02x, pc=%08X\n",
			GetUartRegFromAddr(address, true),
			address, val, m68k_get_reg(NULL, M68K_REG_PPC));
#endif
	return val;
}
