#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

// Telnet IAC (Interpret As Command) byte-stripping state machine
typedef enum {
	IAC_NORMAL,    // normal data
	IAC_AFTER_FF,  // received 0xFF, waiting for command byte
	IAC_AFTER_CMD, // received command, waiting for option byte
	IAC_AFTER_CR   // just passed '\r' to firmware; discard next '\0' (Telnet NVT: CR NUL = bare CR)
} IacState;

typedef struct {
	int ListenA, ListenB;   // server listening sockets (bound to ports)
	int SocketA, SocketB;   // connected client sockets (-1 when none)
	bool TxEnA, TxEnB;
	bool RxEnA, RxEnB;
	bool MRnA, MRnB;
	uint8_t MRA[2];
	uint8_t MRB[2];
	uint8_t IMR;
	uint8_t IVR;
	uint8_t OutPort;
	uint8_t InPort;               // Input port register (IP0-IP6); bit 4 = IP4 = Ignition Sense
	uint8_t RxBufA, RxBufB;      // single-byte RX buffers
	bool    RxReadyA, RxReadyB;  // data-available flags
	IacState IacStateA, IacStateB;
	uint8_t  IacPendingCmdA, IacPendingCmdB;  // buffered IAC command byte
	int      CounterTick;        // counts UartPollRx() calls since last START COUNTER
	bool     CounterReady;       // ISR bit 3: counter/timer reached zero
} uart_s;

extern uart_s Uart;

int UartInit(void);
void UartDone(void);
void UartPollRx(void);
const char *GetUartRegFromAddr(const uint32_t addr, const bool reading);
void UartRegWrite(uint32_t address, uint8_t value);
uint8_t UartRegRead(uint32_t address);

#endif // UART_H_INCLUDED
