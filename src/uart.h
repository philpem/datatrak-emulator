#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

typedef struct {
	int SocketA, SocketB;
	bool TxEnA, TxEnB;
	bool RxEnA, RxEnB;
	bool MRnA, MRnB;
	uint8_t MRA[2];
	uint8_t MRB[2];
	uint8_t IMR;
	uint8_t IVR;
	uint8_t OutPort;
} uart_s;

extern uart_s Uart;

int UartInit(void);
void UartDone(void);
uint8_t UartRx(void);
const char *GetUartRegFromAddr(const uint32_t addr, const bool reading);
void UartRegWrite(uint32_t address, uint8_t value);
uint8_t UartRegRead(uint32_t address);

#endif // UART_H_INCLUDED