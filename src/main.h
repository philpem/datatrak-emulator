#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

typedef struct {
	bool phase_tick;
	bool uart;
} InterruptFlags_s;

extern volatile InterruptFlags_s InterruptFlags;

#endif // MAIN_H_INCLUDED