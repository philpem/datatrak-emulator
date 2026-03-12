#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

typedef struct {
	bool phase_tick;
	bool uart;
} InterruptFlags_s;

extern volatile InterruptFlags_s InterruptFlags;

void m68k_update_ipl(void);

#endif // MAIN_H_INCLUDED