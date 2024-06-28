/**************
 * Datatrak LF signal generation
 */

#ifndef _DATATRAK_GEN_H
#define _DATATRAK_GEN_H

#include <stdint.h>

#define DATATRAK_BUF_LEN 1680

typedef enum {
	DATATRAK_MODE_EIGHTSLOT,		///< F1 chain only, 8 slots, no interlacing.
	DATATRAK_MODE_INTERLACED		///< F1 and F2 chain, 24 slots, interlaced.
} DATATRAK_MODE;

typedef struct {
	// -- User configurable parameters (at any time) --
	uint8_t rfNoiseLevel;						///< RF noise level (returned for unmodulated slots)
	uint16_t slotPhaseOffset[24];				///< Slot phase offsets
	uint8_t slotPower[24];						///< Slot transmit power
	uint8_t trig1Power, trig2Power;				///< F1/F2 trigger transmit power

	// -- Calculated per-mode parameters --
	int numNavslotsPerCycle;					///< Number of navslots per cycle (usually 8)
	int numNavslotsTotal;						///< Total number of navslots (including interlacing)
	uint16_t msPerCycle;						///< Number of milliseconds per cycle
	
	// -- Internal state: don't touch! --
	uint16_t trig50_template[40];				///< Trigger 50Hz signal template
	uint16_t trig375_template[40];				///< Trigger 37.5Hz signal template
	int goldcode_n;								///< Current Goldcode offset (0-63)
	int clock_n;								///< Current Clock value (0-65535)
} DATATRAK_LF_CTX;

typedef struct {
	uint16_t f1_phase[DATATRAK_BUF_LEN];		///< F1 phase value 0-999
	uint16_t f2_phase[DATATRAK_BUF_LEN];		///< F2 phase value 0-999
	uint8_t  f1_amplitude[DATATRAK_BUF_LEN];	///< F1 signal strength 0-255
	uint8_t  f2_amplitude[DATATRAK_BUF_LEN];	///< F2 signal strength 0-255
} DATATRAK_OUTBUF;

void datatrak_gen_init(DATATRAK_LF_CTX *ctx, const DATATRAK_MODE mode);
void datatrak_gen_generate(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf);
void datatrak_gen_dumpRaw(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf, char *filename);
void datatrak_gen_dumpModulated(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf, char *filename);

#endif
