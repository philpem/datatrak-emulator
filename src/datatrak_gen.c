#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "datatrak_gen.h"

// Phase measurement zero level
#define PHASE_ZERO 499
// Phase measurement maximum swing
#define PHASE_AMPL 499

// RSSI minimum
#define RSSI_MIN 1
// RSSI maximum
#define RSSI_MAX 255

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

void datatrak_gen_init(DATATRAK_LF_CTX *ctx, const DATATRAK_MODE mode)
{
	switch (mode)
	{
		case DATATRAK_MODE_EIGHTSLOT:
			ctx->numNavslotsPerCycle = 8;
			ctx->numNavslotsTotal = 8;
			break;

		default:
			assert(1==0);
			break;
	}

	// Calculate number of milliseconds per cycle for this Datatrak mode
	ctx->msPerCycle = (340 + (ctx->numNavslotsPerCycle * 80) + 40 + (ctx->numNavslotsPerCycle * 80) + 20);

	// Set initial conditions
	ctx->goldcode_n = 0;
	ctx->clock_n = 12345;

	// Generate trigger templates
#if 0
	// 40ms at 1kHz sample rate = 40 samples
	const double nSamples = 40.0; // 1000.0 * 40.0e-3;
# warning "Generating Trigger from scratch: this currently doesn't work well"
	for (int i=0; i < 40; i++) {
		// 50Hz trigger is 2 cycles of 50Hz
		trig_50_template[i] = trunc(sin((double)(i) / nSamples * M_PI * 2.0 * 2.0) * PHASE_AMPL + PHASE_ZERO);
		// 37.5Hz trigger is 1.5 cycles of 37.5Hz with 180-degree phase offset
		trig_375_template[i] = trunc(sin(((double)(i) / nSamples * M_PI * 2.0 * 1.5) + M_PI) * PHASE_AMPL + PHASE_ZERO);
	}
#else
//# warning "Generating Trigger from rescaled firmware values"
	const double scale = 1.73;	// 1.73 gives the best trigger match quality (705)
								// 500 peak = 289 after scaling, what's going on?
	
	for (int i=0; i < 40; i++) {
		// Re-scale the firmware templates - convert from signed-around-zero to unsigned 0-1000
		ctx->trig50_template[i]  = trunc((double)(DT_TRIG50_TEMPLATE[i])  * scale + PHASE_ZERO);
		ctx->trig375_template[i] = trunc((double)(DT_TRIG375_TEMPLATE[i]) * scale + PHASE_ZERO);
	}
#endif
}

/**
 * Datatrak Gold code. Sent once per cycle in the "trigger" slot.
 * It looks like these words should be the other way around, with the null
 * byte at the end of transmission - but Mk2 expects it to be in the middle.
 * I guess it might be a bug but who knows?
 */
const uint32_t GOLDCODE[] = {0xFA9B8700, 0xAE32BD97};

void datatrak_gen_generate(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf)
{
	uint8_t goldcode_word = (ctx->goldcode_n / 32);
	uint8_t goldcode_bit  = (ctx->goldcode_n % 32);

	// -- Preamble --
	// AA1: 0-40ms (phase=0)
	// Trigger and clock: 5ms gap, 40ms clock, 10ms gap, 20ms clock, 5ms gap
	// Station data: 15ms gap, (20ms dibit, 5ms gap)*2 ==> (120 to 185 ms =  65ms long)
	// Vehicle data: 15ms gap, (20ms dibit, 5ms gap)*4 ==> (185 to 300 ms = 115ms long)
	// AA2: 300-340ms (phase=0)
	//
	// -- F1 Navslots --
	// Navslots F1: start at 340ms, 80ms each (40ms F+, 40ms F-)
	//
	// -- Guard time --
	// G1: 40ms, TX off
	//
	// -- F2 Navslots --
	// Navslots F2 - as F1 navslots but on F2
	//
	// G2: 20ms, TX off

	for (size_t i=0; i<ctx->msPerCycle; i++) {
		// Empty the output buffers to start with -- phase=0 and TX off
		buf->f1_phase[i]     = buf->f2_phase[i]     = PHASE_ZERO;
		buf->f1_amplitude[i] = buf->f2_amplitude[i] = RSSI_MIN;

		if ( (i < 40) ||					//   0 -  40ms: Anti-aliasing 1
			((i >= 40) && (i < 45)) ||		//  40 -  45ms: pre-trigger settling
			((i >= 85) && (i < 95)) ||		//  85 -  95ms: pre-clock settling
			((i >= 115) && (i < 120)) ||	// 115 - 120ms: post-clock settling
					// TODO: Station Data / Vehicle Data settling
			((i >= 300) && (i < 340))		// 300 - 340ms: Anti-aliasing 2
				) {
			// -- 85-
			buf->f1_phase[i] = PHASE_ZERO;
			buf->f1_amplitude[i] = RSSI_MAX;	// FIXME amplitude_max
		} else if ((i >= 45) && (i < 85)) {		// 45-85ms: TRIGGER
			// -- 45 - 85ms: Trigger (Gold Code) --
			if (GOLDCODE[goldcode_word] & (1<<goldcode_bit)) {
				buf->f1_phase[i] = roundf(ctx->trig375_template[i-45] * 1);
			} else {
				buf->f1_phase[i] = roundf(ctx->trig50_template[i-45] * 1);
			}
			buf->f1_amplitude[i] = RSSI_MAX;
		} else if ((i >= 95) && (i < 115)) {
			// -- 85 to 115ms: Clock --

			const float CLOCK_AMPL = 1.0;

			// Clock is sent 2 bits at a time, LSB to MSB
			int bit_n = (ctx->goldcode_n % 8) * 2;
			int bits = (ctx->clock_n >> bit_n) & 3;

			// If we're on the second part of the goldcode, the clock is inverted
			if (ctx->goldcode_n >= 32) {
				bits = bits ^ 3;
			}

			// Convert the clock dibit into a phase offset
			int pha;
			switch(bits) {
				case 0: pha = 0; break;
				case 1: pha = 5; break;
				case 2: pha = 15; break;
				case 3: pha = 10; break;
			}
			buf->f1_phase[i] = roundf((ctx->trig50_template[((i-95)+pha) % 20] * CLOCK_AMPL) + (PHASE_ZERO * (1.0 - CLOCK_AMPL)));
			buf->f1_amplitude[i] = 255;

		// Interlacing means that while stations 1-8 are transmitting on F1, either
		// stations 9-16 (odd cycles) or 17-24 (even cycles) will be transmitting
		// on F2, and vice versa.
		//
		// We can't implement this until we're handling F1/F2 switching.
		//

		} else if ((i >= 340) && (i < 340 + (ctx->numNavslotsPerCycle * 80))) {
			// Navslots (F1)

			// Navslot number (0 to 7 = slot 1 to 8)
			int navslot_n = (i - 340) / 80;
			// Time in the nav slot (0 to 79 ms)
			int time_in_slot = (i - 340) % 80;

			// Each navslot has 40ms of +40Hz, then 40ms of -40Hz frequency offset.
			// We achieve this with phase rotation. One full rotation happens every 25ms.
			// 1000 counts / 25ms = an increment of 40 counts per ms

			// TODO: Allow phase offset for each slot to be set, so we can emulate navigation.
			int slot_phase_ofs_plus  = PHASE_ZERO;
			int slot_phase_ofs_minus = PHASE_ZERO;

			if (time_in_slot < 40) {
				// F1+ slot, phase advance.
				buf->f1_phase[i] = (slot_phase_ofs_plus  + (time_in_slot * 40)) % 1000;
				buf->f1_amplitude[i] = RSSI_MAX;
			} else {
				// F1- slot, phase delay.
				int x = (slot_phase_ofs_minus - ((time_in_slot - 40) * 40));
				while (x < 0) x += 1000;
				buf->f1_phase[i] = x;
				buf->f1_amplitude[i] = RSSI_MAX;
			}

		// After this, 40ms guard1 for frequency switching, then 1-8 tx on F2+/F2- for 8 slots.
		} else if ((i >= (340 /* preamble */ + (ctx->numNavslotsPerCycle * 80) /* F1 slots */ + 40 /* G1 */)) &&
				   (i <  (340 /* preamble */ + (ctx->numNavslotsPerCycle * 80) /* F1 slots */ + 40 /* G1 */ + (ctx->numNavslotsPerCycle * 80))))
		{
			// Navslots (F2)

			// Navslot number (0 to 7 = slot 1 to 8)
			int navslot_n = (i - 340 - (ctx->numNavslotsPerCycle * 80) - 40) / 80;

			// Time in the nav slot (0 to 79 ms)
			int time_in_slot = (i - 340 - (ctx->numNavslotsPerCycle * 80) - 40) % 80;

			// Each navslot has 40ms of +40Hz, then 40ms of -40Hz frequency offset.
			// We achieve this with phase rotation. One full rotation happens every 25ms.
			// 1000 counts / 25ms = an increment of 40 counts per ms

			// TODO: Allow phase offset for each slot to be set, so we can emulate navigation.
			int slot_phase_ofs_plus  = PHASE_ZERO;
			int slot_phase_ofs_minus = PHASE_ZERO;

			if (time_in_slot < 40) {
				// F2+ slot, phase advance.
				buf->f2_phase[i] = (slot_phase_ofs_plus  + (time_in_slot * 40)) % 1000;
				buf->f2_amplitude[i] = RSSI_MAX;
			} else {
				// F2- slot, phase delay.
				int x = (slot_phase_ofs_minus - ((time_in_slot - 40) * 40));
				while (x < 0) x += 1000;
				buf->f2_phase[i] = x;
				buf->f2_amplitude[i] = RSSI_MAX;
			}

		// After this, 20ms guard2 and we're done with the cycle
		} else {
			buf->f1_phase[i] = buf->f2_phase[i] = PHASE_ZERO;
			buf->f1_amplitude[i] = buf->f2_amplitude[i] = RSSI_MIN;
		}
	}

	// advance to next period
	ctx->goldcode_n++;
	if (ctx->goldcode_n == 64) {
		ctx->goldcode_n = 0;
		ctx->clock_n++;
	}
}

void datatrak_gen_dumpRaw(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf, char *filename)
{
	FILE *fp = fopen(filename, "ab");

	// Sample buffer, F1 and F2
	int16_t samp[DATATRAK_BUF_LEN*2];

	for (size_t msec = 0; msec < ctx->msPerCycle; msec++) {
		samp[msec*2 + 0] = ((int)buf->f1_phase[msec] - PHASE_ZERO) * 32;
		samp[msec*2 + 1] = ((int)buf->f2_phase[msec] - PHASE_ZERO) * 32;
	}
	fwrite(samp, sizeof(int16_t), ctx->msPerCycle*2, fp);
	fclose(fp);
}

float phi_f1 = 0, phi_f2 = 0;
void datatrak_gen_dumpModulated(DATATRAK_LF_CTX *ctx, DATATRAK_OUTBUF *buf, char *filename)
{
	FILE *fp = fopen(filename, "ab");

	const double SAMPLERATE = 44100;
	const double FREQUENCY  = 1000;

	const size_t SAMPLES_PER_MS = SAMPLERATE/1000;

	// Phase shift per cycle (to generate the base modulation frequency)
	const double theta = (2.0 * M_PI) * FREQUENCY / SAMPLERATE;

	// Sample buffer
	int16_t samp[SAMPLES_PER_MS*2];

	// Previous phase offset
	int last_ph_f1 = PHASE_ZERO;
	int last_ph_f2 = PHASE_ZERO;

	for (size_t msec = 0; msec < ctx->msPerCycle; msec++) {
		for (size_t s = 0; s < SAMPLES_PER_MS; s++) {
			// calculate phase shift from last cycle to this
			double ph_sh_f1 = (((int)buf->f1_phase[msec] - last_ph_f1) / (double)PHASE_AMPL) * (2.0 * M_PI);
			double ph_sh_f2 = (((int)buf->f2_phase[msec] - last_ph_f2) / (double)PHASE_AMPL) * (2.0 * M_PI);
			last_ph_f1 = buf->f1_phase[msec];
			last_ph_f2 = buf->f2_phase[msec];

			// update phase
			phi_f1 = phi_f1 + theta + ph_sh_f1;
			phi_f2 = phi_f2 + theta + ph_sh_f2;

			// generate sine point
			samp[s*2 + 0] = roundf((16383.0 * (buf->f1_amplitude[msec] / 255.0)) * sin(phi_f1));
			samp[s*2 + 1] = roundf((16383.0 * (buf->f2_amplitude[msec] / 255.0)) * sin(phi_f2));
		}
		fwrite(samp, sizeof(int16_t), SAMPLES_PER_MS*2, fp);
	}
	fclose(fp);
}

