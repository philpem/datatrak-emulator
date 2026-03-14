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

// Trigger templates from the Datatrak firmware

/**
 * 50Hz Trigger/Clock template
 */
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

/**
 * 37.5Hz Trigger/Clock template
 */
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

/**
 * Datatrak Gold code. Sent once per cycle in the "trigger" slot.
 * It looks like these words should be the other way around, with the null
 * byte at the end of transmission - but Mk2 expects it to be in the middle.
 * I guess it might be a bug but who knows?
 */
const uint32_t GOLDCODE[] = {0xFA9B8700, 0xAE32BD97};

/*
 * Trigger synthesis
 * =================
 *
 * Generates a pure-sine waveform whose parameters (amplitude A, starting
 * phase φ) are chosen so that after passing through the receiver's software
 * IIR filter (getPhaseMeas, firmware 0x97CE), the filter's output matches
 * the firmware's stored 40-sample trigger templates as closely as possible.
 *
 * Signal path from generator to SAD comparator:
 *
 *   gen_trigger() → [IIR bandpass filter, 0x97CE] → SAD → lock decision
 *
 *
 * Derivation of A and φ (three steps; see derive_trigger_params.c)
 * ----------------------------------------------------------------
 *
 * Step 1 — Template DFT
 *   Fit A*sin(2πfn/Fs + φ) to the firmware template using a single-bin
 *   DFT.  This gives the amplitude and phase the IIR is expected to output
 *   at steady state.
 *
 *     50Hz template:    A_tmpl ≈ 253.8,  φ_tmpl = +70.45°
 *     37.5Hz template:  A_tmpl ≈ 240.6,  φ_tmpl = −118.21°
 *
 * Step 2 — LTI steady-state IIR correction
 *   The IIR (H(z) = (65/256)(1−z⁻¹) / [(1−(13/16)z⁻¹)(1−(11/16)z⁻¹)])
 *   attenuates the input by |H| ≈ 0.577 and phase-advances the output by
 *   ∠H at both trigger frequencies.  Working backwards:
 *
 *     A_lti   = A_tmpl / |H|
 *     φ_lti   = φ_tmpl − ∠H
 *
 *   IIR frequency response at trigger frequencies:
 *
 *     Frequency    |H|     ∠H (advance)   Group delay
 *     50 Hz        0.5776   +1.61°          2.58 ms
 *     37.5 Hz      0.5728  +15.32°          3.57 ms
 *
 * Step 3 — Numerical startup-transient correction
 *   The IIR starts each trigger window cold (settled on the unmodulated
 *   carrier).  The large-step guard in the firmware fires on the first
 *   sample of the trigger waveform, kicking iir1 by ±5333 counts.  This
 *   transient takes ~5 samples to decay and shifts the apparent phase of
 *   the waveform as seen by the SAD correlator.  A coarse-then-fine
 *   numerical search (A: 300–600 in 0.5-count steps; φ: ±180° in 0.1°
 *   steps) over the exact integer IIR model finds the global minimum SAD.
 *
 *   Combined phase corrections:
 *
 *     Freq     φ_tmpl    − ∠H      − Δφ_transient  = φ_opt
 *     50Hz:   +70.45°  −  1.61°  −  50.63°        = +18.20°
 *     37.5Hz: −118.21° − 15.32°  −  32.66°        = −166.20°
 *
 *   Both results can also be expressed as small deviations from the
 *   transmitter's nominal ideal phases:
 *
 *     50Hz:    φ_opt = 0°   + 18.2°   (carrier starts 18° ahead of ideal)
 *     37.5Hz:  φ_opt = 180° − 13.8°   (carrier starts 14° ahead of ideal)
 *
 *
 * Why are both corrections similar (~14–18°)?
 * -------------------------------------------
 * The firmware templates were captured from a live receiver, not from a
 * simulation.  The receiver's IF bandpass filter (a two-section stagger-
 * tuned LC filter, f₁=21.1 kHz, f₂=19.2 kHz) introduces a group delay of
 * τ ≈ 1.017 ms at the passband peak (~20.9 kHz).  This delay is equivalent
 * to the phase advance seen at modulation frequencies:
 *
 *     τ = φ / (360° × f)
 *     50Hz:    18.2° / (360° × 50)   × 1000 = 1.011 ms  ⎤
 *     37.5Hz:  13.8° / (360° × 37.5) × 1000 = 1.022 ms  ⎦  mean: 1.017 ms
 *
 * The corrections are therefore not arbitrary — they encode the group delay
 * of the receiver's hardware IF filter, which the simulator lacks.
 *
 * IMPORTANT — deployment dependency:
 *   These phase constants are correct for direct injection to the receiver's
 *   phase-measurement input (bypassing the IF).  If the signal is instead
 *   transmitted over RF and received through a real antenna and IF chain, use
 *   ideal phases 0° and 180°: the IF filter will apply the ~1 ms delay
 *   naturally.  Using the pre-corrected phases through a real IF would double
 *   the correction and degrade lock performance.
 *
 *
 * Achieved SAD scores (lower = better; lock threshold = 1000, target ≤ 500):
 *   50Hz:    493  (hardware: 493, exact match)
 *   37.5Hz:  392  (hardware: 394, 2-count rounding)
 */
static const double PHI_50HZ_MK2  =   18.20 * (M_PI / 180.0);  /* 0°   + 18.2° IF-filter correction */
static const double PHI_375HZ_MK2 = -166.20 * (M_PI / 180.0);  /* 180° − 13.8° IF-filter correction */
static const double A_TRIG        = 450.0;                     /* pre-IIR amplitude (IIR outputs ~254) */

// Ideal phase values
static const double PHI_50HZ_IDEAL  =    0.0 * (M_PI / 180.0);  /* 0°   */
static const double PHI_375HZ_IDEAL = -180.0 * (M_PI / 180.0);  /* 180° */

/*
 * gen_trigger - write one trigger waveform into out[0..pre_len+39]
 *
 *   out     - output buffer, must hold (pre_len + 40) uint16_t values
 *   f_hz    - trigger frequency (50.0 or 37.5)
 *   phi_rad - starting phase (PHI_50HZ or PHI_375HZ)
 *   pre_len - samples of continuous sine written before the 40-sample FTS
 *             window (n = -pre_len .. -1).  The caller places these in the
 *             pre-trigger gap immediately before the trigger slot so that
 *             the IIR enters the window already tracking the waveform.
 *             Set to 0 for a cold-start trigger (current usage; SAD 493/392).
 *
 * Note: no clamping to [PHASE_MIN, PHASE_MAX] is applied.  With A_TRIG=450
 * and PHASE_ZERO=499 the output spans [49, 949], well within [0, 999].
 * Restore the clamp if A_TRIG is ever increased beyond ~500.
 */
static void gen_trigger(uint16_t *out, double f_hz, double phi_rad, int pre_len)
{
    for (int n = -pre_len; n < 40; n++) {
        double v = A_TRIG * sin(2.0 * M_PI * f_hz * n / 1000.0 + phi_rad) + PHASE_ZERO;
        out[n + pre_len] = (uint16_t)round(v);
    }
}

void datatrak_gen_init(DATATRAK_LF_CTX *ctx, const DATATRAK_MODE mode, const DATATRAK_COMPENSATION comp)
{
	switch (mode)
	{
		case DATATRAK_MODE_EIGHTSLOT:
			ctx->numNavslotsPerCycle = 8;
			ctx->numNavslotsTotal = 8;
			break;

		case DATATRAK_MODE_INTERLACED:
			ctx->numNavslotsPerCycle = 8;
			ctx->numNavslotsTotal = 24;
			break;

		default:
			assert(1==0);
			break;
	}

	ctx->mode = mode;

	// Calculate number of milliseconds per cycle for this Datatrak mode
	ctx->msPerCycle = (340 + (ctx->numNavslotsPerCycle * 80) + 40 + (ctx->numNavslotsPerCycle * 80) + 20);

	// Set initial conditions
	ctx->goldcode_n = 0;
	ctx->clock_n = 0;
	for (size_t i=0; i<ctx->numNavslotsTotal; i++) {
		ctx->slotPhaseOffset[i] = 0;
		ctx->slotPower[i] = DATATRAK_RSSI_MIN;
	}

	// Set up IF-strip compensation
	ctx->compensation = comp;
	double phi50, phi375;
	switch(comp) {
		case DATATRAK_COMPENSATION_NONE:
			// No compensation, generate signals for transmission
			phi50  = PHI_50HZ_IDEAL;
			phi375 = PHI_375HZ_IDEAL;
			break;

		case DATATRAK_COMPENSATION_MK2:
			// Compensation for Mk2 Locator IF strip
			phi50  = PHI_50HZ_MK2;
			phi375 = PHI_375HZ_MK2;
			break;

		default:
			assert(1==0);
			break;
	}

	// Generate trigger templates
	gen_trigger(ctx->trig50_template,  50,   phi50,  0);
	gen_trigger(ctx->trig375_template, 37.5, phi375, 0);
}

/**
 * Wrap phase measurement into 0..999 range.
 *
 * This is used when a phase calculation is done, to make sure the phase
 * measurement mirrors what the Datatrak hardware would return.
 *
 * It starts by dividing by 1000, and retaining the modulus and sign.
 * If the result is negative, 1000 is added to wrap it around.
 */
static inline int phaseWrap(const int x)
{
	int xm = x % 1000;
	if (xm < 0) {
		xm += 1000;
	}
	return xm;
}

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
		buf->f1_amplitude[i] = buf->f2_amplitude[i] = DATATRAK_RSSI_MIN;

		// The antialiasing slots and trigger/clock settling need to be modulated
		// with zero phase by the chain master.
		// The IIR filter in the receiver uses them to eliminate slow phase
		// wander caused by drift between the base station and the Locator's
		// TCXO clock.

		if ( (i < 40) ||					//   0 -  40ms: Anti-aliasing 1
			((i >= 40) && (i < 45)) ||		//  40 -  45ms: pre-trigger settling
			((i >= 85) && (i < 95)) ||		//  85 -  95ms: pre-clock settling
			((i >= 115) && (i < 120)) ||	// 115 - 120ms: post-clock settling
					// TODO: Station Data / Vehicle Data settling
			((i >= 300) && (i < 340))		// 300 - 340ms: Anti-aliasing 2
				) {
			// -- 85-
			buf->f1_phase[i] = PHASE_ZERO;
			buf->f1_amplitude[i] = DATATRAK_RSSI_MAX;	// FIXME amplitude_max
		} else if ((i >= 45) && (i < 85)) {		// 45-85ms: TRIGGER
			// -- 45 - 85ms: Trigger (Gold Code) --
			if (GOLDCODE[goldcode_word] & (1<<goldcode_bit)) {
				buf->f1_phase[i] = ctx->trig375_template[i-45];
			} else {
				buf->f1_phase[i] = ctx->trig50_template[i-45];
			}
			buf->f1_amplitude[i] = DATATRAK_RSSI_MAX;
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
			buf->f1_amplitude[i] = DATATRAK_RSSI_MAX;	// TODO: trigger_rssi

		// Interlacing means that while stations 1-8 are transmitting on F1, either
		// stations 9-16 (odd cycles) or 17-24 (even cycles) will be transmitting
		// on F2, and vice versa.
		//
		// We can't implement this until we're handling F1/F2 switching.
		//

		} else if ((i >= 340) && (i < 340 + (ctx->numNavslotsPerCycle * 80))) {
			// Navslots (F1 and Interlaced F2)

			// Navslot number (0 to 7 = slot 1 to 8)
			int navslot_n = (i - 340) / 80;
			// Time in the nav slot (0 to 79 ms)
			int time_in_slot = (i - 340) % 80;

			// Each navslot has 40ms of +40Hz, then 40ms of -40Hz frequency offset.
			// We achieve this with phase rotation. One full rotation happens every 25ms.
			// 1000 counts / 25ms = an increment of 40 counts per ms

			int slot_phase_ofs = PHASE_ZERO + ctx->slotPhaseOffset[navslot_n];

			if (time_in_slot < 40) {
				// F1+ slot, phase advance.
				if (ctx->slotPower[navslot_n] > DATATRAK_RSSI_MIN) {
					buf->f1_phase[i] = phaseWrap(slot_phase_ofs + (time_in_slot * 40));
				} else {
					buf->f1_phase[i] = PHASE_ZERO;
				}
				buf->f1_amplitude[i] = ctx->slotPower[navslot_n];
			} else {
				// F1- slot, phase delay.
				if (ctx->slotPower[navslot_n] > DATATRAK_RSSI_MIN) {
					buf->f1_phase[i] = phaseWrap(slot_phase_ofs - ((time_in_slot - 40) * 40));
				} else {
					buf->f1_phase[i] = PHASE_ZERO;
				}
				buf->f1_amplitude[i] = ctx->slotPower[navslot_n];
			}

			// Interlaced mode? If so, generate F2 interlaced slots.
			if (ctx->mode == DATATRAK_MODE_INTERLACED) {
				int ilslot_n = navslot_n + (ctx->goldcode_n & 1 ? 16 : 8);
				int il_phase_ofs = PHASE_ZERO + ctx->slotPhaseOffset[ilslot_n];

				if (time_in_slot < 40) {
					// IL F2+ slot, phase advance.
					if (ctx->slotPower[ilslot_n] > DATATRAK_RSSI_MIN) {
						buf->f2_phase[i] = phaseWrap(il_phase_ofs + (time_in_slot * 40));
					} else {
						buf->f2_phase[i] = PHASE_ZERO;
					}
					buf->f2_amplitude[i] = ctx->slotPower[ilslot_n];
				} else {
					// IL F2- slot, phase delay.
					if (ctx->slotPower[ilslot_n] > DATATRAK_RSSI_MIN) {
						buf->f2_phase[i] = phaseWrap(il_phase_ofs - ((time_in_slot - 40) * 40));
					} else {
						buf->f2_phase[i] = PHASE_ZERO;
					}
					buf->f2_amplitude[i] = ctx->slotPower[ilslot_n];
				}
			}

		// After this, 40ms guard1 for frequency switching, then 1-8 tx on F2+/F2- for 8 slots.
		} else if ((i >= (340 /* preamble */ + (ctx->numNavslotsPerCycle * 80) /* F1 slots */ + 40 /* G1 */)) &&
				   (i <  (340 /* preamble */ + (ctx->numNavslotsPerCycle * 80) /* F1 slots */ + 40 /* G1 */ + (ctx->numNavslotsPerCycle * 80))))
		{
			// Navslots (F2 and Interlaced F1)

			// Navslot number (0 to 7 = slot 1 to 8)
			int navslot_n = (i - 340 - (ctx->numNavslotsPerCycle * 80) - 40) / 80;

			// Time in the nav slot (0 to 79 ms)
			int time_in_slot = (i - 340 - (ctx->numNavslotsPerCycle * 80) - 40) % 80;

			// Each navslot has 40ms of +40Hz, then 40ms of -40Hz frequency offset.
			// We achieve this with phase rotation. One full rotation happens every 25ms.
			// 1000 counts / 25ms = an increment of 40 counts per ms

			int slot_phase_ofs = PHASE_ZERO + ctx->slotPhaseOffset[navslot_n];

			if (time_in_slot < 40) {
				// F2+ slot, phase advance.
				if (ctx->slotPower[navslot_n] > DATATRAK_RSSI_MIN) {
					buf->f2_phase[i] = phaseWrap(slot_phase_ofs + (time_in_slot * 40));
				} else {
					buf->f2_phase[i] = PHASE_ZERO;
				}
				buf->f2_amplitude[i] = ctx->slotPower[navslot_n];
			} else {
				// F2- slot, phase delay.
				if (ctx->slotPower[navslot_n] > DATATRAK_RSSI_MIN) {
					buf->f2_phase[i] = phaseWrap(slot_phase_ofs - ((time_in_slot - 40) * 40));
				} else {
					buf->f2_phase[i] = PHASE_ZERO;
				}
				buf->f2_amplitude[i] = ctx->slotPower[navslot_n];
			}

			// Interlaced mode? If so, generate F1 interlaced slots.
			if (ctx->mode == DATATRAK_MODE_INTERLACED) {
				int ilslot_n = navslot_n + (ctx->goldcode_n & 1 ? 8 : 16);
				int il_phase_ofs = PHASE_ZERO + ctx->slotPhaseOffset[ilslot_n];

				if (time_in_slot < 40) {
					// IL F1+ slot, phase advance.
					if (ctx->slotPower[ilslot_n] > DATATRAK_RSSI_MIN) {
						buf->f1_phase[i] = phaseWrap(il_phase_ofs + (time_in_slot * 40));
					} else {
						buf->f1_phase[i] = PHASE_ZERO;
					}
					buf->f1_amplitude[i] = ctx->slotPower[ilslot_n];
				} else {
					// IL F1- slot, phase delay.
					if (ctx->slotPower[ilslot_n] > DATATRAK_RSSI_MIN) {
						buf->f1_phase[i] = phaseWrap(il_phase_ofs - ((time_in_slot - 40) * 40));
					} else {
						buf->f1_phase[i] = PHASE_ZERO;
					}
					buf->f1_amplitude[i] = ctx->slotPower[ilslot_n];
				}
			}

		// After this, 20ms guard2 and we're done with the cycle
		} else {
			buf->f1_phase[i] = buf->f2_phase[i] = 238;
			buf->f1_amplitude[i] = buf->f2_amplitude[i] = DATATRAK_RSSI_MIN;
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
		// Calculate the per-sample phase contribution from the ms-level phase delta.
		// ±PHASE_AMPL counts = ±π radians (half-cycle); distribute evenly across samples.
		double ph_sh_f1 = (((int)buf->f1_phase[msec] - last_ph_f1) / (double)PHASE_AMPL) * M_PI / SAMPLES_PER_MS;
		double ph_sh_f2 = (((int)buf->f2_phase[msec] - last_ph_f2) / (double)PHASE_AMPL) * M_PI / SAMPLES_PER_MS;
		last_ph_f1 = buf->f1_phase[msec];
		last_ph_f2 = buf->f2_phase[msec];

		for (size_t s = 0; s < SAMPLES_PER_MS; s++) {
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

