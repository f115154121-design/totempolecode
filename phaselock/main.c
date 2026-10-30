#include "DSP28x_Project.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

//==================================================================
// Function prototypes
//==================================================================
void InitEPwmTimer(void);
void currentloop(void);
void VoltageLoop(void);
void Busloop(void);
void PLL(void);
void lowpass(void);
void debun(void);

void UpdateVref(void);        // shared boost-bus reference used by both currentloop() and Busloop()
void SetSafeOutputs(void);    // full safe state (PFC + Buck) - used by turnoff() on a real fault trip
void SetSafeOutputsPFC(void); // PFC-leg-only safe state - used by Hold(), leaves Buck's own output alone
void turnoff(void);          // hardware trip (protection event)
void Hold(void);             // normal idle reset (not a fault)
void protect(void);          // latched fault supervisor
void ZeroCrossDetect(void);  // edge-detected zero crossing
void CheckPLLLock(void);     // counter-based PLL lock confirmation

__interrupt void adc_isr(void);

//==================================================================
// State / sequencing
//==================================================================
int state = 0;

int startupFLAG = 0;         // set once the post-power-up settle delay below has elapsed
int system_running = 0, starset_up = 0, switch_state = 0, count = 0;

//----- second switch (GPIO10): lets you manually decide when Buck is allowed to start,
//----- instead of relying only on the automatic PLL_flag && Vbus_real>=120 check in
//----- Busloop() - useful on the bench to wait until Vbus_real has visibly settled
//----- before letting Buck engage, sidestepping any chatter across that 120V threshold
//----- during the initial bus charge-up transient.
int buck_switch_state = 0, buck_count = 0;

//----- GPIO10 engages the Buck only on a zero crossing, mirroring the PFC's starset_up.
//----- Cleared whenever GPIO10 drops or the PLL unlocks, so every start re-syncs rather
//----- than resuming wherever the previous run left off.
int buck_started = 0;

//----- shutdown latch: once BOTH switches have been on (full start reached), turning
//----- GPIO11 off latches the whole system fully off - Buck AND PFC - instead of the
//----- Buck falling back to its 10% preload. Clears only when BOTH switches are off,
//----- so a restart must redo the GPIO10 -> GPIO11 sequence from scratch.
int fully_started = 0, shutdown_latch = 0;

//----- power-up settle delay ----------------------------------------
// ADC / analog front-end can read stale or transient values for a short time after
// power-up, which could otherwise trip protect() immediately. Hold everything in a
// safe state for STARTUP_DELAY_CYCLES ADC-ISR cycles (~50ms @ 20kHz, per PLL()'s
// dt=0.00005) before allowing protection or switching to engage. This window also
// gives the PLL time to converge before duty commands are trusted.
#define STARTUP_DELAY_CYCLES 1000
int startupDelayCount = 0;

//----- PLL lock confirmation (counter-based, mirrors CheckPLLLock() pattern) -----
int PLL_flag = 0;
int PLLcount = 0, PLLERRcount = 0;

//----- PFC outer-loop target -----
// Mirrors Vout_soft, assigned at the end of Busloop(). There is one soft-start
// ramp in this design - the Buck's - and the PFC voltage loop simply aims at
// it. See the comment at that assignment for what the second, independent ramp
// used to do.
float Vout_ref = 0;

//----- Buck duty soft-start (see Busloop) -----------------------------
// The Buck's own target, separate from Vout_ref (which the PFC outer loop
// regulates against). Seeded at the handover so the first regulated duty
// equals the preload duty exactly - stepping from the flat 10% preload
// straight to Vout_ref/Vref would otherwise jump the duty in one cycle.
// VOUT_SOFT_STEP is per 20kHz cycle: 0.0008 -> 16 V/s, so covering the ~80V
// from the seed (0.10*Vref, i.e. ~16-20V) to VOUT_SOFT_FINAL takes about 5s.
#define BUCK_PRELOAD_DUTY 0.10f
#define VOUT_SOFT_FINAL   100.0f
#define VOUT_SOFT_STEP    0.0008f
float Vout_soft = 0;

//----- one-stop debug snapshot -------------------------------------
// Everything worth reading at a glance, in one struct. Add `dbg` to the CCS
// Expressions view once and expand it: every value appears with its own name,
// and the whole block copies in a single selection. Refreshed at the end of
// every ADC ISR, including after a trip, so the latched state stays visible.
struct dbg_snapshot {
    float Ipre, Ipre_max;        // current-amplitude command and its soft-start ceiling
    float Vref, Vbus_real;       // bus target vs actual
    float Ve, Vout_ref;          // outer-loop error and its target
    float Vout_soft, Vout_real;  // Buck command vs actual output
    float Vo_filt;               // what the outer loop actually compares against
    float b1, Iin_real;          // line reconstruction and input current
    float duty, Vbus_duty;       // PFC and Buck duty
    float Ikp;                   // current-loop gain this cycle (K/Vbus)
    float Ii_sum;                // current-loop integrator state, in duty units. Zero until
                                 // the soft start hands over (Ipre_max >= 8); after that it
                                 // should settle to a small trim. Sitting on
                                 // +-CURRENT_LOOP_ISUM_LIM means it has saturated.
    int   PLL_flag, system_running, starset_up, buck_started;
    int   protectFLAG, Iin_protectFLAG, Vbus_protectFLAG;
    int   switch_state, buck_switch_state, shutdown_latch;

    // Extremes over the whole run, since a single cycle may not be
    // representative and there is no way to watch this live. These only ever
    // grow - zero them by hand in CCS before a run you want clean numbers for.
    float Iin_peak;              // max |Iin_real|
    float Vbus_max, Vbus_min;    // bus excursion
    float duty_max, duty_min;
    float Ipre_seen;             // highest Ipre the outer loop ever asked for
    float Ii_sum_absmax;         // max |Ii_sum|. If this reaches CURRENT_LOOP_ISUM_LIM the
                                 // integrator hit its clamp somewhere in the run, which the
                                 // live value above may well have left by the time you look.
};
struct dbg_snapshot dbg;

//----- latched protection -----------------------------------------
// Bus trip at 300 V. The old 230 V sat only 8% above the 212 V the bus actually
// reaches in normal running - the |b1|+40 envelope peaks at 201 V and 10 uF
// lets it swing - so normal operation was close enough to trip on its own.
// 300 V is 40% clear of that, and still well under the 450 V bus capacitor and
// far under the 900 V C3M0065090 switches.
#define VBUS_TRIP_V     300.0f
#define VBUS_OV_CONFIRM 2       // consecutive samples (see protect())

//----- input over-current trip --------------------------------------
// Has to clear the peak the outer loop is allowed to command (IPRE_MAX_FINAL)
// plus the inductor ripple (about 1 A p-p) plus transient headroom, while
// staying far enough below the switch rating to still be a real protection.
// C3M0065090 is good for 23 A at 100C, so there is room; the binding limit is
// the PFC inductor's saturation current, which is NOT yet measured.
// Step it together with IPRE_MAX_FINAL - see the staging table below.
#define IIN_TRIP_A      15.0f

//----- outer-loop current ceiling and soft start ---------------------
// Ipre is the AMPLITUDE of the current reference (Ic = Ipre * cos_value), so
// IPRE_MAX_FINAL is the peak line current the loop may ask for. Ipre_max ramps
// from IPRE_MAX_START to IPRE_MAX_FINAL at IPRE_MAX_STEP per 20kHz cycle, and
// that ramp doubles as the soft-start progress indicator for duty_cap (see
// currentloop()), the Vref ramp (UpdateVref()) and the current-loop integrator
// gate - which is why the span must never be hard-coded in those places.
//
// Staging table (104 Vrms, PF 0.997). Raise BOTH defines together, one step at
// a time, checking dbg.Iin_peak and the PFC inductor current waveform at each
// stage - the toroid's saturation point is the real unknown, and it shows up
// as the tail of the triangle curling upward before anything else goes wrong.
//
//   P_out   eff     P_in    Irms    Ipeak   IPRE_MAX_FINAL   IIN_TRIP_A
//    518 W  96.1%    539 W   5.20 A  7.35 A      8.0            12.0   (measured)
//    700 W  95.5%    733 W   7.07 A 10.00 A     10.5            15.0   <- here
//    850 W  95.0%    895 W   8.63 A 12.20 A     12.8            17.0
//   1000 W  94.4%   1059 W  10.21 A 14.44 A     15.2            20.0
//
// Efficiency is expected to FALL across these stages: losses go as I^2 while
// output goes as I, so the curve peaks at part load. That is normal.
#define IPRE_MAX_START  1.0f
#define IPRE_MAX_FINAL  10.5f
#define IPRE_MAX_SPAN   (IPRE_MAX_FINAL - IPRE_MAX_START)
#define IPRE_MAX_STEP   0.0005f   // per 20kHz cycle -> span/step/20k seconds
#define IPRE_MIN        1.0f      // floor on the commanded amplitude

int protectFLAG = 0;
int Iin_protectFLAG = 0, Vbus_protectFLAG = 0;
int vbus_ov_count = 0;
float Iin_protect_value = 0, Vbus_protect_value = 0;   // snapshot at trip time, for debugging

//----- zero-cross (edge-detected) ----------------------------------
// Fires once per line cycle, on the POSITIVE-going crossing only - see
// ZeroCrossDetect(). Everything that starts or stops a stage syncs to it.
int   zc_armed = 0, zero_cross_event = 0;

//----- capture buffer (debug) --------------------------------------
// Arms itself the instant starset_up first goes 0->1 (the first activated
// switching cycle after system_running turns on), then logs one sample per
// ADC-ISR cycle (every 50us). DBG_LEN * 50us = 16.65ms, almost exactly one
// 60Hz line cycle. protect() tripping freezes the buffer automatically, since
// the inner block stops running once protectFLAG latches.
//
// Two modes, picked by dbg_rolling:
//
//   dbg_rolling = 1 (default) - ROLLING. Keeps overwriting, so halting the
//     target at any moment leaves the LAST line cycle in the buffer. This is
//     what you want for steady-state work (half-cycle asymmetry, THD, phase).
//     The data wraps: dbg_index is the write head, so the OLDEST sample sits
//     at dbg_index and the newest at dbg_index-1. Unwrap before plotting.
//
//   dbg_rolling = 0 - ONE-SHOT. Stops after DBG_LEN samples, so the buffer
//     holds the FIRST line cycle after the start. This is the start-up
//     transient (inrush, duty ramp, Vref ramp) and does not wrap.
//
// Either mode can be re-armed by hand without restarting: halt, set
// dbg_index = 0 / dbg_done = 0 / dbg_armed = 1 in the Expressions view, then
// resume. starset_up_prev is already 1 by then, so the auto-arm above will
// not overwrite it.
//
// Read out with CCS (View -> Expressions, or Tools -> Graph on the arrays).
//
// Useful trick: a real ac current averages to zero over a whole line cycle,
// so mean(dbg_Iin_real) over the full buffer IS the current sensor's dc
// offset - the error term that makes the two half cycles unequal, since
// currentloop() rectifies with Ifb = +/-Iin_real. No need to stop and
// measure at zero current.
//----------------------------------------------------------------------
#define DBG_LEN 333
float dbg_b1[DBG_LEN];
float dbg_final_duty[DBG_LEN];
float dbg_Ie[DBG_LEN];
float dbg_Iin_real[DBG_LEN];
float dbg_Ipre[DBG_LEN];
float dbg_Vin_real[DBG_LEN];
float dbg_Vbus_real[DBG_LEN];
float dbg_Ii_sum[DBG_LEN];
int   dbg_index = 0;
int   dbg_armed = 0;
int   dbg_done = 0;
int   dbg_rolling = 1;   // see above: 1 = last line cycle, 0 = first line cycle
int   starset_up_prev = 0;

//==================================================================
// PLL / control constants
//==================================================================
#define kp 2.5
#define ki 0.5

// PFC outer voltage loop (VoltageLoop, one update per line cycle).
// Was 0.1 / 0.01, which put the crossover at 0.006 Hz - an integral time
// constant near 28 s, so a load step left Vo off by ~9 V for tens of seconds
// (docs/control_analysis README section 4.5/4.6). 0.03 / 1 moves the crossover
// to 0.54 Hz with 88 deg phase margin: back to 100 V in about a second, still
// far below the 120 Hz bus ripple and the 60 Hz sampling this loop runs at.
#define Vkp 0.03
#define Vki 1.0

//----- duty slew-rate limiter (insurance against a single-cycle jump) --
// duty_cap already bounds duty's magnitude, but nothing previously stopped
// it from jumping straight from 0 (Hold()'s reset value) to duty_cap on the
// very first activated cycle. Limiting the per-cycle step forces even a
// worst-case first cycle (e.g. wrong slow-leg direction) to ramp in over
// several cycles instead of slamming the full volt-seconds in one shot -
// gives protect() a chance to catch a real fault at a lower peak current
// before it dumps into Vbus.
//
// Two tiers, because one value cannot serve both jobs (docs/control_analysis
// README section 3). The feedforward's own slope near the zero crossing is
//     |d(d_ff)/dt| = w*Vm/Vref = 377*163/140 = 439/s -> 0.022 per 50us cycle
// so a 0.01 limit only tracks HALF of what the feedforward demands there: the
// duty falls behind every crossing and the current never gets shaped. The
// simulation puts that at THD 86% / PF 0.21, and the bench waveform shows the
// same thing - current in ramp segments instead of a sine.
//   0.03 is the steady-state value: >=0.025 restores THD 4.4% / PF 0.998.
// Switched on Ipre_max, which is already the soft-start progress indicator.
//
// The soft-start value was 0.01 on the same "keep the startup protection"
// reasoning, but the demand above applies during soft-start too, and that lasts
// the whole 0.7 s Ipre_max ramp. Measured 10/14: duty started correctly at 0.950
// against a required 0.969, then fell behind by 0.01 every cycle until it was
// 0.219 high, which over-boosts - Vbus = |vac|/(1-duty) reached 246 V and
// tripped the 230 V protection. The slow start value was not protecting
// anything, it was blocking a descent the converter needs. The bumpless seed
// below is what makes the start safe, so both stages can use the same rate.
#define DUTY_SLEW_START 0.03f
#define DUTY_SLEW_RUN   0.03f

// Startup duty_cap floor (see currentloop()). duty is the boost's STORING duty,
// so equilibrium is |vac| = (1-duty)*Vbus, i.e. duty = 1 - |vac|/Vbus. Near the
// zero crossing |vac| -> 0 and the REQUIRED duty approaches 1.0.
//
// That near-1 demand is not something to blunt - it is the operating point. High
// duty there is harmless (the inductor is shorted across a few volts, so di/dt is
// tiny); capping it low is what is dangerous, because it puts -(1-duty)*Vbus
// across the inductor and drives the current negative. Measured at 0.4: the
// inductor saw about -100 V just after the crossing and the current reached
// -10.5 A in 350 us, which is what the cap predicts for |vac| ~ 17 V.
//
// sim/sysim.c, the model the control analysis was validated against, has no cap
// at all - duty is free up to the 0.98 clamp. Keeping this near 0.98 makes the
// firmware match that model.
//
// A low cap also pins the bus: holding duty at the cap forces the steady state
// Vbus = |vac|/(1-cap), so 0.4 locks it at 155/0.6 = 258 V and the 230 V
// protection can only ever trip. Measured twice: Vbus peaked at 258.09 V on
// 10/01, and tripped at 246 V climbing toward the same point on 10/14.
#define DUTY_CAP_START 0.95f
float duty_prev_cmd = 0;
int   duty_seed_pending = 1;   // see currentloop(): seed the slew limiter on the first
                               // activated cycle instead of ramping up from zero

//----- Vref startup ramp (see UpdateVref) -----------------------------
// Vref departs from wherever the bus actually sits when switching begins
// (Vref_start, latched from Vbus_real) and interpolates up to the full
// |b1|+40 envelope, so the feedforward is accurate from the very first
// cycle instead of aiming at a target the bus hasn't reached yet.
// VREF_MIN only bounds the 1/Vref divisions in currentloop()/Busloop().
#define VREF_MIN 50.0f
float Vref_start = 0;

//----- current-loop Kp (dynamic) ---------------------------------------
// Kp = K/Vbus, matching the PSIM model this design was validated against.
// The boost's duty->current plant gain is Vbus/(sL), so dividing by the
// measured bus cancels it and holds the loop bandwidth constant at every
// bus voltage - a fixed Kp instead detunes the loop everywhere except the
// one bus it was designed at.
//   K = 2*pi*fc*L,  L = 1.566 mH (confirmed 2026-10-07)
//
// K is NOT set by the current loop alone. On its own the ceiling is where fc
// reaches 1/(4*Td) = 3.33 kHz, i.e. K ~= 33. What actually binds is the 10uF
// bus resonating against the Buck inductor (2.9-4.2 kHz) sitting right where
// the delayed loop looks like a negative resistance. Freezing the four-state
// system at each instantaneous |vac| gives K_max 8.5-12, worst at |vac| ~
// 100 V where it is 8.5 (docs/control_analysis README section 2).
//
// 7.0 -> fc = K/(2*pi*L) = 712 Hz, leaving ~1.5 of margin under that 8.5.
// The previous 9.3 was already past it and only survived because the dwell
// near |vac|=100 V is short; whole-system sim: K=7 THD 5.8%, K=9.3 THD 4.5%,
// K=10 THD 38% (unstable). 1.3 points of THD buys the margin back, and the
// ESR/DCR behind that 8.5 are still assumed values, not measured.
//
// Floor the divisor (not Kp) at 10V, same as PSIM: below that the bus sits
// under the line peak and the boost isn't regulating anyway.
#define CURRENT_LOOP_K       7.0f
#define CURRENT_LOOP_VBUS_MIN 10.0f
#define CURRENT_LOOP_KP_MIN   0.001f

//----- current-loop integral term (discrete PI, 2026-10-19) ------------------
// Kp + Ki/s = Ki(s/wz + 1)/s with wz = Ki/Kp. Discretised s -> (z-1)/Ts:
//   u[n] = u[n-1] + Kp(e[n]-e[n-1]) + Ki*Ts*e[n]
// Ki has to scale with 1/Vbus exactly like Kp, otherwise the zero walks around
// as the bus follows the envelope. Since Ki = wz*Kp, just scale Kp:
//   Iki_Ts = Ikp * wz * dt      (= 0.7917/Vbus at K=7, FZ=360, dt=50us)
//
// MATLAB (docs/control_analysis/matlab/pfc_buck_loops.m):
//   fc 711 -> 783 Hz,  PM 70.8 -> 44.2 deg,  GM 12.7 dB
//   60Hz tracking error 8.42% -> 1.41%,  60Hz phase error -4.83 -> -0.11 deg
// The zero costs 26.6 deg of phase margin - that is what dropping K from 9.3 to
// 7.0 paid for, so do NOT put K back up. README section 8 has the derivation;
// section 2's K_max = 8.5 was derived for pure P and does not apply unchanged.
//
// Set FZ to 0 to fall back to pure P (the behaviour before 2026-10-19).
#define CURRENT_LOOP_FZ       360.0f
#define CURRENT_LOOP_WZ       (2.0f * 3.14159265f * CURRENT_LOOP_FZ)
// Backstop clamp on the integrator state, in duty units. Normal operation only
// needs the integrator to trim what the feedforward gets wrong, which is small;
// 0.3 is far above that but still bounds a runaway from a bad sensor reading.
#define CURRENT_LOOP_ISUM_LIM 0.3f

//----- slow-leg (EPwm2) commutation dead window -----------------------
// Both slow-leg arms are held off while |b1| is under this threshold. The window has to
// outlast several ISR cycles or the 20kHz sampling steps straight over it and the leg
// commutates with zero dead time - EPwm2's dead-band module adds none of its own (its
// RED/FED delays cancel out in both commutation directions), so this window is the leg's
// only interlock. Half-width in time = SLOW_LEG_DEAD_V / (b1 peak * 377), i.e. ~86us at a
// 155V peak, about 3.4 ISR cycles. Re-derive if the line voltage or the ISR rate changes.
#define SLOW_LEG_DEAD_V  3.0f

//----- AQCSFRC.CSFx software-force levels (TI's headers define no constants for these) -----
#define AQ_SW_LOW   1     // force a continuous low on that output
#define AQ_SW_HIGH  2     // force a continuous high on that output

int Vin = 0, Iin = 0, Vbus = 0, Iout = 0, Vout = 0;

float a = 0, b = 0, b1 = 0, sfq = 0, sfd = 0, s5q = 0, s5d = 0, cfq = 0, cfd = 0, c5q = 0, c5d = 0,
      c5a = 0, c5b = 0, s5a = 0, s5b = 0, cfa = 0, cfb = 0, sfa = 0, sfb = 0, d = 0, q = 0, d1 = 0, q1 = 0, dt = 0.00005,
      prevoutput = 0, prevoutput1 = 0, z = 0.001568, error = 0,
      integral = 0, integral1 = 0, w = 0, Ifb = 0, Iref = 0,
      Ie = 0, duty = 0, Vac_in = 0, Iin_real = 0, feedforward = 0, Ikp = 0,
      Vbus_real = 0, Vout_real = 0, Ipi = 0, Ii_sum = 0, Iki_Ts = 0,
      Vo_filt = 0, Vo_prev = 0, Ve = 0, Ipre = 0, Ve_prev = 0.0, Ipre_prev = 0, Ic = 0, triggered = 0,
      cos_value = 0, sin_value = 0, Vbus_duty = 0, Ipre_max = IPRE_MAX_START,
      Vref = 140,Vin_real = 0;

//==================================================================
// main
//==================================================================
int main(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    EALLOW;
    PieVectTable.ADCINT1 = &adc_isr;
    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;

    // external switch inputs
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO11 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO15 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;  GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;

    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;  //LED7 - system_running indicator
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  //LED8 - protect indicator
    GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  //LED9 - PLL lock indicator (live: on while PLL_flag==1, off when lost)
    GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
    EDIS;

    //--------------------------------------------------------------
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();

    InitCpuTimers();
    CpuTimer1Regs.TCR.all = 0x4000;

    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 2;

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 3;
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 4;
    AdcRegs.ADCSOC5CTL.bit.CHSEL    = 5;
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 6;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 6;
    EDIS;

    InitAdc();

    c5d = c5a = cos(0.08722);
    c5q = c5b = cos(0.08722);
    s5d = s5a = sin(0.08722);
    s5q = s5b = sin(0.08722);

    //--------------------------------------------------------------
    // EPwm/PWM start (and with it, EPwm1's SOCA->ADC trigger) is moved to run
    // AFTER the ADC is fully configured above - it used to run before, so the
    // very first SOC pulses could hit an unconfigured ADC.
    //--------------------------------------------------------------
    // Minimum trackers must start high or the zero-initialised value is already
    // the smallest and they never update.
    dbg.Vbus_min = 1.0e6f;
    dbg.duty_min = 1.0e6f;

    IER |= M_INT1;
    InitEPwmTimer();
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EINT;
    ERTM;

    //================================================================
    // main loop: only non-time-critical / debounced work lives here.
    // Everything time-critical (control loops, protection re-check)
    // runs in adc_isr() at the ADC sample rate.
    //================================================================
    while(1)
    {
        debun();          // switch debounce - not urgent, doesn't need ISR-rate execution
        // startupFLAG is now set inside adc_isr() after STARTUP_DELAY_CYCLES have elapsed
    }
}

//==================================================================
void InitEPwmTimer()
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
    InitEPwm1Gpio();
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBPRD = 2250;
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // still the sync master
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // sync OUT to EPWM2/3
    EPwm1Regs.TBCTR = 0x0000;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBRED = 50;
    EPwm1Regs.DBFED = 50;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;     // was AQ_CLEAR - with no SET action anywhere, EPWM1A/B could never go high; combined with
                                            // IN_MODE/POLSEL below this forced both switches on this leg permanently HIGH (shoot-through).
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;     // kept in sync with AQCTLA; unused while IN_MODE=DBA_RED_DBB_FED (B is derived from A).
    EPwm1Regs.CMPA.half.CMPA = 0;          // both arms off before the first ISR - the reset value 0/0 leaves EPWM1B 100% on
    EPwm1Regs.CMPB = 2250;

    //----------------------------------------------------------------
    // EPWM2 = totem-pole PFC leg B - this is the role your old EPWM3 had.
    // Slaved to EPWM1 via SYNCOSEL/TB_SYNC_IN, same as before.
    //----------------------------------------------------------------
    InitEPwm2Gpio();
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBPRD = 4500;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTR = 0x0000;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HI;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBRED = 2000;
    EPwm2Regs.DBFED = 2000;
    // Line-frequency leg: commutated by the AQCSFRC software force below, not by CMPA/CMPB.
    // CMPA=0 could not express "off" here - ZRO=SET outranks CAU=CLEAR, so it held A high.
    // The AQCTL actions below never take effect while a continuous force is active.
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm2Regs.AQSFRC.bit.RLDCSF = 0;           // reload AQCSFRC at CTR=0, same boundary as the CMP shadows
    EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_LOW;    // both arms off until the ISR commutates the leg
    EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_LOW;

    InitEPwm3Gpio();
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBPRD = 2250;   // was 539 (~167kHz) - now 90MHz/4500 = 20kHz, matching EPwm1
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
    EPwm3Regs.TBPHS.half.TBPHS = 0;
    EPwm3Regs.TBCTR = 0x0000;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBRED = 75;
    EPwm3Regs.DBFED = 75;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;     
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;
    EPwm3Regs.CMPA.half.CMPA = 0;   // both arms off until Busloop() takes over - the reset value 0/0 is D=1, i.e. high-side 100% on
    EPwm3Regs.CMPB = 2250;

    //----------------------------------------------------------------
    // Trip zone. TZCTL was never configured, so turnoff()'s TZFRC.OST had no defined effect on
    // the pins. OST is a one-shot latch and nothing in this file ever writes TZCLR, so a trip
    // holds both gates low until the device is reset or reprogrammed.
    //----------------------------------------------------------------
    EALLOW;
    EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

__interrupt void adc_isr(void)
{

    Vin  = AdcResult.ADCRESULT0 - 1558;
    Iin  = AdcResult.ADCRESULT1 - 2037;
    Iout = AdcResult.ADCRESULT2 - 2020;
    Vout = AdcResult.ADCRESULT3 - 2032;
    Vbus = AdcResult.ADCRESULT4 - 2029;

    Iin_real  = Iin  * 0.0222222;
    Vbus_real = Vbus * 0.48604;
    Vout_real = Vout * 0.123668;
    Vin_real = Vin * 0.231343;

    lowpass();
    PLL();
    ZeroCrossDetect();   // zero_cross_event: once per line cycle, positive half-cycle start only
    CheckPLLLock();      // updates PLL_flag - runs every cycle so it's already progressing during the settle window below

    if(PLL_flag)
    {
        GpioDataRegs.GPASET.bit.GPIO29 = 1;    // LED9 on - PLL locked
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;  // LED9 off - not locked (or lock lost)
    }

    //--------------------------------------------------------------
    // power-up settle: hold everything safe until the ADC/analog front-end
    // (and the PLL) have had time to settle, so stale readings at power-on
    // can't trip protect() or command a bogus duty cycle.
    //--------------------------------------------------------------
    if(!startupFLAG)
    {
        if(startupDelayCount < STARTUP_DELAY_CYCLES)
        {
            startupDelayCount++;
        }
        else
        {
            startupFLAG = 1;   // settle time elapsed - protection and switching may now engage
        }
        Hold();
        AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
        return;
    }

    protect();

    if(!protectFLAG)
    {
        //--------------------------------------------------------------
        // Shutdown latch. Full start = both switches on at once; after that,
        // dropping GPIO11 latches everything fully off (Buck + PFC) and keeps
        // it off until BOTH switches are off again (clean restart required).
        //--------------------------------------------------------------
        if(switch_state && buck_switch_state)
        {
            fully_started = 1;
        }
        if(fully_started && switch_state == 0)
        {
            shutdown_latch = 1;
        }
        if(switch_state == 0 && buck_switch_state == 0)
        {
            fully_started = 0;
            shutdown_latch = 0;
        }

        if(shutdown_latch)
        {
            system_running = 0;
            Vbus_duty = 0;
            Vout_ref = 0;
            Vout_soft = 0;                   // next handover re-seeds from the preload
            buck_started = 0;
            Hold();                          // PFC off + reset all PFC control state
            EPwm3Regs.CMPA.half.CMPA = 0;    // Buck fully off (both arms)
            EPwm3Regs.CMPB = 2250;

            AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
            PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
            return;
        }

        //--------------------------------------------------------------
        // PFC run/stop. Runs before Busloop() so the Buck reads this cycle's system_running -
        // that's what makes both stages stop on the same zero crossing.
        //--------------------------------------------------------------
        if(switch_state == 1 && PLL_flag)   // added PLL_flag - don't start switching until the PLL is confirmed locked
        {
            system_running = 1;
        }
        else if(zero_cross_event)
        {
            system_running = 0;
        }

        UpdateVref();          // shared by currentloop() and Busloop() - must run before both
        Busloop();

        //--------------------------------------------------------------
        // Buck runs off Vbus_duty every cycle, independently of switch_state - that's what
        // gives the ">60V bus, 10% preload before the main switch" behaviour.
        //--------------------------------------------------------------
        if(Vbus_duty <= 0)
        {
            EPwm3Regs.CMPA.half.CMPA = 0;   // real both-off pair; the (1-D) form maps D=0 to low-side 100% on, not off
            EPwm3Regs.CMPB = 2250;
        }
        else
        {
            EPwm3Regs.CMPA.half.CMPA = Vbus_duty * 2250;
            EPwm3Regs.CMPB = Vbus_duty * 2250;
        }

        if(system_running == 1)
        {
            GpioDataRegs.GPASET.bit.GPIO27 = 1;

            if(zero_cross_event || starset_up)
            {
                starset_up = 1;

                VoltageLoop();
                currentloop();

                float final_duty = duty;
                if(final_duty > 0.97) final_duty = 0.97;
                if(final_duty < 0.03) final_duty = 0.03;

                //----- startup capture (debug) -----
                if(!starset_up_prev)   // starset_up just went 0->1 this cycle: (re)arm from sample 0
                {
                    dbg_index = 0;
                    dbg_armed = 1;
                    dbg_done = 0;
                }
                starset_up_prev = starset_up;

                if(dbg_armed && !dbg_done)
                {
                    dbg_b1[dbg_index]         = b1;
                    dbg_final_duty[dbg_index] = final_duty;
                    dbg_Ie[dbg_index]         = Ie;
                    dbg_Iin_real[dbg_index]   = Iin_real;
                    dbg_Ipre[dbg_index]       = Ipre;
                    dbg_Vin_real[dbg_index]   = Vin_real;
                    dbg_Vbus_real[dbg_index]  = Vbus_real;
                    dbg_Ii_sum[dbg_index]     = Ii_sum;
                    dbg_index++;
                    if(dbg_index >= DBG_LEN)
                    {
                        dbg_index = 0;          // wrap; dbg_index is the write head
                        if(!dbg_rolling)        // one-shot mode stops here instead
                        {
                            dbg_done = 1;
                            dbg_armed = 0;
                        }
                    }
                }

            if(b1 < 5 && b1 > -5)
            {
                EPwm1Regs.CMPA.half.CMPA = 0;
                EPwm1Regs.CMPB = 2250;
                EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_LOW;    // both slow-leg arms off through the zero crossing
                EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_LOW;
            }

            else if(b1 > 0)
            {
                EPwm1Regs.CMPA.half.CMPA = (1 - final_duty) * 2250;
                EPwm1Regs.CMPB = (1 - final_duty) * 2250;
                EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_LOW;
                EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_HIGH;
            }
            else
            {
                EPwm1Regs.CMPA.half.CMPA = final_duty * 2250;
                EPwm1Regs.CMPB = final_duty * 2250;
                EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_HIGH;
                EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_LOW;
            }
            }
            else
            {
                Hold();
            }
        }
        else
        {
            Hold();
            starset_up = 0;
        }
    }
    else
    {
        Hold();
    }


    if(system_running && !protectFLAG)
    {
        if(fabsf(Iin_real) > dbg.Iin_peak) dbg.Iin_peak  = fabsf(Iin_real);
        if(Vbus_real > dbg.Vbus_max)       dbg.Vbus_max  = Vbus_real;
        if(Vbus_real < dbg.Vbus_min)       dbg.Vbus_min  = Vbus_real;
        if(duty > dbg.duty_max)            dbg.duty_max  = duty;
        if(duty < dbg.duty_min)            dbg.duty_min  = duty;
        if(Ipre > dbg.Ipre_seen)           dbg.Ipre_seen = Ipre;
        if(fabsf(Ii_sum) > dbg.Ii_sum_absmax) dbg.Ii_sum_absmax = fabsf(Ii_sum);

        dbg.Ipre      = Ipre;       dbg.Ipre_max  = Ipre_max;
        dbg.Vref      = Vref;       dbg.Vbus_real = Vbus_real;
        dbg.Ve        = Ve;         dbg.Vout_ref  = Vout_ref;
        dbg.Vout_soft = Vout_soft;  dbg.Vout_real = Vout_real;
        dbg.Vo_filt   = Vo_filt;
        dbg.b1        = b1;         dbg.Iin_real  = Iin_real;
        dbg.duty      = duty;       dbg.Vbus_duty = Vbus_duty;
        dbg.Ikp       = Ikp;        dbg.Ii_sum    = Ii_sum;
    }


    dbg.PLL_flag  = PLL_flag;   dbg.system_running = system_running;
    dbg.starset_up = starset_up; dbg.buck_started  = buck_started;
    dbg.protectFLAG = protectFLAG;
    dbg.Iin_protectFLAG = Iin_protectFLAG;
    dbg.Vbus_protectFLAG = Vbus_protectFLAG;
    dbg.switch_state = switch_state;
    dbg.buck_switch_state = buck_switch_state;
    dbg.shutdown_latch = shutdown_latch;

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void ZeroCrossDetect(void)
{
    if(b1 < -SLOW_LEG_DEAD_V)
    {
        zc_armed = 1;
    }

    zero_cross_event = (zc_armed && b1 > SLOW_LEG_DEAD_V);

    if(zero_cross_event)
    {
        zc_armed = 0;
    }
}


void CheckPLLLock(void)
{
    if(fabsf(d1) > 1 || integral1 < -0.2 || integral1 > 6.5)
    {
        PLLcount = 0;
        if(PLLERRcount < 11)
        {
            PLLERRcount++;
        }
        else
        {
            PLL_flag = 0;
        }
        return;
    }

    if(fabsf(d1) < 0.5 && q1 >= 5 && q1 < 400 && integral1 >= 0 && integral1 < 6.3)
    {
        PLLERRcount = 0;
        if(PLLcount < 1000)
        {
            PLLcount++;
        }
        else
        {
            PLL_flag = 1;
        }
        return;
    }

    PLLcount = 0;   // in-between zone - neither clearly locked nor clearly failed; don't let count creep up here
}

void currentloop(void)
{
    Ic = Ipre * cos_value;

    float vbus_for_ikp = Vbus_real;

    if(vbus_for_ikp < CURRENT_LOOP_VBUS_MIN) vbus_for_ikp = CURRENT_LOOP_VBUS_MIN;
    Ikp = CURRENT_LOOP_K / vbus_for_ikp;
    if(Ikp < CURRENT_LOOP_KP_MIN) Ikp = CURRENT_LOOP_KP_MIN;

    if(b1 >= 0)
    {
        Vac_in = b1;
        Ifb = Iin_real;
        Iref = Ic;
    }
    else
    {
        Vac_in = -b1;
        Ifb = -Iin_real;
        Iref = -Ic;
    }

    feedforward = 1 - (Vac_in / Vref);
    Ie = Iref - Ifb;

    // Integral term. Held hard at zero until the Ipre_max soft start has finished,
    // so the whole start-up sequence runs as the pure-P version that is known to
    // come up at 7.4 A peak. Engaging afterwards is bumpless: the state starts at
    // zero and only grows to cancel the error P leaves behind, by which time the
    // converter is already in steady state.
    if(CURRENT_LOOP_FZ > 0.0f && Ipre_max >= IPRE_MAX_FINAL)
    {
        Iki_Ts = Ikp * (CURRENT_LOOP_WZ * dt);   // Ki = wz*Kp, so reuse Ikp's divide
        Ii_sum += Iki_Ts * Ie;
    }
    else
    {
        Iki_Ts = 0;
        Ii_sum = 0;
    }
    if(Ii_sum >  CURRENT_LOOP_ISUM_LIM) Ii_sum =  CURRENT_LOOP_ISUM_LIM;
    if(Ii_sum < -CURRENT_LOOP_ISUM_LIM) Ii_sum = -CURRENT_LOOP_ISUM_LIM;

    Ipi = Ikp * Ie + Ii_sum;

    float duty_unsat = feedforward + Ipi;   // kept for the anti-windup back-calculation
    duty = duty_unsat;

    float duty_cap = DUTY_CAP_START + (1.0f - DUTY_CAP_START) * ((Ipre_max - IPRE_MAX_START) / IPRE_MAX_SPAN);
    if(duty > duty_cap) duty = duty_cap;

    if(duty >= 0.98) duty = 0.98;
    if(duty <= 0.05) duty = 0.05;
    if(duty_seed_pending)
    {
        duty_prev_cmd = duty;
        duty_seed_pending = 0;
    }


    float duty_slew = (Ipre_max < IPRE_MAX_FINAL) ? DUTY_SLEW_START : DUTY_SLEW_RUN;
    if(duty > duty_prev_cmd + duty_slew) duty = duty_prev_cmd + duty_slew;
    if(duty < duty_prev_cmd - duty_slew) duty = duty_prev_cmd - duty_slew;
    duty_prev_cmd = duty;

    // Back-calculation anti-windup. Applied AFTER duty_cap, the hard clamps and
    // the slew limiter, because the integrator has to account for what was really
    // commanded, not what was computed. This matters most near the zero crossing:
    // there feedforward is already 1 - 0/140 = 1.0 and duty sits on its ceiling
    // every cycle, so without this the state would wind up twice per line cycle
    // and dump the excess into the next quarter cycle as a current overshoot.
    // When nothing saturates the correction is exactly zero.
    Ii_sum += (duty - duty_unsat);
    if(Ii_sum >  CURRENT_LOOP_ISUM_LIM) Ii_sum =  CURRENT_LOOP_ISUM_LIM;
    if(Ii_sum < -CURRENT_LOOP_ISUM_LIM) Ii_sum = -CURRENT_LOOP_ISUM_LIM;
}


void UpdateVref(void)
{

    float ramp = (Ipre_max - IPRE_MAX_START) / IPRE_MAX_SPAN;
    if(ramp < 0.0f) ramp = 0.0f;
    if(ramp > 1.0f) ramp = 1.0f;

    if(ramp <= 0.0f)
    {
        Vref_start = Vbus_real;
    }

    float vref_full;
    if(fabsf(b1) > 100)   // b1 = PLL-locked, filtered reconstruction of Vac (same source currentloop() uses)
    {
        vref_full = fabsf(b1) + 40;
    }
    else
    {
        vref_full = 140;
    }

    Vref = Vref_start + (vref_full - Vref_start) * ramp;

    if(Vref < vref_full) Vref = vref_full;

    if(Vref < VREF_MIN) Vref = VREF_MIN;
}

void Busloop(void)
{
    if(PLL_flag && buck_switch_state)
    {
        if(zero_cross_event)
        {
            buck_started = 1;
        }

        if(!buck_started)
        {
            Vbus_duty = 0;   // GPIO10 already on, but hold off until the next zero crossing
            Vout_soft = 0;   // re-seed on the next handover
        }
        else if(switch_state == 1)
        {
            if(Vout_soft <= 0.0f)
            {
                Vout_soft = BUCK_PRELOAD_DUTY * Vref;   // continuous with the preload
            }

            Vout_soft += VOUT_SOFT_STEP;
            if(Vout_soft > VOUT_SOFT_FINAL) Vout_soft = VOUT_SOFT_FINAL;

            Vbus_duty = Vout_soft / Vref;
            if(Vbus_duty > 0.90) Vbus_duty = 0.90;
            if(Vbus_duty < 0.10) Vbus_duty = 0.10;
        }
        else
        {
            Vbus_duty = BUCK_PRELOAD_DUTY;
            Vout_soft = 0;   // main switch off again - next handover re-seeds
        }
    }
    else
    {
        buck_started = 0;
        Vbus_duty = 0;
        Vout_soft = 0;
    }

    Vout_ref = Vout_soft;
}

void SetSafeOutputsPFC(void)
{
    EPwm1Regs.CMPA.half.CMPA = 0;   // 0% physical on-time for EPwm1 (POLSEL=HIC convention, same as main.c)
    EPwm1Regs.CMPB = 2250;
    EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_LOW;   // both slow-leg arms off
    EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_LOW;
    
}

void SetSafeOutputs(void)
{
    SetSafeOutputsPFC();
    EPwm3Regs.CMPA.half.CMPA = 0;   // both-off pair: outA = CMPA/TBPRD = 0, outB = 1 - CMPB/TBPRD = 0
    EPwm3Regs.CMPB = 2250;
}


void turnoff(void)
{
    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
    SetSafeOutputs();
}


void Hold(void)
{
    SetSafeOutputsPFC();

    starset_up = 0;
    starset_up_prev = 0;   // so the next start re-arms dbg capture from sample 0 again
    Ipi = 0;
    Ii_sum = 0;            // never carry integral state across a stop/start
    Iki_Ts = 0;
    Ie = 0;
    Iref = 0;
    Ifb = 0;
    duty = 0;
    duty_seed_pending = 1;   // next start re-seeds the slew limiter from its own
                             // first computed duty rather than climbing from zero
    Ipre_prev = 0;
    Ve_prev = 0;
    Ipre_max = IPRE_MAX_START;
    triggered = 0;
}


void protect(void)
{
    if(protectFLAG)
        goto ProtectLED;

    if(Vbus_real > VBUS_TRIP_V) vbus_ov_count++;
    else                        vbus_ov_count = 0;

    if(fabsf(Iin_real) > IIN_TRIP_A || vbus_ov_count >= VBUS_OV_CONFIRM)
    {
        turnoff();
        protectFLAG = 1;

        if(fabsf(Iin_real) > IIN_TRIP_A)
        {
            Iin_protect_value = Iin_real;
            Iin_protectFLAG = 1;
        }
        if(vbus_ov_count >= VBUS_OV_CONFIRM)
        {
            Vbus_protect_value = Vbus_real;
            Vbus_protectFLAG = 1;
        }
    }

    ProtectLED:
        if(protectFLAG)
        {
            turnoff();
            GpioDataRegs.GPASET.bit.GPIO28 = 1;   // fault indicator LED
        }
}

void debun(void)
{
    if(GpioDataRegs.GPADAT.bit.GPIO11 == 1)
    {
        if(count < 1000)
        {
            count++;
        }
        if(count >= 1000)
        {
            switch_state = 1;
        }
    }
    else
    {
        if(count > 0)
        {
            count--;
        }
        else
        {
            switch_state = 0;
        }
    }

    //----- second switch (GPIO10) - same debounce pattern, gates Buck separately -----
    if(GpioDataRegs.GPADAT.bit.GPIO10 == 1)
    {
        if(buck_count < 1000)
        {
            buck_count++;
        }
        if(buck_count >= 1000)
        {
            buck_switch_state = 1;
        }
    }
    else
    {
        if(buck_count > 0)
        {
            buck_count--;
        }
        else
        {
            buck_switch_state = 0;
        }
    }
}

void lowpass(void)
{
    Vo_filt = Vo_prev + z * (Vout_real - Vo_filt);
    Vo_prev = Vo_filt;
}

void VoltageLoop(void)
{
    if(integral1 > 4.69494 && integral1 < 4.727444)
    {
        if(!triggered)
        {
            Ve = Vout_ref - Vo_filt;
            Ipre = Ipre_prev + Vkp * (Ve - Ve_prev) + Vki * 0.01667 * Ve;
            if(Ipre > Ipre_max) Ipre = Ipre_max;
            if(Ipre < IPRE_MIN) Ipre = IPRE_MIN;

            Ve_prev = Ve;
            Ipre_prev = Ipre;
            triggered = 1;
        }
    }
    else
    {
        triggered = 0;
    }

    if(Ipre_max < IPRE_MAX_FINAL)
    {
        Ipre_max = Ipre_max + IPRE_MAX_STEP;
    }
    if(Ipre_max > IPRE_MAX_FINAL)
    {
        Ipre_max = IPRE_MAX_FINAL;
    }
    if(Ipre > Ipre_max) Ipre = Ipre_max;
    if(Ipre < IPRE_MIN) Ipre = IPRE_MIN;
}

void PLL(void)
{
    b = Vin * 0.231343;

    cos_value = cos(integral1);
    sin_value = sin(integral1);
    cfd = cfa = cfq = cfb = cos_value;
    sfd = sfa = sfq = sfb = sin_value;

    switch(state)
    {
        case 0:
            if(integral1 > 0)
            {
                state = 1;
            }
            d = (c5a * a + s5b * b);
            q = ((-s5a * a) + c5b * b);
            a = (c5d * d1 + (-s5q * q1));
            b1 = (s5d * d1 + c5q * q1);
            break;
        case 1:
            d = (cfa * a + sfb * b);
            q = ((-sfa * a) + cfb * b);
            a = (cfd * d1 + (-sfq * q1));
            b1 = (sfd * d1 + cfq * q1);
            state = 1;
            break;
    }

    error = -d;
    integral = integral + error * ki * dt;
    w = (error * kp) + integral + 376.991;

    integral1 = fmod(integral1 + w * dt, 6.28);

    z = 0.001568;
    d1 = z * d + (1 - z) * prevoutput;
    prevoutput = d1;
    q1 = z * q + (1 - z) * prevoutput1;
    prevoutput1 = q1;
}
