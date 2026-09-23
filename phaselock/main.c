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
void UpdateVoutRefRamp(void); // soft-starts the shared Vout_ref target once the main switch is on
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

//----- Buck soft-start ramp (shared Vout_ref target for VoltageLoop() and Busloop()) -----
// Once the main switch (SW1 -> switch_state) turns on, Vout_ref ramps from VOUT_REF_START
// to VOUT_REF_FINAL over BUCK_RAMP_CYCLES ADC-ISR cycles (~2s @ 20kHz, per PLL()'s dt=0.00005).
// Both the PFC outer voltage loop and the Buck feedforward read this SAME variable so they
// always target the same setpoint - if they used different/unsynced targets, the PFC's Ipre
// and the Buck's duty would be aiming at different output levels during the ramp.
#define VOUT_REF_START   10.0f
#define VOUT_REF_FINAL   100.0f
#define BUCK_RAMP_CYCLES 40000

//----- Buck soft-start ramp state -----
float Vout_ref = VOUT_REF_START;
int   buck_ramp_count = 0;

//----- latched protection -----------------------------------------
int protectFLAG = 0;
int Iin_protectFLAG = 0, Vbus_protectFLAG = 0;
float Iin_protect_value = 0, Vbus_protect_value = 0;   // snapshot at trip time, for debugging

//----- zero-cross (edge-detected) ----------------------------------
float theta_prev = 0;
int   zero_cross_pos = 0, zero_cross_neg = 0;
int   zc_now = 0, zc_prev = 0, zero_cross_event = 0;

//----- startup capture buffer (debug) ------------------------------
// Arms itself the instant starset_up first goes 0->1 (i.e. the very first
// activated switching cycle after system_running turns on), then logs one
// sample per ADC-ISR cycle (every 50us) until DBG_LEN samples are taken or
// protect() trips (whichever comes first - the inner block stops running
// once protectFLAG latches, which freezes the buffer automatically).
// Read out with CCS (View -> Expressions, or Tools -> Graph on the arrays)
// after halting the target; dbg_index shows how many samples were filled.
//----------------------------------------------------------------------
#define DBG_LEN 333
float dbg_b1[DBG_LEN];
float dbg_final_duty[DBG_LEN];
float dbg_Ie[DBG_LEN];
float dbg_Iin_real[DBG_LEN];
float dbg_Ipre[DBG_LEN];
float dbg_Vin_real[DBG_LEN];
float dbg_Vbus_real[DBG_LEN];
int   dbg_index = 0;
int   dbg_armed = 0;
int   dbg_done = 0;
int   starset_up_prev = 0;

//==================================================================
// PLL / control constants
//==================================================================
#define kp 2.5
#define ki 0.5
#define Vkp 0.1
#define Vki 0.01

//----- duty slew-rate limiter (insurance against a single-cycle jump) --
// duty_cap already bounds duty's magnitude, but nothing previously stopped
// it from jumping straight from 0 (Hold()'s reset value) to duty_cap on the
// very first activated cycle. Limiting the per-cycle step forces even a
// worst-case first cycle (e.g. wrong slow-leg direction) to ramp in over
// several cycles instead of slamming the full volt-seconds in one shot -
// gives protect() a chance to catch a real fault at a lower peak current
// before it dumps into Vbus. 0.01/cycle -> ~15 cycles (0.75ms) to reach the
// DUTY_CAP_START startup duty_cap; independent of and much faster than the
// existing ~0.7s Ipre_max ramp, so it doesn't slow down normal soft-start.
#define DUTY_SLEW_MAX 0.01f

// Startup duty_cap floor (see currentloop()) - was 0.30, lowered further to
// shrink the residual Vo/Vac step still seen at the very first activated
// cycle even with DUTY_SLEW_MAX in place. Retune if soft-start needs more
// headroom (too low may stall current buildup near the zero crossing).
#define DUTY_CAP_START 0.15f
float duty_prev_cmd = 0;

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
//   K = 2*pi*fc*L = 2*pi*945*0.0015658 ~= 9.3  (L=1.5658mH measured)
// Floor the divisor (not Kp) at 10V, same as PSIM: below that the bus sits
// under the line peak and the boost isn't regulating anyway.
#define CURRENT_LOOP_K       9.3f
#define CURRENT_LOOP_VBUS_MIN 10.0f
#define CURRENT_LOOP_KP_MIN   0.001f

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
      Vbus_real = 0, Vout_real = 0, Ipi = 0,
      Vo_filt = 0, Vo_prev = 0, Ve = 0, Ipre = 0, Ve_prev = 0.0, Ipre_prev = 0, Ic = 0, triggered = 0,
      cos_value = 0, sin_value = 0, Vbus_duty = 0, Ipre_max = 1,
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
    Iin  = AdcResult.ADCRESULT1 - 2049;
    Iout = AdcResult.ADCRESULT2 - 2020;
    Vout = AdcResult.ADCRESULT3 - 2037;
    Vbus = AdcResult.ADCRESULT4 - 2034;

    Iin_real  = Iin  * 0.0222222;
    Vbus_real = Vbus * 0.48604;
    Vout_real = Vout * 0.123668;
    Vin_real = Vin * 0.231343;

    lowpass();
    PLL();
    ZeroCrossDetect();   // computes zero_cross_pos/neg + zero_cross_event (edge only, fires once per crossing)
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
            buck_ramp_count = 0;
            Vout_ref = VOUT_REF_START;
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
        UpdateVoutRefRamp();   // soft-starts the shared Vout_ref target once switch_state && buck_switch_state
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

                // Independent of currentloop()'s own [0.03, 0.97] clamp on purpose:
                // this is the last bound before the CMPA writes below, and both the
                // D and (1-D) branches need final_duty inside [0,1] or the Uint16
                // register lands past TBPRD (or wraps, for a negative float) and the
                // compare stops matching - which latches a gate on instead of switching.
                float final_duty = duty;
                if(final_duty > 0.95) final_duty = 0.95;
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
                    dbg_index++;
                    if(dbg_index >= DBG_LEN)
                    {
                        dbg_done = 1;
                        dbg_armed = 0;
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
                EPwm1Regs.CMPA.half.CMPA = final_duty * 2250;   // was test_duty - undeclared in main.c (that's
                EPwm1Regs.CMPB = final_duty * 2250;             // test_pll_pwm.c's stand-in var)
                EPwm2Regs.AQCSFRC.bit.CSFA = AQ_SW_LOW;
                EPwm2Regs.AQCSFRC.bit.CSFB = AQ_SW_HIGH;
            }
            else
            {
                EPwm1Regs.CMPA.half.CMPA = (1 - final_duty) * 2250;
                EPwm1Regs.CMPB = (1 - final_duty) * 2250;
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

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void ZeroCrossDetect(void)
{
    zero_cross_pos = (theta_prev < 1.55f && integral1 >= 1.55f);
    zero_cross_neg = (theta_prev < 4.69f && integral1 >= 4.69f);

    zc_now = (zero_cross_pos || zero_cross_neg);
    zero_cross_event = (zc_now && !zc_prev);   // edge only - fires once
    zc_prev = zc_now;

    theta_prev = integral1;
}

//==================================================================
// CheckPLLLock - counter-based lock confirmation (mirrors the reference
// CheckPLLLock() pattern, adapted to this file's own d1/q1/integral1).
// Thresholds are carried over as a starting point from that reference
// (same 0.231343 ADC scaling, so amplitudes should be in a similar range)
// - watch q1/d1 on the bench and retune if they don't match your hardware.
//==================================================================
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
    Ic = Ipre * cos_value;   // moved above its use below - was computed after Iref=Ic, so Iref was
                              // reading last cycle's stale Ic instead of this cycle's fresh value

    float vbus_for_ikp = Vbus_real;   // measured bus, not Vref - this divisor exists to
                                       // cancel the plant's own Vbus gain, so it has to be
                                       // the real one (unlike the feedforward, where Vref
                                       // is what shapes the bus)
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
    Ipi = Ikp * Ie;
    duty = feedforward + Ipi;

    // Startup duty soft-start: ride the total duty up with Ipre_max (1->8, the same
    // ~0.7s current-amplitude ramp) instead of letting switching begin at the near-1
    // feedforward duty near the zero crossing - which the low-gain P current loop
    // can't pull back before the inductor current overshoots. Cap reaches 1.0 once
    // Ipre_max hits 8, so it stops limiting after soft-start and the 0.97 clamp rules.
    float duty_cap = DUTY_CAP_START + (1.0f - DUTY_CAP_START) * ((Ipre_max - 1.0f) / 7.0f);
    if(duty > duty_cap) duty = duty_cap;

    if(duty >= 0.97) duty = 0.97;
    if(duty <= 0.03) duty = 0.03;

    // Slew-rate insurance (see DUTY_SLEW_MAX above) - clamp this cycle's step
    // relative to what was actually commanded last cycle.
    if(duty > duty_prev_cmd + DUTY_SLEW_MAX) duty = duty_prev_cmd + DUTY_SLEW_MAX;
    if(duty < duty_prev_cmd - DUTY_SLEW_MAX) duty = duty_prev_cmd - DUTY_SLEW_MAX;
    duty_prev_cmd = duty;
}

//==================================================================
// UpdateVref - the bus reference the feedforward shapes Vbus onto. Because
// currentloop() commands d = 1 - Vac_in/Vref, the boost settles at
// Vbus = Vac_in/(1-d) = Vref, so this IS the bus-shaping mechanism: Vbus
// follows whatever envelope Vref describes. Keep the feedforward on Vref -
// swapping in the measured bus makes d = 1 - Vac/Vbus an identity with no
// restoring force, and the bus stops tracking anything.
//
// The gap between Vref and the still-charging bus is what the feedforward
// turns into inrush current: pure-P can only correct it by (d_error/Ikp),
// so at Ikp=0.06 even a 0.25 duty mismatch is ~4A of current error that no
// realistic gain can claw back. Rather than fight it, start Vref AT the
// measured bus and interpolate up to the full envelope - the feedforward is
// then correct from the first switching cycle, which is the same thing the
// reference design buys with its pre-charge stage before switching begins.
//==================================================================
void UpdateVref(void)
{
    // 0 -> 1 over the existing Ipre_max soft-start (1->8, ~0.7s), so the bus
    // target and the current-amplitude limit open up together
    float ramp = (Ipre_max - 1.0f) / 7.0f;
    if(ramp < 0.0f) ramp = 0.0f;
    if(ramp > 1.0f) ramp = 1.0f;

    // Re-latch the present bus until the ramp actually starts moving, so the
    // departure point is wherever the bus sits at that moment. Frozen from
    // then on: Vref must not keep chasing Vbus_real, or a sagging bus would
    // drag its own target down after it.
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

    if(Vref < VREF_MIN) Vref = VREF_MIN;
}

//==================================================================
// Busloop - GPIO10 (buck_switch_state) arms Buck at a fixed 10% preload duty;
// GPIO11 (switch_state) on top of that switches it into closed-form regulation
// at Vout_ref/Vref. Neither switch on (or PLL not locked) holds Buck fully off.
//==================================================================
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
        }
        else if(switch_state == 1)
        {
            Vbus_duty = Vout_ref / Vref;
            if(Vbus_duty > 0.90) Vbus_duty = 0.90;
            if(Vbus_duty < 0.10) Vbus_duty = 0.10;
        }
        else
        {
            Vbus_duty = 0.1;
        }
    }
    else
    {
        buck_started = 0;
        Vbus_duty = 0;
    }
}

//==================================================================
// UpdateVoutRefRamp - soft-starts the shared Vout_ref target from VOUT_REF_START up to
// VOUT_REF_FINAL over BUCK_RAMP_CYCLES cycles, once both switches are on - the same
// condition Busloop() uses to enter its Vout_ref/Vref branch, so the ramp starts
// exactly when that branch starts reading it. Resets back to the start value whenever
// either switch drops, so every fresh start begins a clean ramp rather than resuming
// mid-ramp.
//==================================================================
void UpdateVoutRefRamp(void)
{
    if(switch_state && buck_switch_state)
    {
        if(buck_ramp_count < BUCK_RAMP_CYCLES)
        {
            buck_ramp_count++;
            Vout_ref = VOUT_REF_START + (VOUT_REF_FINAL - VOUT_REF_START) *
                       ((float)buck_ramp_count / (float)BUCK_RAMP_CYCLES);
        }
        else
        {
            Vout_ref = VOUT_REF_FINAL;
        }
    }
    else
    {
        buck_ramp_count = 0;
        Vout_ref = VOUT_REF_START;
    }
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

// Hold - normal idle / not-yet-running reset for the PFC side only. NOT a fault: resets
// every PFC control-loop integrator/state so a later start begins clean. Deliberately does
// NOT touch EPwm3/Buck (uses SetSafeOutputsPFC, not the full SetSafeOutputs) - Buck now runs
// independently of system_running/starset_up and manages its own output via Busloop() every
// cycle, so forcing it "safe" here would fight that and undo what Busloop() just computed.
void Hold(void)
{
    SetSafeOutputsPFC();

    starset_up = 0;
    starset_up_prev = 0;   // so the next start re-arms dbg capture from sample 0 again
    Ipi = 0;
    Ie = 0;
    Iref = 0;
    Ifb = 0;
    duty = 0;
    duty_prev_cmd = 0;   // so the slew limiter also ramps from true zero on the next start
    Ipre_prev = 0;
    Ve_prev = 0;
    Ipre_max = 1;
    triggered = 0;
}


void protect(void)
{
    if(protectFLAG)
        goto ProtectLED;

    if(fabsf(Iin_real) > 15 || Vbus_real > 230)   // combined so a simultaneous over-current AND over-voltage
                                                   // both get recorded below, instead of the first one checked
                                                   // (via else-if) silently hiding the other
    {
        turnoff();
        protectFLAG = 1;

        if(fabsf(Iin_real) > 15)
        {
            Iin_protect_value = Iin_real;
            Iin_protectFLAG = 1;
        }
        if(Vbus_real > 230)
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
            Ve_prev = Ve;
            Ipre_prev = Ipre;
            triggered = 1;
        }
    }
    else
    {
        triggered = 0;
    }

    if(Ipre_max < 8)
    {
        Ipre_max = Ipre_max + 0.0005;
    }
    if(Ipre_max > 8)
    {
        Ipre_max = 8;
    }
    if(Ipre > Ipre_max) Ipre = Ipre_max;
    if(Ipre < 1) Ipre = 1;
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
