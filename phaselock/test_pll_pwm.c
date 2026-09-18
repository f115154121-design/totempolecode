#include "DSP28x_Project.h"
#include <math.h>
#include <string.h>

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsRunStart;

//==================================================================
// Bench test: PLL + the PWM state it drives, and nothing else.
//
// Feed a real sine wave (signal generator) into ADCINA0 (Vin) and check:
//   1) the PLL locks onto it 嚙踝蕭 watch d1/q1/integral1/PLL_flag in a
//      Watch/Expression window (or DAC one of them out if you have a
//      spare channel)
//   2) EPwm1A/B, EPwm2A/B track the half-cycle / zero-crossing dead
//      window correctly on a scope
//
// No current/voltage feedback, no currentloop()/VoltageLoop()/Busloop(),
// no protect(), no power-up settle delay, no Buck stage (EPwm3) 嚙踝蕭 all of
// that depends on real power-stage measurements this bench setup doesn't
// have. `test_duty` below stands in for what currentloop() would normally
// compute.
//
// IMPORTANT 嚙踝蕭 only one file with main() can be in the active CCS build at
// a time. Exclude main.c (and test.c) from the build before building this
// file (right-click the file in Project Explorer -> Exclude from Build).
//
// IMPORTANT 嚙踝蕭 a signal generator's sine output swings positive AND
// negative, but the ADC can only read 0..3.3V. You need a bias/attenuator
// circuit between the generator and ADCINA0 that shifts the sine into the
// ADC's range, the same way the real line-voltage sensor would. The
// "1558" offset below assumes that bias sits at the same midpoint as the
// real sensor circuit 嚙踝蕭 if your bench bias circuit is different, retune
// this constant so Vin reads ~0 at the sine wave's zero crossing.
//==================================================================

void InitEPwmTimer(void);
void PLL(void);
void ZeroCrossDetect(void);
void CheckPLLLock(void);
void Hold(void);     // safe-state PWM output while waiting for GPIO18 / the next zero-crossing
void debun(void);    // GPIO18 debounce, called from the main loop (not time-critical)

__interrupt void adc_isr(void);

//==================================================================
// PLL state (same structure/constants as main.c's PLL())
//==================================================================
int state = 0;

#define kp 2.5
#define ki 0.5

int   Vin = 0;
float Vin_real = 0;

float a = 0, b = 0, b1 = 0, d = 0, q = 0, d1 = 0, q1 = 0, dt = 0.00005,
      prevoutput = 0, prevoutput1 = 0, z = 0.001568,
      error = 0, error_1 = 0, integral = 0, integral1 = 0, w = 0,
      c5a = 0, s5a = 0, cos_value = 0, sin_value = 0;

//----- zero-cross (edge-detected) ----------------------------------
float theta_prev = 0;
int   zero_cross_pos = 0, zero_cross_neg = 0;
int   zc_now = 0, zc_prev = 0, zero_cross_event = 0;

//----- PLL lock confirmation (same pattern as main.c) ----------------
int PLL_flag = 0;
int PLLcount = 0, PLLERRcount = 0;

//----- stand-in for currentloop()'s duty (no real current feedback here) --
float test_duty = 0;   // bench-safe starting value; adjust as needed

//----- GPIO18 start trigger, zero-cross synchronized (mirrors main.c's switch_state/
//----- system_running/starset_up pattern) ---------------------------------
int switch_state = 0, system_running = 0, starset_up = 0, count = 0;

//----- capture buffers for bench inspection -----------------------------
// Circular capture: fills index by index every ADC-ISR cycle and wraps back
// to 0 once full, continuously overwriting the oldest sample — so pausing
// the debugger at any point shows the most recent MATRIX_ROWS samples, not
// just whatever happened right at power-up. Three separate 1D arrays instead
// of one interleaved matrix — each is already one contiguous block, so
// exporting them (CCS Memory Browser / Graph) drops straight into its own
// Excel column with no INDEX()/MOD() unscrambling needed. Note: because it
// wraps, index 0 isn't necessarily the oldest sample — there's a one-sample
// discontinuity in time order right at wherever matrix_index currently sits.
// volatile so the compiler can't optimize the writes away as dead stores —
// nothing else in this file ever reads them back.
#define MATRIX_ROWS 255
volatile float Vin_real_buf[MATRIX_ROWS];
volatile float b1_buf[MATRIX_ROWS];
volatile float integral1_buf[MATRIX_ROWS];
volatile int   matrix_index = 0;

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
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;   // GPIO18 as plain GPIO input �� bench start-trigger switch
    GpioCtrlRegs.GPADIR.bit.GPIO17  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO17  = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 01;  GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
    EDIS;

    //--------------------------------------------------------------
    // Relocate the "ramfuncs" section (includes the DELAY_US()/_DSP28x_usDelay
    // code InitAdc() calls below) to RAM and set Flash wait-states for the
    // 90MHz clock InitSysCtrl() just configured. This file was missing both ��
    // without them, InitAdc()'s internal DELAY_US() call runs straight out of
    // Flash at 90MHz with unconfigured wait-states, which is exactly what the
    // F2806x_usDelay.asm header warns can hang/fault the CPU.
    //--------------------------------------------------------------
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();

    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCREFSEL   = 0;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 0;   // was 2 (copied from main.c's 6-channel round-robin) 嚙踝蕭 this file
                                            // only converts SOC0, so the interrupt must fire on EOC0, not EOC2
                                            // (which never happens here) or ADCINT1 never fires at all

    AdcRegs.ADCSOC0CTL.bit.CHSEL   = 0;   // ADCINA0 = Vin (signal generator sine, biased into 0..3.3V)
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // triggered by EPwm1 SOCA
    AdcRegs.ADCSOC0CTL.bit.ACQPS   = 6;
    EDIS;

    InitAdc();

    c5a = cos(0.08722);
    s5a = sin(0.08722);

    IER |= M_INT1;
    InitEPwmTimer();
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    EINT;
    ERTM;

    while(1)
    {
        debun();   // GPIO18 debounce �� not time-critical, doesn't need ISR-rate execution
    }
}

//==================================================================
void InitEPwmTimer(void)
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    // EPWM1 = timebase master + ADC trigger + PFC leg A (high-frequency)
    InitEPwm1Gpio();
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;

    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    EPwm1Regs.TBPRD               = 2250;   // 90MHz/(2*2250) = 20kHz
    EPwm1Regs.TBPHS.half.TBPHS    = 0x0000;
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE;   // sync master
    EPwm1Regs.TBCTL.bit.PRDLD     = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL  = TB_CTR_ZERO;  // sync out to EPWM2
    EPwm1Regs.TBCTR               = 0x0000;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV1;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.DBCTL.bit.IN_MODE  = DBA_RED_DBB_FED;
    EPwm1Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBRED = 50;
    EPwm1Regs.DBFED = 50;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

    // EPWM2 = PFC leg B (line-frequency synchronous-rectification leg)
    InitEPwm2Gpio();
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
    EPwm2Regs.TBPRD               = 4500;
    EPwm2Regs.TBCTL.bit.PHSEN     = TB_ENABLE;
    EPwm2Regs.TBPHS.half.TBPHS    = 0;
    EPwm2Regs.TBCTR               = 0x0000;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1;
    EPwm2Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm2Regs.DBCTL.bit.IN_MODE  = DBA_RED_DBB_FED;
    EPwm2Regs.DBCTL.bit.POLSEL   = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBRED = 2000;
    EPwm2Regs.DBFED = 2000;
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;   
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;

    InitEPwm3Gpio();
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBPRD = 2250;   // was 539 (~167kHz) — now 90MHz/4500 = 20kHz, matching EPwm1
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
    EPwm3Regs.DBRED = 50;
    EPwm3Regs.DBFED = 50;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

//==================================================================
__interrupt void adc_isr(void)
{
    Vin      = AdcResult.ADCRESULT0 - 1558;   // retune 1558 to your bench bias circuit's zero-crossing code
    Vin_real = Vin * 0.231343f;

    PLL();
    ZeroCrossDetect();
    CheckPLLLock();

    Vin_real_buf[matrix_index] = Vin_real;
    b1_buf[matrix_index] = b1;
    integral1_buf[matrix_index] = integral1;
    matrix_index++;
    if(matrix_index >= MATRIX_ROWS)
    {
        matrix_index = 0;   // wrap around — keeps overwriting so the buffer always holds the most
                             // recent MATRIX_ROWS samples, not just whatever happened right at power-up
    }

    //--------------------------------------------------------------
    // GPIO18 start trigger, zero-cross synchronized: pressing/asserting GPIO18
    // "arms" system_running immediately, but actual switching (starset_up=1)
    // only begins at the next zero_cross_event. Same pattern as main.c's
    // switch_state/system_running/starset_up, using GPIO18 instead of SW1.
    //--------------------------------------------------------------
    if(switch_state == 1)
    {
        system_running = 1;
    }
    else if(zero_cross_event)
    {
        system_running = 0;
    }

    if(system_running == 1)
    {
        if(zero_cross_event || starset_up)
        {
            starset_up = 1;

            test_duty = 1-(fabs(b1)/400);
            if(test_duty > 0.95)
                {
                test_duty = 0.95;
                }
            if(test_duty < 0)
                {
                    test_duty = 0;
                }
            EPwm3Regs.CMPA.half.CMPA = 500;   // matches TBPRD=4500 and the Vbus_duty=0 convention used elsewhere
            EPwm3Regs.CMPB = 500;
            if(b1 > 2)
            {
                EPwm1Regs.CMPA.half.CMPA = test_duty * 2250;
                EPwm1Regs.CMPB = test_duty * 2250;
                EPwm2Regs.CMPA.half.CMPA = 4500;
                EPwm2Regs.CMPB = 0;
            }
            else if(b1 < -2)
            {
                EPwm1Regs.CMPA.half.CMPA = (1 - test_duty) * 2250;
                EPwm1Regs.CMPB = (1 - test_duty) * 2250;
                EPwm2Regs.CMPA.half.CMPA = 0;
                EPwm2Regs.CMPB = 4500;
            }
            else
            {
                 EPwm1Regs.CMPA.half.CMPA = 0;   // 0% physical on-time for EPwm1 (POLSEL=HIC convention, same as main.c)
                 EPwm1Regs.CMPB = 2250;
                 EPwm2Regs.CMPA.half.CMPA = 0;      // defined static level �� verify on scope this is the state you want;
                 EPwm2Regs.CMPB = 0;                // retune if EPwm2's HIC polarity means this isn't the "safe" combo
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

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//==================================================================
void Hold(void)
{
    EPwm1Regs.CMPA.half.CMPA = 0;   // 0% physical on-time for EPwm1 (POLSEL=HIC convention, same as main.c)
    EPwm1Regs.CMPB = 2250;
    EPwm2Regs.CMPA.half.CMPA = 0;      // defined static level �� verify on scope this is the state you want;
    EPwm2Regs.CMPB = 0;                // retune if EPwm2's HIC polarity means this isn't the "safe" combo
    EPwm3Regs.CMPA.half.CMPA = 0;   // matches TBPRD=4500 and the Vbus_duty=0 convention used elsewhere
    EPwm3Regs.CMPB = 2250;
}

void debun(void)
{
    if(GpioDataRegs.GPADAT.bit.GPIO17 == 1)   // active-low: F2806x GPIOs default to pull-up ENABLED, so
                                               // floating/unconnected reads 1 (safe/untriggered) �� short
                                               // GPIO18 to GND to trigger, don't wire it to 3.3V
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
}

//==================================================================
void ZeroCrossDetect(void)
{
    // was integral1-range based (two separate windows near phase ~1.55/~4.69) — any PLL phase
    // error misaligns those windows against b1's actual zero-crossing, causing erratic switching
    // near the zero point. Using b1's own magnitude instead catches both crossings (positive-going
    // and negative-going) with a single band, and tracks the real reconstructed voltage directly.
    zc_now = (b1 > -1.0f && b1 < 1.0f);
    zero_cross_event = (zc_now && !zc_prev);
    zc_prev = zc_now;
}

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

    PLLcount = 0;
}

//==================================================================
void PLL(void)
{
    b = Vin_real;

    cos_value = cos(integral1);
    sin_value = sin(integral1);

    switch(state)
    {
        case 0:
            if(integral1 > 0)
            {
                state = 1;
            }
            d  = (c5a * a + s5a * b);
            q  = ((-s5a * a) + c5a * b);
            a  = (c5a * d1 + (-s5a * q1));
            b1 = (s5a * d1 + c5a * q1);
            break;
        case 1:
            d  = (cos_value * a + sin_value * b);
            q  = ((-sin_value * a) + cos_value * b);
            a  = (cos_value * d1 + (-sin_value * q1));
            b1 = (sin_value * d1 + cos_value * q1);
            break;
    }

    error_1 = error;
    error   = -d;
    integral = integral + error * ki * dt;
    w = (error * kp) + integral + 376.991;

    integral1 = integral1 + w * dt;
    if(integral1 >= 6.283185)
    {
        integral1 = integral1 - 6.283185;
    }

    d1 = z * d + (1 - z) * prevoutput;
    prevoutput = d1;
    q1 = z * q + (1 - z) * prevoutput1;
    prevoutput1 = q1;
}
