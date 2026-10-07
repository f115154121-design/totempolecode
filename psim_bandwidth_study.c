#include <Stdlib.h>
#include <String.h>
#include <math.h>
#include <Psim.h>

//======================================================================
// Bandwidth-interaction study: PFC current loop vs Buck bus-voltage loop.
//
// Topology (per the block diagram):
//   AC -> Lac -> totem-pole (Q1..Q4) -> Cbus -> Buck (Q5/Q6) -> Ldc -> Vo
//   Vo is a 100V VOLTAGE SOURCE, so there is no output-voltage loop.
//   The PFC current loop runs on a FIXED Iref.
//
// Both stages aim at the SAME bus target - the envelope Vref = |b1|+40.
// The PFC feedforward shapes Vbus onto it; the Buck trims around it. If the
// two used different targets (say an envelope here and a constant there)
// they fight, the bus loop never reaches its operating point, and a
// bandwidth sweep measures that conflict instead of the loop dynamics.
//
// That leaves exactly two tunable bandwidths, which is the point of the
// experiment:
//   I_LOOP_FC   - current-loop crossover (PFC)
//   Vkp         - bus-loop proportional gain (Buck)
//
// Enables:
//   in[6] -> 0.2s, the inrush resistor being bypassed. A hardware event, not
//            a control enable: it just means the bus is now precharged to the
//            line peak through a low impedance.
//   in[5] -> 0.3s, BOTH stages start together.
//
// Both stages share in[5] deliberately. Cbus is only 10uF - about 0.12J at
// 155V - so at 600W the bus empties in 0.2ms. Letting the Buck run alone for
// the 0.1s between the two triggers would collapse the bus and leave the bus
// loop throttling against a sagging rail, and that transient would sit right
// on top of the loop interaction this block exists to measure.
//
// The PLL below is the DSP's own (main.c PLL()), so the phase driving Iref
// and the sign driving the leg commutation come from the reconstruction b1
// rather than from the true line - same as the hardware. Set USE_PLL 0 to
// fall back to the schematic's theta (in[4]) and the true Vac, which is the
// clean reference case: any difference between the two runs is the PLL's
// contribution, not the loop bandwidths under study.
//======================================================================
#define USE_PLL 1

//----- PLL loop filter (main.c) ---------------------------------------
#define PLL_KP   2.5
#define PLL_KI   0.5
#define PLL_DT   0.00005

//----- operating point -------------------------------------------------
#define VAC_PEAK      155.0     // line amplitude
#define VO_NOMINAL    100.0     // the output voltage source
#define P_IN_TARGET   600.0     // input power setpoint

// P = Vrms*Irms = (Vpk/sqrt2)(Ipk/sqrt2) = Vpk*Ipk/2  ->  Ipk = 2P/Vpk
#define IAC_PEAK_REF  (2.0 * P_IN_TARGET / VAC_PEAK)    // ~7.74 A peak

//----- the two bandwidth knobs ----------------------------------------
#define L_AC          0.0015608   // input inductor [H]
#define I_LOOP_FC     1200.0      // current-loop crossover [Hz]  <-- sweep me
#define Vkp           0.001       // bus-loop proportional gain   <-- sweep me

//----- rates -----------------------------------------------------------
#define PWM_FREQ         20000.0
#define CONTROL_FREQ     20000.0
#define Ts  (1.0 / CONTROL_FREQ)

//----- limits ----------------------------------------------------------
#define DUTY_MAX        0.98
#define DUTY_MIN        0.02
#define DUTY_BUCK_MAX   0.95
#define DUTY_BUCK_MIN   0.05
#define VBUS_SAFE_MIN   10.0

//======================================================================
// measured inputs
//======================================================================
double Vac = 0, Vbus = 0, Vo = 0, Iac = 0, theta = 0;

//----- PLL state (main.c PLL()) ---------------------------------------
int    pll_state = 0;
double a = 0, b = 0, b1 = 0;
double d = 0, q = 0, d1 = 0, q1 = 0;
double c5a = 0, c5b = 0, c5d = 0, c5q = 0;
double s5a = 0, s5b = 0, s5d = 0, s5q = 0;
double cfa = 0, cfb = 0, cfd = 0, cfq = 0;
double sfa = 0, sfb = 0, sfd = 0, sfq = 0;
double cos_value = 0, sin_value = 0;
double pll_error = 0, pll_integral = 0, integral1 = 0, w = 0;
double prevoutput = 0, prevoutput1 = 0;
double z = 0.001568;

int PLL_flag = 0, PLLcount = 0, PLLERRcount = 0;

//----- PFC current loop ------------------------------------------------
double Iref = 0, Ifb = 0, Ie = 0;
double kp = 0, Ip = 0, pifcL = 0;
double feedforward = 0, duty = 0, Vref = 0;
double phase = 0;

//----- Buck bus-voltage loop -------------------------------------------
double Ve_bus = 0;
double feedforward_buck = 0, duty_buck = 0;

//----- carrier / gating -------------------------------------------------
double saw = 0, saw_0 = 0, saw1 = 0, saw_1 = 0;
int S = 0, R = 0, Q = 0;
int S1 = 0, R1 = 0, Q1 = 0;
int counter20k = 0;

//======================================================================
// PLL - verbatim from main.c. b is taken in volts straight from the PSIM
// input; the DSP gets the same units by scaling raw ADC counts by 0.231343.
//======================================================================
static void PLL(void)
{
    b = Vac;

    cos_value = cos(integral1);
    sin_value = sin(integral1);
    cfd = cfa = cfq = cfb = cos_value;
    sfd = sfa = sfq = sfb = sin_value;

    switch(pll_state)
    {
        case 0:
            if(integral1 > 0) pll_state = 1;
            d  = (c5a * a + s5b * b);
            q  = ((-s5a * a) + c5b * b);
            a  = (c5d * d1 + (-s5q * q1));
            b1 = (s5d * d1 + c5q * q1);
            break;
        case 1:
            d  = (cfa * a + sfb * b);
            q  = ((-sfa * a) + cfb * b);
            a  = (cfd * d1 + (-sfq * q1));
            b1 = (sfd * d1 + cfq * q1);
            pll_state = 1;
            break;
    }

    pll_error    = -d;
    pll_integral = pll_integral + pll_error * PLL_KI * PLL_DT;
    w            = (pll_error * PLL_KP) + pll_integral + 376.991;

    integral1 = fmod(integral1 + w * PLL_DT, 6.28);

    z  = 0.001568;
    d1 = z * d + (1 - z) * prevoutput;
    prevoutput  = d1;
    q1 = z * q + (1 - z) * prevoutput1;
    prevoutput1 = q1;
}

//======================================================================
static void CheckPLLLock(void)
{
    if(fabs(d1) > 1 || integral1 < -0.2 || integral1 > 6.5)
    {
        PLLcount = 0;
        if(PLLERRcount < 11) PLLERRcount++;
        else                 PLL_flag = 0;
        return;
    }

    if(fabs(d1) < 0.5 && q1 >= 5 && q1 < 400 && integral1 >= 0 && integral1 < 6.3)
    {
        PLLERRcount = 0;
        if(PLLcount < 1000) PLLcount++;
        else                PLL_flag = 1;
        return;
    }

    PLLcount = 0;
}

//======================================================================
void SimulationStep(
        double t, double delt, double *in, double *out,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k++;

    //------------------------------------------------------------------
    // 20kHz control task. The divider is derived from the actual sim step
    // instead of a hardcoded 100, so changing Time Step in Simulation
    // Control cannot silently move the control rate. At 5E-07 this is 100.
    //------------------------------------------------------------------
    int ctrl_div = (int)((1.0 / CONTROL_FREQ) / delt + 0.5);
    if(ctrl_div < 1) ctrl_div = 1;

    if(counter20k >= ctrl_div)
    {
        Vac   = in[0];
        Vbus  = in[1];
        Iac   = in[2];
        Vo    = in[3];
        theta = in[4];

#if USE_PLL
        PLL();
        CheckPLLLock();
#else
        // reference case: schematic's PLL output, and the true line for
        // phase/sign - no reconstruction error anywhere
        integral1 = theta;
        cos_value = cos(theta);
        b1        = Vac;
        PLL_flag  = 1;
#endif

        // after PLL() so it uses this cycle's b1, not the previous one
        Vref = (fabs(b1) > 100.0) ? (fabs(b1) + 40.0) : 140.0;

        double vbus_safe = (Vbus < VBUS_SAFE_MIN) ? VBUS_SAFE_MIN : Vbus;

        //--------------------------------------------------------------
        // Buck bus-voltage loop  (in[5], 0.3s - same trigger as the PFC)
        //
        // feedforward_buck = Vo/Vref puts the Buck exactly at break-even
        // when Vbus sits on the envelope: duty*Vbus = Vo, so V_Ldc = 0 and
        // the inductor current holds. The P term trims around that.
        //
        // Sign / stability limit. Substituting Vbus = Vref - Ve_bus:
        //
        //   V_Ldc = (Vo/Vref + Vkp*Ve)(Vref - Ve) - Vo
        //         = Ve*(Vkp*Vref - Vo/Vref) - Vkp*Ve^2
        //
        // The feedforward already supplies negative feedback through the
        // -Vo/Vref*Ve term: the bus sags, duty*Vbus falls with it, V_Ldc
        // goes negative, the Buck draws less and the bus recovers. The
        // +Vkp*Ve term works AGAINST that, so this form stays stable only
        // while the bracket is negative:
        //
        //   Vkp < Vo / Vref^2
        //
        // At Vo=100 that is 5.1e-3 at the envelope trough (Vref=140) and
        // 2.6e-3 at the peak (Vref=195) - the peak is what binds. Above it
        // the loop flips to positive feedback. Sweeping Vkp should find
        // that boundary at ~2.6e-3; predicting it and then hitting it is a
        // cleaner result than sweeping blind.
        //
        // Flipping this + to a - removes the limit entirely (the bracket
        // becomes -Vkp*Vref - Vo/Vref, always negative), which is what you
        // want if the sweep should run to the real dynamic limit instead
        // of stopping at this algebraic sign flip.
        //--------------------------------------------------------------
        if(in[5] > 0)
        {
            Ve_bus           = Vref - Vbus;
            feedforward_buck = Vo / Vref;
            duty_buck        = feedforward_buck + (Ve_bus * Vkp);

            if(duty_buck > DUTY_BUCK_MAX) duty_buck = DUTY_BUCK_MAX;
            if(duty_buck < DUTY_BUCK_MIN) duty_buck = DUTY_BUCK_MIN;
        }
        else
        {
            Ve_bus    = 0;
            duty_buck = 0;
        }

        //--------------------------------------------------------------
        // PFC current loop  (in[5], 0.3s) - fixed Iref, no outer loop
        //--------------------------------------------------------------
        if(in[5] > 0 && PLL_flag)
        {
            phase = cos_value;        // PLL's phase, not the line's

            // fixed-amplitude rectified reference, in phase with the line
            Iref = IAC_PEAK_REF * fabs(phase);

            Ifb = (b1 < 0) ? -Iac : Iac;

            Ie = Iref - Ifb;

            // Kp = 2*pi*fc*L / Vbus : dividing by the measured bus cancels
            // the plant's own Vbus/(sL) gain, holding the crossover at
            // I_LOOP_FC independently of where the bus happens to sit
            pifcL = 2.0 * 3.14159265 * I_LOOP_FC * L_AC;
            kp    = pifcL / vbus_safe;

            feedforward = 1.0 - (fabs(b1) / Vref);

            Ip   = Ie * kp;
            duty = feedforward + Ip;

            if(duty > DUTY_MAX) duty = DUTY_MAX;
            if(duty < DUTY_MIN) duty = DUTY_MIN;
        }
        else
        {
            Iref = 0; Ie = 0; Ip = 0; duty = 0;
        }

        counter20k = 0;
    }

    //------------------------------------------------------------------
    // carriers
    //------------------------------------------------------------------
    saw += PWM_FREQ * delt;
    if(saw >= 1.0) saw -= 1.0;
    saw_0 = 1 - saw;

    saw1 += PWM_FREQ * delt;
    if(saw1 >= 1.0) saw1 -= 1.0;
    saw_1 = 1 - saw1;

    S = (duty > saw_0) ? 1 : 0;
    R = (0.99 > saw_0) ? 1 : 0;
    if(S && !R) Q = 1;
    if(!S && R) Q = 0;
    if(S && R)  Q = 1;

    S1 = (duty_buck > saw_1) ? 1 : 0;
    R1 = (0.98 > saw_1) ? 1 : 0;
    if(S1 && !R1) Q1 = 1;
    if(!S1 && R1) Q1 = 0;
    if(S1 && R1)  Q1 = 1;

    //------------------------------------------------------------------
    // PFC legs
    //------------------------------------------------------------------
    if(in[5] > 0 && PLL_flag)
    {
        // commutation follows b1, i.e. the PLL's idea of the crossing
        if(b1 < 0) { out[0] = 1 - Q; out[1] = Q;     out[2] = 1; out[3] = 0; }
        else       { out[0] = Q;     out[1] = 1 - Q; out[2] = 0; out[3] = 1; }
    }
    else
    {
        out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 0;
    }

    //------------------------------------------------------------------
    // Buck leg
    //------------------------------------------------------------------
    if(in[5] > 0) { out[4] = Q1; out[5] = 1 - Q1; }
    else          { out[4] = 0;  out[5] = 0;      }

    //------------------------------------------------------------------
    // probes
    //------------------------------------------------------------------
    out[6]  = Ie;
    out[7]  = Iref;
    out[8]  = Ifb;
    out[9]  = feedforward;
    out[10] = feedforward_buck;
    out[11] = Vo;
    out[12] = Vbus;
    out[13] = duty;
    out[14] = duty_buck;
    out[15] = Ve_bus;          // bus-loop error
    out[16] = phase;
    out[17] = kp;              // current-loop gain actually in use
    out[18] = Vac * Ifb;       // instantaneous input power
    out[19] = b1;              // PLL reconstruction - overlay on Vac to see
                               // the phase error the hardware actually has
    out[20] = Vref;            // envelope both stages are aiming at
}

//======================================================================
void SimulationBegin(
        const char *szId, int nInputCount, int nOutputCount,
        int nParameterCount, const char ** pszParameters,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k = 0;
    saw = 0; saw1 = 0;
    duty = 0; duty_buck = 0;
    Q = 0; Q1 = 0;
    Iref = 0; Ie = 0; Ip = 0;
    Ve_bus = 0; feedforward_buck = 0;

    // main.c seeds the case-0 rotation constants here
    c5d = c5a = cos(0.08722);
    c5q = c5b = cos(0.08722);
    s5d = s5a = sin(0.08722);
    s5q = s5b = sin(0.08722);

    pll_state = 0;
    integral1 = 0; pll_integral = 0; w = 0;
    d = q = d1 = q1 = a = b = b1 = 0;
    prevoutput = prevoutput1 = 0;
    PLL_flag = 0; PLLcount = 0; PLLERRcount = 0;
}

//======================================================================
void SimulationEnd(const char *szId, void ** reserved_UserData,
                   int reserved_ThreadIndex, void * reserved_AppPtr)
{
}
