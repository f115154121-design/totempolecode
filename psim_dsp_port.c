#include <Stdlib.h>
#include <String.h>
#include <math.h>
#include <Psim.h>

//======================================================================
// PSIM C-block, ported to match the DSP control path (phaselock/main.c).
//
// Why this exists: the original PSIM block fed the controller the TRUE
// Vac (in[0]) and an ideal theta (in[4]). The DSP has neither - it runs
// a PLL and then drives ALL of the following from the reconstruction:
//
//     Vref            = |b1| + 40
//     feedforward     = 1 - |b1|/Vref
//     current ref     = Ipre * cos(integral1)
//     slow-leg sign   = sign(b1)
//     zero-cross blank= |b1| < 5
//
// So a phase or amplitude error in b1 lands directly on the duty AND on
// the commutation instant. With a 10uF bus and no bulk storage, a
// commutation that fires while the line still has voltage on it puts the
// full bus across the inductor: di/dt = Vbus/L ~ 128 A/ms, i.e. ~6.4 A
// per 50us ISR cycle of misalignment. That is the mechanism this port is
// meant to reproduce - the ideal-Vac version cannot show it by construction.
//
// USE_PLL is the bisect switch:
//   1 = DSP behaviour      (b1 / integral1 from the PLL below)
//   0 = original PSIM behaviour (true Vac / in[4]) - reference run
//======================================================================
#define USE_PLL 1

//----- carrier / rates -------------------------------------------------
// DSP: EPwm1 TBPRD=2250, up-down, 90MHz TBCLK -> 20kHz, center-aligned.
// The original block ran an 85kHz sawtooth, which gives 4.25x less
// inductor ripple than the hardware. Both are matched here.
#define PWM_FREQ        20000.0
#define CONTROL_FREQ    20000.0
#define CONTROL_FREQ_OUT 60.0
#define Ts      (1.0 / CONTROL_FREQ)
#define Ts_out  (1.0 / CONTROL_FREQ_OUT)
#define CENTER_ALIGNED  1      // 1 = triangle carrier (matches DSP up-down)

//----- PLL loop filter (main.c) ---------------------------------------
#define PLL_KP   2.5
#define PLL_KI   0.5
#define PLL_DT   0.00005

//----- outer voltage loop (main.c VoltageLoop) ------------------------
#define Vkp 0.1
#define Vki 0.01
#define VOUT_TARGET 50.0       // PSIM schematic's output setpoint

//----- current loop (main.c currentloop) ------------------------------
#define CURRENT_LOOP_K        9.3
#define CURRENT_LOOP_VBUS_MIN 10.0
#define CURRENT_LOOP_KP_MIN   0.001

//----- startup shaping (main.c) ---------------------------------------
#define DUTY_SLEW_MAX   0.01
#define DUTY_CAP_START  0.15
#define VREF_MIN        50.0
#define ZC_BLANK_V      5.0    // main.c uses a literal 5 here, not SLOW_LEG_DEAD_V

//======================================================================
// measured inputs
//======================================================================
double Vac = 0, Vbus = 0, Vo = 0, Iac = 0, theta_in = 0;

//======================================================================
// PLL state (main.c PLL())
//======================================================================
int   pll_state = 0;
double a = 0, b = 0, b1 = 0;
double d = 0, q = 0, d1 = 0, q1 = 0;
double c5a = 0, c5b = 0, c5d = 0, c5q = 0;
double s5a = 0, s5b = 0, s5d = 0, s5q = 0;
double cfa = 0, cfb = 0, cfd = 0, cfq = 0;
double sfa = 0, sfb = 0, sfd = 0, sfq = 0;
double cos_value = 0, sin_value = 0;
double error = 0, integral = 0, integral1 = 0, w = 0;
double prevoutput = 0, prevoutput1 = 0;
double z = 0.001568;

//----- PLL lock confirmation (main.c CheckPLLLock) --------------------
int PLL_flag = 0, PLLcount = 0, PLLERRcount = 0;

//----- zero crossing (main.c ZeroCrossDetect) -------------------------
double theta_prev = 0;
int zero_cross_pos = 0, zero_cross_neg = 0;
int zc_now = 0, zc_prev = 0, zero_cross_event = 0;

//======================================================================
// control state
//======================================================================
int system_running = 0, starset_up = 0;
double Vref = 0, Vref_start = 0;
double feedforward = 0, duty = 0, duty_prev_cmd = 0, final_duty = 0;
double Vac_in = 0, Ifb = 0, Iref = 0, Ie = 0, Ikp = 0, Ipi = 0;
double Ic = 0, Ipre = 0, Ipre_prev = 0, Ipre_max = 1;
double Ve = 0, Ve_prev = 0;
double Vo_filt = 0, Vo_prev = 0;
int triggered = 0;

//----- Buck ------------------------------------------------------------
double duty_buck = 0;

//----- carrier / gating ------------------------------------------------
double saw = 0, saw_0 = 0, saw1 = 0, saw_1 = 0;
int tri_dir = 1, tri_dir1 = 1;
int S = 0, R = 0, Q = 0;
int S1 = 0, R1 = 0, Q1 = 0;
int counter20k = 0;

//======================================================================
// PLL - verbatim from main.c, with b taken from the PSIM input in volts
// (the DSP scales raw ADC counts by 0.231343 to get the same units).
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

    error    = -d;
    integral = integral + error * PLL_KI * PLL_DT;
    w        = (error * PLL_KP) + integral + 376.991;

    integral1 = fmod(integral1 + w * PLL_DT, 6.28);

    z  = 0.001568;
    d1 = z * d + (1 - z) * prevoutput;
    prevoutput  = d1;
    q1 = z * q + (1 - z) * prevoutput1;
    prevoutput1 = q1;
}

//======================================================================
static void ZeroCrossDetect(void)
{
    zero_cross_pos = (theta_prev < 1.55 && integral1 >= 1.55);
    zero_cross_neg = (theta_prev < 4.69 && integral1 >= 4.69);

    zc_now = (zero_cross_pos || zero_cross_neg);
    zero_cross_event = (zc_now && !zc_prev);
    zc_prev = zc_now;

    theta_prev = integral1;
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
// Vref - the bus reference the feedforward shapes Vbus onto. Starts AT
// the measured bus and interpolates up to the |b1|+40 envelope, so the
// feedforward is correct from the first switching cycle.
//======================================================================
static void UpdateVref(void)
{
    double ramp = (Ipre_max - 1.0) / 7.0;
    if(ramp < 0.0) ramp = 0.0;
    if(ramp > 1.0) ramp = 1.0;

    if(ramp <= 0.0) Vref_start = Vbus;

#if USE_PLL
    double env = fabs(b1);
#else
    double env = fabs(Vac);
#endif

    double vref_full = (env > 100.0) ? (env + 40.0) : 140.0;

    Vref = Vref_start + (vref_full - Vref_start) * ramp;
    if(Vref < VREF_MIN) Vref = VREF_MIN;
}

//======================================================================
// Outer voltage loop - incremental PI, fired once per line cycle in a
// narrow phase window, plus the Ipre_max soft-start ramp (1 -> 8).
//======================================================================
static void VoltageLoop(void)
{
    if(integral1 > 4.69494 && integral1 < 4.727444)
    {
        if(!triggered)
        {
            Ve   = VOUT_TARGET - Vo_filt;
            Ipre = Ipre_prev + Vkp * (Ve - Ve_prev) + Vki * 0.01667 * Ve;
            Ve_prev   = Ve;
            Ipre_prev = Ipre;
            triggered = 1;
        }
    }
    else
    {
        triggered = 0;
    }

    if(Ipre_max < 8) Ipre_max = Ipre_max + 0.0005;
    if(Ipre_max > 8) Ipre_max = 8;
    if(Ipre > Ipre_max) Ipre = Ipre_max;
    if(Ipre < 1)       Ipre = 1;
}

//======================================================================
// Current loop - pure P plus feedforward, with the dynamic Kp = K/Vbus
// and the startup duty cap / slew limiter.
//======================================================================
static void currentloop(void)
{
    Ic = Ipre * cos_value;

    double vbus_for_ikp = Vbus;
    if(vbus_for_ikp < CURRENT_LOOP_VBUS_MIN) vbus_for_ikp = CURRENT_LOOP_VBUS_MIN;
    Ikp = CURRENT_LOOP_K / vbus_for_ikp;
    if(Ikp < CURRENT_LOOP_KP_MIN) Ikp = CURRENT_LOOP_KP_MIN;

#if USE_PLL
    double vsign = b1;
#else
    double vsign = Vac;
#endif

    if(vsign >= 0) { Vac_in =  vsign; Ifb =  Iac; Iref =  Ic; }
    else           { Vac_in = -vsign; Ifb = -Iac; Iref = -Ic; }

    feedforward = 1 - (Vac_in / Vref);
    Ie   = Iref - Ifb;
    Ipi  = Ikp * Ie;
    duty = feedforward + Ipi;

    double duty_cap = DUTY_CAP_START + (1.0 - DUTY_CAP_START) * ((Ipre_max - 1.0) / 7.0);
    if(duty > duty_cap) duty = duty_cap;

    if(duty >= 0.97) duty = 0.97;
    if(duty <= 0.03) duty = 0.03;

    if(duty > duty_prev_cmd + DUTY_SLEW_MAX) duty = duty_prev_cmd + DUTY_SLEW_MAX;
    if(duty < duty_prev_cmd - DUTY_SLEW_MAX) duty = duty_prev_cmd - DUTY_SLEW_MAX;
    duty_prev_cmd = duty;
}

//======================================================================
static void Hold(void)
{
    starset_up = 0;
    Ipi = 0; Ie = 0; Iref = 0; Ifb = 0; duty = 0;
    Ipre_prev = 0; Ve_prev = 0; Ipre_max = 1; triggered = 0;
    duty_prev_cmd = 0;
}

//======================================================================
void SimulationStep(
        double t, double delt, double *in, double *out,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k++;

    if(in[5] > 0)
    {
        //--------------------------------------------------------------
        // 20kHz control task (the DSP's adc_isr)
        //--------------------------------------------------------------
        if(counter20k >= 100)
        {
            Vac      = in[0];
            Vbus     = in[1];
            Iac      = in[2];
            Vo       = in[3];
            theta_in = in[4];

            // output filter (main.c lowpass)
            Vo_filt = Vo_prev + z * (Vo - Vo_filt);
            Vo_prev = Vo_filt;

            PLL();
            ZeroCrossDetect();
            CheckPLLLock();

#if !USE_PLL
            // reference run: ideal phase straight from the schematic
            integral1 = theta_in;
            cos_value = cos(theta_in);
            b1        = Vac;
            PLL_flag  = 1;
#endif

            // PFC run/stop - start requires a confirmed lock, stop lands
            // on a zero crossing
            if(PLL_flag) system_running = 1;
            else if(zero_cross_event) system_running = 0;

            UpdateVref();

            if(system_running)
            {
                if(zero_cross_event || starset_up)
                {
                    starset_up = 1;
                    VoltageLoop();
                    currentloop();

                    final_duty = duty;
                    if(final_duty > 0.95) final_duty = 0.95;
                    if(final_duty < 0.03) final_duty = 0.03;
                }
                else
                {
                    Hold();
                }
            }
            else
            {
                Hold();
            }

            duty_buck = 0.1;   // Buck preload, as in the current PSIM block
            counter20k = 0;
        }

        //--------------------------------------------------------------
        // carrier
        //--------------------------------------------------------------
#if CENTER_ALIGNED
        saw += PWM_FREQ * 2.0 * delt * tri_dir;
        if(saw >= 1.0) { saw = 1.0; tri_dir = -1; }
        if(saw <= 0.0) { saw = 0.0; tri_dir =  1; }
        saw_0 = saw;

        saw1 += PWM_FREQ * 2.0 * delt * tri_dir1;
        if(saw1 >= 1.0) { saw1 = 1.0; tri_dir1 = -1; }
        if(saw1 <= 0.0) { saw1 = 0.0; tri_dir1 =  1; }
        saw_1 = saw1;
#else
        saw += PWM_FREQ * delt;
        if(saw >= 1.0) saw -= 1.0;
        saw_0 = 1 - saw;

        saw1 += PWM_FREQ * delt;
        if(saw1 >= 1.0) saw1 -= 1.0;
        saw_1 = 1 - saw1;
#endif

        S = (final_duty > saw_0) ? 1 : 0;
        R = (0.99 > saw_0) ? 1 : 0;
        if(S && !R) Q = 1;
        if(!S && R) Q = 0;
        if(S && R)  Q = 1;

        S1 = (duty_buck > saw_1) ? 1 : 0;
        R1 = (0.98 > saw_1) ? 1 : 0;
        if(S1 && !R1) Q1 = 1;
        if(!S1 && R1) Q1 = 0;
        if(S1 && R1)  Q1 = 1;

        //--------------------------------------------------------------
        // Leg drive. Note this is decided by b1 (the reconstruction),
        // NOT by the true Vac - including the zero-cross blanking window,
        // which is what makes the commutation instant follow the PLL's
        // idea of the zero crossing rather than the line's.
        //--------------------------------------------------------------
#if USE_PLL
        double vcomm = b1;
#else
        double vcomm = Vac;
#endif

        if(!starset_up)
        {
            out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 0;
        }
        else if(vcomm < ZC_BLANK_V && vcomm > -ZC_BLANK_V)
        {
            out[0] = 0; out[1] = 0;      // fast leg off through the crossing
            out[2] = 0; out[3] = 0;      // both slow-leg arms off
        }
        else if(vcomm > 0)
        {
            out[0] = Q;   out[1] = 1 - Q;
            out[2] = 0;   out[3] = 1;
        }
        else
        {
            out[0] = 1 - Q; out[1] = Q;
            out[2] = 1;     out[3] = 0;
        }
    }
    else
    {
        out[0] = 0; out[1] = 0; out[2] = 0; out[3] = 0;
    }

    out[4]  = Q1;
    out[5]  = 1 - Q1;
    out[6]  = Ie;
    out[7]  = Iref;
    out[8]  = Ifb;
    out[9]  = feedforward;
    out[10] = duty_buck;
    out[11] = Vo;
    out[12] = Vo_filt;
    out[13] = final_duty;
    out[14] = b1;            // <-- compare these two: the whole point
    out[15] = Vac;           // <-- true line vs reconstruction
    out[16] = cos_value;
    out[17] = Ipre;
    out[18] = PLL_flag;
    out[19] = Ikp;
    out[20] = Vref;
}

//======================================================================
void SimulationBegin(
        const char *szId, int nInputCount, int nOutputCount,
        int nParameterCount, const char ** pszParameters,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k = 0;

    // main.c seeds the case-0 rotation constants here
    c5d = c5a = cos(0.08722);
    c5q = c5b = cos(0.08722);
    s5d = s5a = sin(0.08722);
    s5q = s5b = sin(0.08722);

    pll_state = 0;
    integral1 = 0; integral = 0; w = 0;
    d = q = d1 = q1 = a = b = b1 = 0;
    prevoutput = prevoutput1 = 0;
    PLL_flag = 0; PLLcount = 0; PLLERRcount = 0;

    system_running = 0; starset_up = 0;
    Ipre = 0; Ipre_prev = 0; Ipre_max = 1;
    Ve = 0; Ve_prev = 0; triggered = 0;
    Vo_filt = 0; Vo_prev = 0;
    duty = 0; final_duty = 0; duty_prev_cmd = 0; duty_buck = 0;
    saw = 0; saw1 = 0; tri_dir = 1; tri_dir1 = 1;
    Vref = 0; Vref_start = 0;
}

//======================================================================
void SimulationEnd(const char *szId, void ** reserved_UserData,
                   int reserved_ThreadIndex, void * reserved_AppPtr)
{
}
