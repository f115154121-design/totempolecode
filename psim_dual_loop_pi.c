#include <Stdlib.h>
#include <String.h>
#include <math.h>
#include <Psim.h>

//======================================================================
// PSIM C-block: cascaded (dual-loop) PI for the Buck stage.
//
//   outer voltage PI : Ve = Vo_ref - Vo   ->  Iref  (A)
//   inner current PI : Ie = Iref   - iL   ->  duty  (0..1)
//   feedforward      : d_ff = Vo_ref / Vbus
//
// Plant signs (see docs/control_analysis/README.md section 5):
//   dVo/dd = +Vbus  and  diL/dd = +Vbus/(s*Lb)   -> BOTH loops use "+".
//   Error is always ref - fb, so the controller sign must be + to keep
//   the loop gain L = C*P positive (otherwise it is positive feedback).
//
// Both PI outputs are divided by Vbus, so the gains below do not depend
// on the bus voltage:
//   inner:  L_i(s) = (Ikp + Iki/s) / (s*Lb)
//           Ikp = 2*pi*f_ci*Lb        [V/A]    f_ci = inner crossover
//           Iki = Ikp*2*pi*(f_ci/5)   [V/A/s]  PI zero below f_ci
//   outer:  with the current loop closed, iL ~ Iref, so
//           Vo/Iref = R/(1 + s*R*Co), which is 1/(s*Co) above the pole
//           Vkp = 2*pi*f_cv*Co        [A/V]
//           Vki = Vkp*2*pi*(f_cv/5)   [A/V/s]
//
// Numbers below: Lb = 75 uH, Co = 2200 uF (f0 = 392 Hz, Z0 = 0.185 ohm),
// fs = 20 kHz, f_ci = 2 kHz (fs/10), f_cv = 20 Hz.
//
// f_cv is deliberately far below 120 Hz: the PFC stage leaves ~6 Vpp of
// 120 Hz ripple on Vo and the PFC voltage loop already integrates the
// energy balance. A fast Buck voltage loop would fight it (two
// integrators on one energy state) and would also try to cancel the
// ripple, which distorts the line current. For a stand-alone Buck fed
// from a stiff DC bus, f_cv = f_ci/10 = 200 Hz is the usual choice.
//
// I/O map (set the C-block to 5 inputs / 4 outputs):
//   in[0] = Vo       output voltage       [V]
//   in[1] = iL       inductor current     [A]
//   in[2] = Vbus     input (bus) voltage  [V]
//   in[3] = enable   > 0 runs, <= 0 holds and resets
//   in[4] = Vo_ref   external setpoint [V]; <= 0 falls back to VOUT_TARGET
//   out[0] = duty    high-side duty (compare against your carrier)
//   out[1] = Iref    inner-loop command   [A]
//   out[2] = Ve      voltage error        [V]
//   out[3] = Ie      current error        [A]
//======================================================================

//----- rates -----------------------------------------------------------
#define CONTROL_FREQ      20000.0            // inner loop = switching rate
#define OUTER_DIV         10                 // outer loop = 2 kHz
#define Ts                (1.0/CONTROL_FREQ)
#define Ts_out            (Ts*OUTER_DIV)

//----- plant (reference only; the gains below are hard numbers) --------
#define LB                75.0e-6
#define CO                2200.0e-6

//----- setpoint --------------------------------------------------------
#define VOUT_TARGET       48.0
#define VOUT_SLEW         (48.0/0.050)       // soft-start: 0 -> 48 V in 50 ms

//----- outer voltage PI: Vkp = 2*pi*20*2200u, Vki = Vkp*2*pi*4 --------
#define Vkp               0.2765             // A/V
#define Vki               6.95               // A/(V*s)
#define IREF_MAX          12.0               // outer clamp = current limit
#define IREF_MIN          0.0                // 0 for a unidirectional load

//----- inner current PI: Ikp = 2*pi*2000*75u, Iki = Ikp*2*pi*400 ------
#define Ikp               0.9425             // V/A
#define Iki               2369.0             // V/(A*s)
#define DUTY_MAX          0.95
#define DUTY_MIN          0.00

//----- numeric guards --------------------------------------------------
#define VBUS_MIN          20.0               // keeps 1/Vbus bounded at start
#define DUTY_SLEW_MAX     0.05               // per inner tick

//======================================================================
// measured inputs / state
//======================================================================
double Vo = 0, iL = 0, Vbus = 0, Vo_ref_in = 0;
int    enable = 0;

double Vo_ref = 0;                           // after the soft-start ramp
double Ve = 0, Vki_int = 0, Iref = 0, Iref_unsat = 0;
double Ie = 0, Iki_int = 0, duty = 0, duty_unsat = 0, duty_prev = 0;
double d_ff = 0, vbus_lim = 0;

double t_ctrl = 0;                           // control-task timer
int    outer_count = 0;

//======================================================================
// Outer voltage loop - PI with clamping anti-windup.
//   Iref = Vkp*Ve + Vki*INT(Ve), saturated to [IREF_MIN, IREF_MAX]
// The integrator is frozen whenever the output is already saturated AND
// the error would push it further out (conditional integration), so the
// integral state cannot run away while the current limit is active.
//======================================================================
static void VoltageLoop(void)
{
    Ve = Vo_ref - Vo;

    Iref_unsat = Vkp*Ve + Vki_int;

    Iref = Iref_unsat;
    if(Iref > IREF_MAX) Iref = IREF_MAX;
    if(Iref < IREF_MIN) Iref = IREF_MIN;

    if(!((Iref_unsat > IREF_MAX && Ve > 0.0) ||
         (Iref_unsat < IREF_MIN && Ve < 0.0)))
    {
        Vki_int += Vki*Ts_out*Ve;            // backward Euler
    }
}

//======================================================================
// Inner current loop - PI plus Vo_ref/Vbus feedforward, both divided by
// Vbus so the loop gain is bus-independent:
//   duty = Vo_ref/Vbus + (Ikp*Ie + Iki*INT(Ie))/Vbus
// Same conditional-integration anti-windup as the outer loop.
//======================================================================
static void CurrentLoop(void)
{
    vbus_lim = (Vbus < VBUS_MIN) ? VBUS_MIN : Vbus;

    d_ff = Vo_ref / vbus_lim;
    Ie   = Iref - iL;

    duty_unsat = d_ff + (Ikp*Ie + Iki_int)/vbus_lim;

    duty = duty_unsat;
    if(duty > DUTY_MAX) duty = DUTY_MAX;
    if(duty < DUTY_MIN) duty = DUTY_MIN;

    if(!((duty_unsat > DUTY_MAX && Ie > 0.0) ||
         (duty_unsat < DUTY_MIN && Ie < 0.0)))
    {
        Iki_int += Iki*Ts*Ie;
    }

    if(duty > duty_prev + DUTY_SLEW_MAX) duty = duty_prev + DUTY_SLEW_MAX;
    if(duty < duty_prev - DUTY_SLEW_MAX) duty = duty_prev - DUTY_SLEW_MAX;
    duty_prev = duty;
}

//======================================================================
// Bumpless start: the ramp starts from the voltage that is already
// there and the duty starts at the feedforward value, so handing over
// to the loop does not kick (same reasoning as the Vout_ref-follows-Vo
// fix in phaselock/main.c).
//======================================================================
static void Arm(void)
{
    Vo_ref    = Vo;
    Vki_int   = 0;
    Iki_int   = 0;
    Iref      = iL;
    duty_prev = (Vbus > VBUS_MIN) ? (Vo / Vbus) : 0.0;
}

static void Hold(void)
{
    Vo_ref = 0; Ve = 0; Vki_int = 0; Iref = 0; Iref_unsat = 0;
    Ie = 0; Iki_int = 0; duty = 0; duty_unsat = 0; duty_prev = 0;
    outer_count = 0;
}

//======================================================================
void SimulationStep(
        double t, double delt, double *in, double *out,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    int enable_now;
    double target;

    Vo        = in[0];
    iL        = in[1];
    Vbus      = in[2];
    Vo_ref_in = in[4];

    enable_now = (in[3] > 0.0) ? 1 : 0;

    if(enable_now && !enable) Arm();
    if(!enable_now)           Hold();
    enable = enable_now;

    if(enable)
    {
        // fixed-rate control task, independent of the PSIM time step
        t_ctrl += delt;
        if(t_ctrl >= Ts)
        {
            t_ctrl -= Ts;

            // setpoint + soft-start ramp
            target = (Vo_ref_in > 0.0) ? Vo_ref_in : VOUT_TARGET;
            if(Vo_ref < target)
            {
                Vo_ref += VOUT_SLEW*Ts;
                if(Vo_ref > target) Vo_ref = target;
            }
            else
            {
                Vo_ref -= VOUT_SLEW*Ts;
                if(Vo_ref < target) Vo_ref = target;
            }

            // the outer loop runs OUTER_DIV times slower than the inner
            if(++outer_count >= OUTER_DIV)
            {
                outer_count = 0;
                VoltageLoop();
            }

            CurrentLoop();
        }
    }

    out[0] = duty;
    out[1] = Iref;
    out[2] = Ve;
    out[3] = Ie;
}


/////////////////////////////////////////////////////////////////////
// FUNCTION: SimulationBegin
// DO NOT CHANGE THE NAME OR PARAMETERS OF THIS FUNCTION
void SimulationBegin(
        const char *szId, int nInputCount, int nOutputCount,
        int nParameterCount, const char ** pszParameters,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    Vo = 0; iL = 0; Vbus = 0; Vo_ref_in = 0; enable = 0;
    t_ctrl = 0;
    Hold();
}


/////////////////////////////////////////////////////////////////////
// FUNCTION: SimulationEnd
// DO NOT CHANGE THE NAME OR PARAMETERS OF THIS FUNCTION
void SimulationEnd(const char *szId, void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
}
