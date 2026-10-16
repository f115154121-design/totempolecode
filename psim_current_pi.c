#include <Stdlib.h>
#include <String.h>
#include <math.h>
#include <Psim.h>

// ============================================================
//  PFC 電流環加入 Ki 的測試版本
//  改動只在電流環,其餘(鎖相/電壓外環/Buck)與原檔相同
//
//  老師的離散推導:
//     Kp + Ki/s = Ki*(s/wz + 1)/s ,  wz = Ki/Kp
//     s -> (z-1)/Ts
//     C(z) = [Kp(z-1) + Ki*Ts]/(z-1)
//     u[n] = u[n-1] + Kp*(e[n]-e[n-1]) + Ki*Ts*e[n]
//
//  本檔用等價的「位置型」寫法(P 項 + 獨立積分狀態),
//  原因:Kp = K/Vbus 而 Vbus 跟著包絡在 140~200V 擺動,
//  速度型會把舊 Kp 算出來的比例量凍在狀態裡,位置型每拍重算,
//  對變增益才正確。穩態行為兩者完全相同。
// ============================================================

#define PI_CONST    3.14159265358979
#define TWO_PI      (2.0 * PI_CONST)

// ---- 電流環兩個旋鈕 ----
#define I_LOOP_K    7.0      // Kp = I_LOOP_K / Vbus   (原檔 9.3, DSP 現為 7.0)
#define I_LOOP_FZ   360.0    // PI 零點 [Hz], 老師給 360。設 0 = 退回純 P
#define I_SUM_LIM   1.0      // 積分器硬上限

// === 系統量測輸入 ===
float Vac = 0, Vbus = 0, Vo = 0, Iac = 0;
float theta = 0;

// === 控制器輸出與中間變數 ===
float duty = 0, duty_buck = 0;
float duty_raw = 0;
float feedforward = 0;
float feedforward_buck = 0;
float saw = 0, saw_0 = 0;
float saw1 = 0, saw_1 = 0;
float Vref = 0;
float kp = 0;

// === 電流環 PI ===
float Iref = 0, Ifb = 0, Ie = 0;
float Ip = 0, Ii = 0;
float I_sum = 0;
float Iki_Ts = 0;
float pifcL = 0;

// === 電壓外環 (注意:下面的 Ikp/Iki 是「電壓外環」的增益,沿用原檔命名) ===
float Vo_fb = 0, Vo_filiter = 0, Vo_filiter_1 = 0;
float Vc = 0, Vc_1 = 0;
float Vpi = 0, Vpi_1 = 0;
float phase = 0;
int VfbLock = 0;

// === Buck ===
float Vbus_soft = 30;
float Ve = 0, Vkp = 0.0013;

// === 其他 ===
float alpha = 0, fc = 20, wc = 2*3.1415926*20;
int counter20k = 0;
int counter60hz = 0;
int load = 0;

// SR 觸發器
int S = 0, R = 0, Q = 0;
int S1 = 0, R1 = 0, Q1 = 0;

// === 頻率設定 ===
#define PWM_FREQ 85000.0
#define CONTROL_FREQ 20000.0
#define CONTROL_FREQ_OUT 60.0
#define Ts (1.0 / CONTROL_FREQ)
#define Ts_out (1.0 / CONTROL_FREQ_OUT)

#define Vkp_Buck 0.001
#define Vki_Buck 0.0001
#define Ikp 0.1      // 電壓外環 Kp
#define Iki 0.5      // 電壓外環 Ki

void SimulationStep(
        double t, double delt, double *in, double *out,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k++;
    counter60hz++;
if(in[5] > 0)
{
    // 1. 電壓外環 (60Hz)
    if(theta > 4.71 && theta < 4.73)
    {
        if(VfbLock == 0)
        {
            Vc_1 = Vc;
            Vc = (100 - Vo_filiter_1);
            Vpi_1 = Vpi;
            Vpi = Vpi_1 + (Ikp + Iki*Ts_out)*Vc - Ikp*Vc_1;
        }
        VfbLock = 1;
    }
    else
    {
        VfbLock = 0;
    }

    if(Vpi < 0) Vpi = 0;

    phase = cos(theta);
    Iref = fabs(Vpi * phase);

    // 2. 20kHz 主迴路
    if(counter20k >= 100)
    {
        Vac = in[0];
        Vbus = in[1];
        Iac = in[2];
        Vo  = in[3];
        Vo_fb = in[3];
        theta = in[4];

        alpha = (wc * 5E-5) / (1 + wc * 5E-5);
        Vo_filiter = Vo_filiter_1 + alpha * (Vo_fb - Vo_filiter_1);
        Vo_filiter_1 = Vo_filiter;

        if(100 < fabs(Vac)) Vref = fabs(Vac) + 40;
        else Vref = 140;

        feedforward = 1 - (fabs(Vac) / Vref);

        float current_Iac = Iac;
        if(Vac < 0) current_Iac = -Iac;
        Ifb = current_Iac;
        Ie = Iref - Ifb;

        // ---------- 電流環 PI ----------
        pifcL = I_LOOP_K;
        float Vbus_Safe = (Vbus < 10.0) ? 10.0 : Vbus;

        kp     = pifcL / Vbus_Safe;                              // Kp   = K/Vbus
        Iki_Ts = TWO_PI * I_LOOP_FZ * pifcL * Ts / Vbus_Safe;    // Ki*Ts = wz*Kp*Ts
        if(kp < 0) kp = 0;

        I_sum += Iki_Ts * Ie;
        if(I_sum >  I_SUM_LIM) I_sum =  I_SUM_LIM;
        if(I_sum < -I_SUM_LIM) I_sum = -I_SUM_LIM;

        Ip = kp * Ie;
        Ii = I_sum;

        duty_raw = feedforward + Ip + Ii;

        duty = duty_raw;
        if(duty > 0.98) duty = 0.98;
        if(duty < 0) duty = 0;

        // 反算抗積分飽和:被限幅削掉多少就從積分器退回多少
        I_sum += (duty - duty_raw);
        if(I_sum >  I_SUM_LIM) I_sum =  I_SUM_LIM;
        if(I_sum < -I_SUM_LIM) I_sum = -I_SUM_LIM;
        // --------------------------------

        Ve = Vref - Vbus;
        feedforward_buck = Vbus_soft / Vref;
        Vbus_soft = Vbus_soft + 0.01;
        if(Vbus_soft > 100) Vbus_soft = 100;
        duty_buck = feedforward_buck - Ve*Vkp;

        counter20k = 0;
    }

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
    if(S && R) Q = 1;

    S1 = (duty_buck > saw_1) ? 1 : 0;
    R1 = (0.98 > saw_1) ? 1 : 0;
    if(S1 && !R1) Q1 = 1;
    if(!S1 && R1) Q1 = 0;
    if(S1 && R1) Q1 = 1;

    if(Vac < 0)
    {
        out[0] = 1-Q;
        out[1] = Q;
        out[2] = 1;
        out[3] = 0;
    }
    else
    {
        out[0] = Q;
        out[1] = 1-Q;
        out[2] = 0;
        out[3] = 1;
    }
}
    out[4] = Q1;
    out[5] = 1-Q1;
    out[6] = Ie;
    out[7] = Iref;
    out[8] = Ifb;
    out[9] = feedforward;
    out[10] = feedforward_buck;
    out[11] = Vo;
    out[12] = Vo_filiter;
    out[13] = duty;
    out[14] = duty_buck;
    out[15] = VfbLock;
    out[16] = cos(theta);
    out[17] = Vpi;
    out[18] = Vc;
    out[19] = kp;
    out[20] = Vref;
    out[21] = Ip;       // 比例項
    out[22] = Ii;       // 積分項 (要把 C-block 輸出數從 21 改成 23)
}

void SimulationBegin(
        const char *szId, int nInputCount, int nOutputCount,
        int nParameterCount, const char ** pszParameters,
        int *pnError, char * szErrorMsg,
        void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
    counter20k = 0;
    counter60hz = 0;
    Ii = 0;
    Ip = 0;
    I_sum = 0;
    Iki_Ts = 0;
    saw = 0;
    saw1 = 0;
    duty = 0;
    duty_buck = 0;
}

void SimulationEnd(const char *szId, void ** reserved_UserData, int reserved_ThreadIndex, void * reserved_AppPtr)
{
}
