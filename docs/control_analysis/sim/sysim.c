// sysim.c - 整體系統平均模型模擬（totem-pole PFC + 10uF tracking bus + 同步 Buck + 2200uF + 電阻負載）
// 控制律逐拍照抄 phaselock/main.c @ 87093bc：currentloop() / VoltageLoop() / Busloop() / UpdateVref() / lowpass()
//
// 編譯： gcc -O2 -o sysim sysim.c -lm
// 執行： ./sysim Vkp Vki K 輸出前綴 [Tend] [t1 R1] [t2 R2]
//   例： SLEW=1 ./sysim 0.03 1 9.3 run 9          -> 1 s 時 20->40 ohm，5 s 時回到 20 ohm
// 環境變數：
//   LB=632e-6   PFC 電感（預設 1.5658e-3，程式註解中的量測值）
//   SLEW=0.01   duty 每拍斜率上限（預設 0.01 = 現行 DUTY_SLEW_MAX；設 1 等於不限制）
//   NODELAY=1   拿掉一拍計算延遲（模擬「PSIM C-block 沒加延遲」的情況）
// 輸出：
//   <前綴>_trend.csv  每 5 ms 一筆：t, Vo, Vo_filt, Ipre, (Vbus-Vref) RMS
//   <前綴>_wave.csv   第一次負載跳變前兩個 line cycle 的逐拍波形（給 THD/PF 用，見 analysis.py thd）
// 模型假設：開關以平均模型表示、PLL/感測器理想、|b1|<5V 時兩臂關閉（電流只能單向經二極體）、
//   DCR 50 mOhm (boost) / 30 mOhm (buck)、ESR 20 mOhm、RK4，每拍 25 個子步。
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char **argv) {
    if (argc < 5) { fprintf(stderr, "usage: sysim Vkp Vki K prefix [Tend] [t1 R1] [t2 R2]\n"); return 1; }
    double Vkp = atof(argv[1]), Vki = atof(argv[2]), K = atof(argv[3]);
    const char *pre = argv[4];
    double Tend = argc > 5 ? atof(argv[5]) : 9.0;
    double ts1 = argc > 7 ? atof(argv[6]) : 1.0, R1 = argc > 7 ? atof(argv[7]) : 40.0;
    double ts2 = argc > 9 ? atof(argv[8]) : 5.0, R2 = argc > 9 ? atof(argv[9]) : 20.0;

    const double PI = 3.14159265358979;
    const double Vm = 115.0 * sqrt(2.0), f_line = 60.0, w = 2 * PI * f_line;
    const double L = getenv("LB")?atof(getenv("LB")):1.5658e-3, rLb = 0.05, Cbus = 10e-6;
    const double Lb = 75e-6, rL = 0.03, Co = 2200e-6, rC = 0.02;
    const double Ts = 50e-6; const int SUB = 25; const double dt = Ts / SUB;
    const double zf = 0.001568, VOSTAR = 100.0, VOREF = 100.0;
    double R = 20.0;
    double SLEW = getenv("SLEW") ? atof(getenv("SLEW")) : 0.01;
    int NODELAY = getenv("NODELAY") ? 1 : 0;

    // states (start near steady state)
    double i = 0, Vbus = 140, ib = 5.0, vC = 100.0;
    double Vo_filt = 100.0, Ipre = 2 * 500.0 / Vm, Ipre_prev = Ipre, Ve_prev = 0;
    int dead_app = 0, dead_next = 0; double d_app = 0.5, d_next = 0.5, db_app = 100.0 / 140, db_next = db_app, duty_prev = 0.5;
    int fired = 0;

    char fn[256];
    sprintf(fn, "%s_trend.csv", pre); FILE *ft = fopen(fn, "w");
    sprintf(fn, "%s_wave.csv", pre);  FILE *fw = fopen(fn, "w");
    fprintf(ft, "t,Vo,Vo_filt,Ipre,Vbus_minus_Vref_rms\n");
    fprintf(fw, "t,vac,iin,iref,Vbus,Vref,Vo,ib,d,db\n");

    long N = (long)(Tend / Ts);
    double accE2 = 0; int accN = 0;
    // for THD: record one full cycle of iin at steady state before first step
    double wave_t0 = ts1 - 2.0 / f_line, wave_t1 = ts1;
    for (long k = 0; k < N; k++) {
        double t = k * Ts;
        if (t >= ts1) R = R1;
        if (t >= ts2) R = R2;
        double th = w * t, vac = Vm * sin(th), av = fabs(vac);
        double vo = (vC + rC * ib) / (1 + rC / R);
        // ---- sampled measurements & control (computed now, applied next period) ----
        Vo_filt = Vo_filt + zf * (vo - Vo_filt);
        double b1 = vac;
        double Vref = fabs(b1) > 100 ? fabs(b1) + 40 : 140;
        // voltage loop: once per line cycle at fixed phase (~3pi/2)
        double ph = fmod(th, 2 * PI);
        if (ph > 4.69494 && ph < 4.727444) {
            if (!fired) {
                double Ve = VOREF - Vo_filt;
                Ipre = Ipre_prev + Vkp * (Ve - Ve_prev) + Vki * 0.01667 * Ve;
                Ve_prev = Ve; Ipre_prev = Ipre; fired = 1;
                if (Ipre > 8) Ipre = 8; if (Ipre < 1) Ipre = 1;
            }
        } else fired = 0;
        // current loop
        double Iref = Ipre * fabs(sin(th));
        double ifb = i;                       // rectified-frame inductor current
        double vbm = Vbus < 10 ? 10 : Vbus;
        double Ikp = K / vbm;
        double duty = 1 - av / Vref + Ikp * (Iref - ifb);
        if (duty > 0.98) duty = 0.98; if (duty < 0.05) duty = 0.05;
        if (duty > duty_prev + SLEW) duty = duty_prev + SLEW;
        if (duty < duty_prev - SLEW) duty = duty_prev - SLEW;
        duty_prev = duty;
        double fd = duty; int dead = fabs(b1) < 5;  if (fd > 0.95) fd = 0.95; if (fd < 0.03) fd = 0.03;
        // buck feedforward (open loop, as Busloop)
        double db = VOSTAR / Vref; if (db > 0.9) db = 0.9; if (db < 0.1) db = 0.1;
        if(dead) fd = 0.0;
        if(NODELAY){ d_app=fd; db_app=db; dead_app=dead; } else { d_app = d_next; d_next = fd; db_app = db_next; db_next = db; dead_app = dead_next; dead_next = dead; }
        // logging
        if (t >= wave_t0 && t < wave_t1)
            fprintf(fw, "%.6f,%.3f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f\n", t, vac, (vac >= 0 ? i : -i), (vac >= 0 ? Iref : -Iref), Vbus, Vref, vo, ib, d_app, db_app);
        accE2 += (Vbus - Vref) * (Vbus - Vref); accN++;
        if (k % 100 == 0) { fprintf(ft, "%.4f,%.4f,%.4f,%.4f,%.3f\n", t, vo, Vo_filt, Ipre, sqrt(accE2 / accN)); accE2 = 0; accN = 0; }
        // ---- plant integration ----
        for (int s = 0; s < SUB; s++) {
            double tt = t + s * dt;
            double x[4] = {i, Vbus, ib, vC}, k1[4], k2[4], k3[4], k4[4], y[4];
            #define F(T_, X, OUT) { double avs = fabs(Vm * sin(w * (T_))); \
                double vos = ((X)[3] + rC * (X)[2]) / (1 + rC / R); \
                OUT[0] = (avs - (1 - d_app) * (X)[1] - rLb * (X)[0]) / L; if(dead_app && (X)[0] <= 0 && OUT[0] < 0) OUT[0] = 0; \
                OUT[1] = ((1 - d_app) * (X)[0] - db_app * (X)[2]) / Cbus; \
                OUT[2] = (db_app * (X)[1] - vos - rL * (X)[2]) / Lb; \
                OUT[3] = ((X)[2] - vos / R) / Co; }
            F(tt, x, k1);
            for (int q = 0; q < 4; q++) y[q] = x[q] + 0.5 * dt * k1[q]; F(tt + 0.5 * dt, y, k2);
            for (int q = 0; q < 4; q++) y[q] = x[q] + 0.5 * dt * k2[q]; F(tt + 0.5 * dt, y, k3);
            for (int q = 0; q < 4; q++) y[q] = x[q] + dt * k3[q]; F(tt + dt, y, k4);
            for (int q = 0; q < 4; q++) x[q] += dt / 6 * (k1[q] + 2 * k2[q] + 2 * k3[q] + k4[q]);
            i = x[0]; if(dead_app && i<0) i=0; Vbus = x[1]; ib = x[2]; vC = x[3];
        }
    }
    fclose(ft); fclose(fw);
    return 0;
}
