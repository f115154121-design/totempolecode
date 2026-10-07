"""analysis.py - 小訊號模型數值計算（對應 ../README.md 各節）

用法：
  python analysis.py current            # §1 電流環 fc / PM / GM，掃 K
  python analysis.py coupling           # §2 bus 耦合：凍結點特徵值，求各 |vac| 下的 K 上限
  python analysis.py voltage            # §4 PFC 電壓環（z 域）裕度
  python analysis.py buck               # §5 Buck 迴路裕度（理想 vs 含寄生）
  python analysis.py thd <prefix>       # 讀 sysim 的 <prefix>_wave.csv，算 THD / PF
  python analysis.py all

需要：numpy、scipy
"""
import sys
import numpy as np
from scipy import signal
from scipy.linalg import expm

# ---------------- 參數（phaselock/main.c @ 87093bc 與硬體值） ----------------
Vm = 115 * np.sqrt(2)          # 線電壓峰值
Ts = 50e-6                     # ISR 週期（20 kHz）
Td = 1.5 * Ts                  # 一拍計算延遲 + 半拍 ZOH
L_PFC = 1.5658e-3              # 程式註解的量測值；另一個候選值 632e-6
K_I = 9.3                      # CURRENT_LOOP_K
Cbus = 10e-6
Lb, rLb_buck, Co, rC = 75e-6, 0.03, 2200e-6, 0.02
rL_boost = 0.05
Vo = 100.0
R = 20.0                       # 500 W
tau_f = Ts / 0.001568          # lowpass() 的時間常數 31.9 ms
T_line = 1 / 60


def margins(f, T):
    m = 20 * np.log10(np.abs(T))
    ph = np.unwrap(np.angle(T)) * 180 / np.pi
    r = {}
    i = np.where(np.diff(np.sign(m)) < 0)[0]
    if len(i):
        r["fc_Hz"] = round(float(f[i[0]]), 4)
        r["PM_deg"] = round(float(180 + ph[i[0]]), 1)
    j = np.where(np.diff(np.sign(ph + 180)) < 0)[0]
    if len(j):
        r["f180_Hz"] = round(float(f[j[0]]), 2)
        r["GM_dB"] = round(float(-m[j[0]]), 1)
    return r


# ---------------- §1 電流環：T = K/(sL) e^{-sTd} ----------------
def current(L=L_PFC):
    f = np.logspace(0, 4.5, 6000)
    s = 2j * np.pi * f
    print(f"L = {L*1e3:.4g} mH")
    for K in [3, 5, 7, 9.3, 12, 15]:
        print(f"  K={K:<5} analytic fc={K/(2*np.pi*L):7.1f} Hz ", margins(f, K / (s * L) * np.exp(-s * Td)))
    print(f"  1/(4Td) = {1/(4*Td):.0f} Hz ; 單看電流環 K_max ≈ {2*np.pi*L/(4*Td):.1f}")


# ---------------- §2 bus 耦合：凍結點離散特徵值 ----------------
def frozen_rho(K, vac, L=L_PFC):
    Vref = abs(vac) + 40 if abs(vac) > 100 else 140
    D = 1 - abs(vac) / Vref
    db = Vo / Vref
    i0 = 500 / abs(vac) if abs(vac) > 1 else 0
    k = 1 / (1 + rC / R)
    # x = [i_L, Vbus, i_buck, vC]，輸入 = 儲能 duty
    A = np.array([
        [-rL_boost / L, -(1 - D) / L, 0, 0],
        [(1 - D) / Cbus, 0, -db / Cbus, 0],
        [0, db / Lb, (-rLb_buck - k * rC) / Lb, -k / Lb],
        [0, 0, (1 - k * rC / R) / Co, -(k / R) / Co]])
    B = np.array([[Vref / L], [-i0 / Cbus], [0], [0]])
    M = np.zeros((5, 5)); M[:4, :4] = A; M[:4, 4:] = B
    E = expm(M * Ts); Ad, Bd = E[:4, :4], E[:4, 4:]
    Aaug = np.zeros((5, 5)); Aaug[:4, :4] = Ad; Aaug[:4, 4:] = Bd
    Aaug[4, 0] = -K / Vref            # d[n+1] = -(K/Vbus) i[n]  （一拍延遲）
    return max(abs(np.linalg.eigvals(Aaug)))


def coupling():
    for L in [L_PFC, 632e-6]:
        print(f"L = {L*1e3:.4g} mH")
        for vac in [20, 60, 100, 130, 163]:
            ks = np.arange(1, 40, 0.25)
            kmax = next((K for K in ks if frozen_rho(K, vac, L) > 1), None)
            print(f"  |vac|={vac:>3} V  K_max ≈ {kmax}   rho(K=9.3) = {frozen_rho(9.3, vac, L):.4f}")
    for Vref in [140, 170, 203]:
        db = Vo / Vref
        print(f"  Vref={Vref}  f_bus = {db/(2*np.pi*np.sqrt(Lb*Cbus)):.0f} Hz")


# ---------------- §4 PFC 電壓環（每 line cycle 取樣一次） ----------------
def voltage():
    for load in ["R", "CPL"]:
        Gd = Vm**2 / (2 * K_I * Vo) + (2 * Vo / R if load == "R" else 0.0)
        num = [Vm / 2]
        den = np.polymul([Co * Vo, Gd], [tau_f, 1])
        nd, dd, _ = signal.cont2discrete((num, den), T_line, method="zoh")
        nd = np.squeeze(nd)
        print(f"load={load}  G_d={Gd:.2f} W/V  pole={Gd/(2*np.pi*Co*Vo):.2f} Hz")
        for Vkp, Vki in [(0.1, 0.01), (0.03, 1), (0.1, 2)]:
            nL = np.polymul(nd, [Vkp + Vki * T_line, -Vkp])
            dL = np.polymul(dd, [1, -1])
            f = np.logspace(-3, np.log10(29.99), 20000)
            z = np.exp(1j * 2 * np.pi * f * T_line)
            Lz = np.polyval(nL, z) / np.polyval(dL, z)
            rho = max(abs(np.roots(np.polyadd(dL, nL))))
            print(f"  Vkp={Vkp:<5} Vki={Vki:<5}", margins(f, Lz), f"max|pole|={rho:.4f}")


# ---------------- §5 Buck ----------------
def buck():
    f = np.logspace(0, 4, 6000)
    s = 2j * np.pi * f
    Vbus = 170.0
    for rc, rl in [(0, 0), (rC, rLb_buck)]:
        Zc = 1 / (s * Co) + rc
        Zp = Zc * R / (Zc + R)
        G = Vbus * Zp / (s * Lb + rl + Zp)
        Z0 = np.sqrt(Lb / Co)
        Q = 1 / (Z0 / R + (rc + rl) / Z0)
        for Kp, Ki in [(0.05, 100), (0.2, 300)]:
            T = (Kp + Ki / s) / Vbus * G * np.exp(-s * Td)
            print(f"rC+rL={1e3*(rc+rl):.0f} mOhm Q={Q:.1f}  Kp={Kp} Ki={Ki}", margins(f, T))


# ---------------- THD / PF ----------------
def thd(prefix):
    w = np.genfromtxt(prefix + "_wave.csv", delimiter=",", names=True)
    n = int(round(T_line / Ts))
    x, v = w["iin"][-n:], w["vac"][-n:]
    X = np.fft.rfft(x)
    t = np.sqrt(np.sum(np.abs(X[2:40]) ** 2)) / np.abs(X[1])
    P = np.mean(x * v)
    S = np.sqrt(np.mean(x * x)) * np.sqrt(np.mean(v * v))
    print(f"THD {t*100:.2f}%  PF {P/S:.4f}  Pin {P:.1f} W")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd == "thd":
        thd(sys.argv[2])
    else:
        for name, fn in [("current", current), ("coupling", coupling), ("voltage", voltage), ("buck", buck)]:
            if cmd in (name, "all"):
                print(f"===== {name} =====")
                fn()
