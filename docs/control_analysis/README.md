# PFC＋Buck 控制迴路推導與驗證

分析對象：`phaselock/main.c` @ commit `87093bc`（極性修正與正半周啟動）
分析日期：2026-10-06
互動版（可調參數的 Bode 圖、模擬波形）：用瀏覽器打開同資料夾的 `pfc-buck-loops.html`

本文件只做分析，沒有修改任何韌體程式。以下的「建議」都還沒套用。

---

## 重點結論

| # | 發現 | 證據 | 相關程式 |
|---|---|---|---|
| 1 | 電流環 `CURRENT_LOOP_K = 9.3` 卡在穩定邊緣。單看電流環 K 上限約 33，但 10 µF bus 與 Buck 電感的諧振（2.9–4.2 kHz）加上 DSP 延遲，把實際上限壓到約 8.5–12 | §2 凍結點特徵值；整體模擬 K=9.3 可運作（THD 4.5%）、K=10 失穩（THD 38%） | `currentloop()`、`CURRENT_LOOP_K` |
| 2 | `DUTY_SLEW_MAX = 0.01` 追不上前饋。零交越附近前饋每拍需要變化 0.022 | §3；slew=0.01 時 THD 86%、PF 0.21；≥0.025 時 THD 4.4%、PF 0.998 | `currentloop()` 末段 |
| 3 | 電壓環 `Vki = 0.01` 太慢（穿越頻率 0.006 Hz），負載跳變後 Vo 偏 9 V 幾十秒回不來。Vkp=0.03、Vki=1 約 1 s 回到 100 V | §4.5、§4.6 | `VoltageLoop()`、`Vkp`/`Vki` |
| 4 | `Ipre_prev` 存在限幅之前，碰到上下限時積分會 windup | 程式閱讀 | `VoltageLoop()` |
| 5 | PFC 電感值不一致：程式註解寫 1.5658 mH（量測），先前討論用 632 µH。632 µH 時 K 上限只有約 3 | §1、§2 | `CURRENT_LOOP_K` 註解 |
| 6 | PSIM C-block 若沒加單位延遲會過度樂觀：632 µH、無延遲 THD 4%；加一拍延遲 THD 117% | §6 | PSIM 模型 |
| 7 | Buck 控制器符號：控上臂時必須是「＋」。理想元件時「−」看似收斂，是因為整條相位平移 180° 把 392 Hz 諧振壓住了，但直流變成正回授，不能加積分 | §5 | 未來的 Buck 閉迴路 |

---

## 參數

| 符號 | 值 | 來源 |
|---|---|---|
| V<sub>m</sub> | 163 V（115 V<sub>rms</sub>） | 硬體 |
| T<sub>s</sub> | 50 µs（20 kHz ISR） | `EPwm1Regs.TBPRD = 2250` |
| T<sub>d</sub> | 75 µs = 1.5 T<sub>s</sub>（一拍計算延遲＋半拍 ZOH） | 假設 |
| L（PFC） | 1.5658 mH（另一候選 632 µH） | 程式註解 |
| K | 9.3 Ω | `CURRENT_LOOP_K` |
| C<sub>bus</sub> | 10 µF | 硬體 |
| L<sub>b</sub>、C<sub>o</sub> | 75 µH、2200 µF | 硬體 |
| ESR、DCR | 20 mΩ、30 mΩ（buck）、50 mΩ（boost） | **假設，請以 datasheet 為準** |
| V<sub>o</sub><sup>*</sup> | 100 V | `VOUT_SOFT_FINAL` / `VOUT_REF_FINAL` |
| V<sub>ref</sub> | \|b1\|+40，下限 140 V | `UpdateVref()` |
| 負載 | 500 W ↔ 20 Ω 電阻 | 假設（WPT 級等效成電阻） |
| 量測低通 | z = 0.001568 → τ = 31.9 ms（5 Hz） | `lowpass()` |
| 電壓環 | Vkp = 0.1、Vki = 0.01，每 line cycle 一次 | `VoltageLoop()` |

---

## §0 方法

1. **平均模型（大訊號）**：開關週期內取平均，令導數為零得到穩態解（＝前饋）。
2. **線性化（小訊號）**：x = X + x̂，丟掉 x̂·ŷ 二階項。例：`d·Vbus ≈ DV + D·v̂ + V·d̂`。`V·d̂` 前的 V 就是受控體增益 ∝ V<sub>bus</sub> 的原因，也是兩個控制器都除以 V<sub>bus</sub> 的原因。
3. **迴路增益**：受控體 × 控制器 × 延遲，讀 PM、GM。
4. **驗證**：時域模擬（`sim/sysim.c`）或 PSIM AC Sweep。

**負回授的判斷**：沿迴路繞一圈數「−」號，奇數個＝負回授。直流符號只保證低頻；高頻還要看相位裕度。

---

## §1 PFC 電流環

整流座標的 boost（d = 儲能開關 duty）：

```
L di/dt = |vac| − (1−d)·Vbus
d_ff    = 1 − |vac|/Vref
```

控制律 `d = d_ff + (K/Vbus)(iref − i)` 代回，Vbus 約掉：

```
L di/dt = |vac|·(1 − Vbus/Vref) + K·(iref − i)
```

K 的作用像一顆串在電感上的「虛擬電阻」（單位 Ω）。迴路增益：

```
T_i(s) = K/(sL) · e^(−s·Td)
fc = K/(2πL)          PM = 90° − 360°·fc·Td
```

| L | K | f<sub>c</sub> | PM | GM |
|---|---|---|---|---|
| 1.566 mH | 9.3 | 944 Hz | 64.5° | 10.9 dB |
| 632 µH | 9.3 | 2.34 kHz | 26.8° | 3.1 dB |

單看電流環，K 的上限是 f<sub>c</sub> 碰到 1/(4T<sub>d</sub>) = 3.33 kHz，即 K<sub>max</sub> ≈ 2πL·3333 ≈ 33（1.566 mH）。

## §2 Bus 與 Buck 的耦合（K 的真正上限）

**Buck 前饋讓它變成直流變壓器**：`Busloop()` 用 d<sub>b</sub> = V<sub>o</sub><sup>*</sup>/V<sub>ref</sub>，低頻時 V<sub>bus</sub> ≈ V<sub>o</sub>/d<sub>b</sub> = V<sub>ref</sub>·V<sub>o</sub>/V<sub>o</sub><sup>*</sup>。兩個前饋用同一個 V<sub>ref</sub>，所以 tracking bus 成立。

**從 bus 看進 boost 的導納**：

```
Y_boost(jω) = (1−D)² / (jωL + K·e^(−jωTd))
Re{Y_boost} ∝ K·cos(ωTd)    →  f > 1/(4Td) = 3.33 kHz 時為負電阻，K 越大越負
```

**Bus 諧振**（Buck 電感折算到 bus 側；2200 µF 視為短路）：

```
f_bus = d_b / (2π·√(Lb·Cbus))
```

| V<sub>ref</sub> | d<sub>b</sub> | f<sub>bus</sub> |
|---|---|---|
| 140 V | 0.714 | 4.15 kHz |
| 170 V | 0.588 | 3.42 kHz |
| 203 V | 0.493 | 2.86 kHz |

諧振落在負電阻區附近。四狀態（i<sub>L</sub>, V<sub>bus</sub>, i<sub>b</sub>, v<sub>C</sub>）＋一拍延遲的離散系統，在各瞬時 |v<sub>ac</sub>| 凍結求特徵值，得到 K 上限（`python sim/analysis.py coupling`）：

| \|v<sub>ac</sub>\| | K<sub>max</sub>（1.566 mH） | K<sub>max</sub>（632 µH） |
|---|---|---|
| 20 V | 10.3 | 4.3 |
| 60 V | 9.5 | 3.5 |
| 100 V | **8.5** | 2.8 |
| 130 V | 8.8 | 2.8 |
| 163 V | 12.0 | 3.8 |

K=9.3 在 |v<sub>ac</sub>|≈100 V 那段已略超過，只是停留時間短。整體模擬：K=6 → THD 6.6%、K=7 → 5.8%、K=9.3 → 4.5%、K=10 → 38%（失穩）。

可考慮的方向（皆未驗證、未套用）：K 降到 6–7；縮短延遲讓 1/(4T<sub>d</sub>) 上移；bus 加 RC 阻尼或改變 C<sub>bus</sub>（會影響 tracking）。

## §3 Duty 斜率限制

```
|d(d_ff)/dt|max = ω·Vm/Vref = 377·163/140 = 439 /s  →  每拍 0.022
```

| slew 上限 | THD | PF |
|---|---|---|
| 0.010（現行） | 86% | 0.21 |
| 0.015 | 101% | 0.45 |
| 0.020 | 42% | 0.88 |
| 0.025 | 4.4% | 0.998 |
| 無限制 | 4.5% | 0.998 |

想保留啟動保護的話，可以只在 soft-start 期間用 0.01，穩態放寬到 ≥0.03。

## §4 PFC 電壓環

能量存在 C<sub>o</sub>（C<sub>bus</sub> 只存 0.14 J，500 W 下撐 0.29 ms），以 line cycle 平均：

```
Co·Vo·dVo/dt = p̄_in − P_load
```

電流含參考項與 §1 的 bus 偏差項；代入 V<sub>bus</sub> = V<sub>ref</sub>·V<sub>o</sub>/V<sub>o</sub><sup>*</sup> 後取平均：

```
p̄_in = Vm·Ipre/2 + (Vm²/2K)·(Vo* − Vo)/Vo*        ← 第二項是「下垂」，由電流環 K 帶出
```

線性化：

```
G_v(s) = v̂o/îpre = (Vm/2) / (Co·Vo·s + Gd)
Gd = Vm²/(2K·Vo*)  +  ∂P_load/∂Vo
   = 14.3 W/V      +  2Vo/R = 10 W/V（電阻）或 0（定功率）
極點：17.5 Hz（電阻 20 Ω）／10.3 Hz（定功率）
```

**取樣不會引入 120 Hz**：取樣固定在每週期同一相位（`integral1` ≈ 4.71），120 Hz 漣波被取樣成固定偏移，不會進入 I<sub>pre</sub>。

**z 域**（ZOH，T = 1/60 s；G(s) = k/((s+a)(s+b))，k = V<sub>m</sub>/(2C<sub>o</sub>V<sub>o</sub>τ)、a = G<sub>d</sub>/(C<sub>o</sub>V<sub>o</sub>)、b = 1/τ）：

```
G(z) = k/(ab) + (1−z⁻¹)·[ (k/(a(a−b)))/(1−e^(−aT)z⁻¹) + (k/(b(b−a)))/(1−e^(−bT)z⁻¹) ]
C(z) = Vkp + Vki·T/(1−z⁻¹)        （程式的速度型 PI）
```

| Vkp / Vki | f<sub>c</sub> | PM | GM | 說明 |
|---|---|---|---|---|
| 0.1 / 0.01（現行） | 0.006 Hz | 110° | 28 dB | 太慢，積分時間常數約 28 s |
| 0.03 / 1 | 0.54 Hz | 88° | 34 dB | 建議 |
| 0.1 / 2 | 1.12 Hz | 93° | 25 dB | 較快 |

（電阻負載；定功率負載的數字見 `analysis.py voltage`）

**模型驗證**：1 s 時 20→40 Ω、5 s 時回 20 Ω，線週期平均模型與整體模擬的 V<sub>o</sub> 差距 < 0.4 V。

## §5 Buck（未來若改閉迴路）

目前 Buck 開迴路前饋是正確的分工：能量平衡交給 PFC 電壓環，Buck 當直流變壓器。

```
G_vd(s) = Vbus · Zp/(s·Lb + rL + Zp),   Zp = R ∥ (rC + 1/(s·Co))
f0 = 1/(2π√(Lb·Co)) = 392 Hz,   Z0 = √(Lb/Co) = 0.185 Ω
1/Q ≈ Z0/R + (rL+rC)/Z0        理想 Q≈108；含 50 mΩ 寄生 Q≈3.6
ESR 零點 fz = 1/(2π·rC·Co) = 3.6 kHz
```

控上臂時 ∂V<sub>o</sub>/∂d = +V<sub>bus</sub>，Ve = Vref − Vo，所以控制律為 **d = d<sub>ff</sub> + (Kp + Ki/s)·Ve / V<sub>bus</sub>**。

| 條件 | ＋號 | −號 |
|---|---|---|
| 理想元件、只有 P（Kp≈0.3） | 392 Hz 振盪發散（GM<0） | 看似很平（諧振被翻成負回授） |
| 含寄生、PI（Kp=0.05、Ki=100） | 穩定，零穩態誤差，GM 18.5 dB | **Vo 崩到 0**（積分器在正回授裡） |
| 只有 P、Kp≥1 | — | 直流增益 1/(1−Kp) 變號，發散 |

穩定條件（P 部分）：√(Kp² + (Ki/ω<sub>0</sub>)²)·Q < 1。PSIM 請務必加 ESR/DCR。

若 Buck 要閉迴路：頻寬必須遠低於 120 Hz，讓 V<sub>o</sub> 保留 120 Hz 漣波（振幅 3 V、峰對峰 6 V），且 PFC 電壓環不能再積分 V<sub>o</sub>（兩個積分器會搶同一個能量狀態）。WPT 定電壓回授建議接到 `Vout_ref`。

## §6 整體系統模擬

`sim/sysim.c`：整流座標 boost → 10 µF bus → 同步 Buck → 2200 µF → 電阻負載；控制律逐拍照抄 `main.c`，含一拍延遲、|b1|<5 V 兩臂關閉、限幅與 slew。

| L | slew | 延遲 | THD | PF | V<sub>o</sub> 峰對峰 |
|---|---|---|---|---|---|
| 1.566 mH | 0.01 | 一拍 | 86% | 0.21 | 68 V |
| 1.566 mH | 0.01 | 無 | 88% | 0.21 | 61 V |
| 1.566 mH | 放寬 | 一拍 | **4.5%** | **0.998** | 6.0 V |
| 1.566 mH | 放寬 | 無 | 3.8% | 0.999 | 6.0 V |
| 632 µH | 放寬 | 一拍 | 117% | 0.30 | 25 V |
| 632 µH | 放寬 | 無 | 4.0% | 0.999 | 5.9 V |
| 632 µH、K=3 | 放寬 | 一拍 | 13% | 0.96 | — |

## §7 頻寬分層

| 迴路 | 負責的量 | 頻寬 | 要避開 |
|---|---|---|---|
| PFC 電流環 | 輸入電流形狀 | ≈ 950 Hz | bus 諧振 2.9–4.2 kHz、延遲 3.3 kHz |
| Boost／Buck 前饋 | V<sub>bus</sub> 追 V<sub>ref</sub>、Buck 比例 | 開迴路 | — |
| PFC 電壓環 | V<sub>o</sub> 能量平衡 | 0.5–1 Hz（建議） | 120 Hz 漣波、60 Hz 取樣 |
| WPT 定電壓（nRF） | `Vout_ref` | ≪ 0.5 Hz | 電壓環頻寬 |

## 待辦

1. 確認 PFC 電感實際值（1.566 mH 或 632 µH）。
2. PSIM：C-block 輸出加單位延遲；電容 ESR、電感 DCR 填實際值。
3. 評估穩態放寬 `DUTY_SLEW_MAX` 到 ≥0.03。
4. 電壓環試 Vkp=0.03、Vki=1，`Ipre_prev` 改存限幅後的值。
5. PSIM AC Sweep 量 v̂<sub>o</sub>/î<sub>pre</sub>，與 §4 的 G<sub>v</sub>(s) 比對。

---

## 重現

```bash
cd docs/control_analysis/sim
gcc -O2 -o sysim sysim.c -lm
SLEW=1 ./sysim 0.03 1 9.3 run 9        # 建議電壓環增益，slew 不限制
python analysis.py thd run              # → THD 4.45%  PF 0.9978
python analysis.py all                  # 各節小訊號數字
```

Windows 上可用 MSYS2/MinGW 的 gcc 或 WSL；`analysis.py` 需要 numpy、scipy。
