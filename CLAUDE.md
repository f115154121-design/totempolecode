# totempolecode

Totem-pole 無橋 PFC + Buck（後接雙邊 LCC 無線充電），TMS320F28069（CCS），先在 PSIM C-block 驗證再移植。
使用者以繁體中文溝通；解釋時從原理講到公式。

## 專案結構

- `phaselock/main.c`：目前主要的 PFC + Buck 控制程式（PLL、`currentloop()`、`VoltageLoop()`、`Busloop()`、`UpdateVref()`）
- `phaselock/test_pll_pwm.c`：PLL／PWM 台架測試（與 main.c 二擇一編譯）
- `1150421PFC+PRI_WPT+SPI_FPU/`：PFC + 一次側 WPT，nRF24L01 接收
- `1150428SEC_WPT+SPI/`：二次側 WPT，nRF24L01 發送
- `psim_*.c`：PSIM C-block 版本
- `docs/control_analysis/`：控制迴路的小訊號推導、Bode 分析與整體系統模擬
- `docs/theory/`：這個拓樸的原理筆記（為什麼是 totem-pole、能量緩衝在哪、三環為何這樣配）。要建立整體理解時從這裡開始

## 控制設計

修改任何控制參數或控制律之前，先讀 `docs/control_analysis/README.md`。重點：

- 控制律符號：誤差一律定義為 `ref − fb`，控制器的符號必須跟受控體的符號一致（迴路增益 `L = C·P` 要為正，否則變成正回授）。
  - PFC 電流環：`∂i/∂d > 0`（duty 加大＝電感充電久一點）→ `d = d_ff + Kp·(iref − i)`，**＋**。
  - Buck 控上臂調 **Vo**：`∂Vo/∂d = +Vbus` → `d = d_ff + (Kp + Ki/s)·(Vo* − Vo)/Vbus`，**＋**（見 README §5）。
  - Buck 控上臂調 **Vbus**：`∂Vbus/∂d < 0`（Buck 是 bus 的負載，duty 加大＝抽走更多能量）→ 必須 **−**。
    等價寫法是把迴路寫在 `1−d` 上（從輸出端看，這級是把 Vo 升到 Vbus 的 boost，`∂Vbus/∂(1−d) > 0`），這樣控制器回到 ＋，負號被變數換算吸收。
    PSIM bandwidth study 實測：用 ＋ 時 `Vkp = 0.0026` 發散，用 − 時同一值正常；靜態翻號條件為 `Vkp < Vo/Vbus²`。
- PFC 電感 **= 1.566 mH（已確認，2026-10-07）**。先前 632 µH 的說法作廢；README §1/§2 中 632 µH 那一欄不適用。
- `CURRENT_LOOP_K` 的上限由 10 µF bus 與 Buck 電感的諧振（2.9–4.2 kHz）加上一拍延遲決定，約 8.5–12，不是單看電流環的 33。
  最嚴格的點在 |v_ac| ≈ 100 V 處，K_max = 8.5；**現行值 7.0**（commit b30de1d 由 9.3 降下。9.3 時模擬 THD 4.5%，K=10 則 38% 失穩）。
  那張表的 ESR/DCR 是假設值，實際寄生更小的話上限還會再降，bring-up 階段建議用 6–7 留裕度。
  注意：K_max = 8.5 是用**純 P** 推的。電流環若加上 PI 零點，迴路形狀改變，那個上限要重算（見 README §8）。
- duty 斜率限制必須 ≥ 0.025 才追得上零交越附近的前饋。現行為 `DUTY_SLEW_START` / `DUTY_SLEW_RUN`，皆 0.03（舊名 `DUTY_SLEW_MAX` 已不存在）。
- **能量緩衝在 Co（2200 µF）不在 Cbus（10 µF）**，這決定了 Buck 該控什麼。120 Hz 的能量擺幅 `ΔE = P/ω`，600 W 時 1.59 J：Cbus 要吞下得擺 936 V（不可能），Co 只要 7.2 V。所以 Vo 本來就該有 ±3.6 V 的 120 Hz 漣波，那是緩衝在工作，不是缺陷。
  推論：**Buck 要控 Vbus，不是控 Vo**（＝老師三環圖的配置）。若 Buck 緊控 Vo，120 Hz 能量被擋在 bus 側而 10 µF 吃不下，迴路會跟物理打架。推導見 `docs/theory/README.md` §5。
- PFC 電壓環負責 Vo 的能量平衡；Buck 目前是開迴路前饋（直流變壓器），兩者不能同時積分 Vo。
- 調參數後可用 `docs/control_analysis/sim/sysim.c` 驗證（見該 README 的「重現」）。

## 硬體（2026-10-16 實測）

- **輸入差模濾波 = 1 µF X2（275 V~ MPX）**。原本是 5 µF，造成輕載時明顯的電流超前。換成 1 µF 後實測相位差 **2.408°**（PF 0.9991）。
  - 機制：電容電流 `I_c = ωCV` 固定不隨負載變，所以超前角 `= atan(I_c / (P/V))`，**輕載最嚴重**。5 µF 在 110 V 時 `I_c = 0.207 A` → 100 W 時 12.8°、200 W 時 6.5°、600 W 時 2.1°。
  - **不要拆光**。無電容時輸入電壓的 20 kHz 擾動達 20 V p-p（電感漣波 ≈ 1 A p-p × 電源內阻抗 ≈ 20 Ω @ 20 kHz）。這是濾波與相位的取捨，1 µF 是目前的平衡點。
- PLL 有約 **3° 的固有相位超前**（PSIM 量 `cos(theta)` 對 `Vac` 的零交越差 110 µs）。不補償：`cos(3°) = 0.9986`，代價 0.14%。
- **Vbus 上應並聯 0.1~1 µF 薄膜電容（≥ 400 V DC），必須緊貼半橋**。它的價值全部來自縮短換流迴路（`V_spike = L_loop × di/dt`，500 nH 就是 80 V），放遠了等於沒裝。不要用陶瓷，1 µF X7R 在 200 V DC bias 下容值掉七八成。**尚未安裝。**
- 示波器量相位不要讀「最大」—— 它會抓到開關漣波的尖峰，峰值位置偏好幾度。用 `dbg_Vin_real` / `dbg_Iin_real`（`DBG_LEN = 333` ≈ 一個 60 Hz 週期，每筆 50 µs = 1.08°）比較可靠。
