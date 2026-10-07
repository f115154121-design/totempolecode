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
  最嚴格的點在 |v_ac| ≈ 100 V 處，K_max = 8.5；現行的 9.3 已略超過（模擬 THD 4.5%，K=10 則 38% 失穩）。
  那張表的 ESR/DCR 是假設值，實際寄生更小的話上限還會再降，bring-up 階段建議用 6–7 留裕度。
- `DUTY_SLEW_MAX` 必須 ≥ 0.025 才追得上零交越附近的前饋。
- PFC 電壓環負責 Vo 的能量平衡；Buck 目前是開迴路前饋（直流變壓器），兩者不能同時積分 Vo。
- 調參數後可用 `docs/control_analysis/sim/sysim.c` 驗證（見該 README 的「重現」）。
