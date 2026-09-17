#include "DSP28x_Project.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "nRF24L01.h"
//----------------------------------------------------------------
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadSize;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
//----------------------------------------------------------------
void InitEPwmTimer (void);
void protect (void);
void MOSFET_off (void);
void debuns (void);
void PLL (void);
void mode_selection (void);
void IrefSET (void);
void PIduty (void);
void switch_modulation (void);
void VoltageEstimation (void);
void UpdateMaxCurrent (void);
void update_mode_request(void);
void UVLO_protect (void);
void ClockDivider (void);
void CheckPLLLock (void);
void ModeSelect_CrossDetect (void);
void Calc_CurrentLoop_Kp (void);
void Resonant_PWM_Settings(void);
//----------------------------------------------------------------SPI NRF24L01+
//#define TX_EN   1     // select device is TX and / or
#define RX_EN   1     // RX
#define PWM1_INT_ENABLE  1

nRF24L01_Vars_t nRF_RX = NRF24L01_DEFAULTS;

uint16_t rxData;
uint16_t rxData1;
int SPIValue = 0, VSPIValue = 0;
float Iout_real = 0, Vout_real = 0;
//----------------------------------------------------------------
__interrupt void adc_isr(void);
//__interrupt void epwm1_timer_isr(void);
__interrupt void cpu_timer1_isr(void);
//----------------------------------------------------------------
int state = 0, state1 = 0, startupFLAG = 0;
#define N 417   // 陣列大小417
volatile float v1[N], v2[N], v3[N], v4[N], v5[N], v6[N], v7[N], v8[N];
int v_index = 0;

#define kp 5
#define ki 0.8
//#define kec 1.0214
float a = 0, bata = 0, a1 = 0, bata1 = 0, y1, sfq, sfd, s5q, s5d, cfq, cfd, c5q, c5d, theta = 0, c5a, c5b, s5a, s5b,
      d = 0, q = 0, d1 = 0, q1 = 0, fc = 5, dt = 0.00004, wc = 0, prevoutput = 0, prevoutput1 = 0, z = 0.00062792, output = 0, output1 = 0, error = 0, integral = 0, integral1 = 0,
      w = 0, d_filt = 0, q_filt = 0, v_current;
float iac_real = 0, vc1_real = 0, vc2_real = 0, vbus_real = 0, vbus_ref = 0, vac_real = 0,iac_protect_value = 0, vbus_protect_value = 0;
int adc_voltage = 0, iac_adc = 0, vc1_adc = 0, vc2_adc = 0, vbus_adc = 0;

Uint16 sw1_input = 0, sw2_input = 0;
int SW1 = 0, shutdown1 = 1, SW1_count = 0, SW1_ONcount = 0, SW2 = 0, shutdown2 = 1, SW2_count = 0, SW2_SETcount = 0;
int start_ready = 0, PLL_flag = 0, PLLcount = 0, PLLERRcount = 0, modulation = 0, modulationLOCK = 0;

float Vctrl = 0;//, dutyset = 0, dutyinvert = 0, VfiveCtrl = 0, DoubleDutyCycle = 0,
float feedforward = 0, Ierr = 0, Iref = 0, Ifb = 0, Ierr_1 = 0, Iacref = 0;
float Ikp = 0, Iki = 0, Ipi_1 = 0, Ipi = 0, Ipi_temp = 0, Vctrl_temp = 0, IkiTs = 0;
float Vctrl_Upper_limit = 0.98, ImaxSET = 0.1;
int protectFLAG = 0, Vbus_protectFLAG = 0, iac_protectFLAG = 0, modulWait = 0, modulWaitFLAG = 0, PLL_protectFLAG = 0, PWMflag = 0, UVLOflag = 0, UVLOcountERR = 0;

float alpha = 0.00062792, Vbus_filiter = 0, Vbus_filiter_1 = 0, Verr = 0, Verr_1 = 0, Vpi = 0, Vpi_1 = 0, Imax = 0.1, phase = 0, Imax_Upper_limit = 0;
float Vkp = 0, Vki = 0;  //p0.28  i  0.11
int VfbLock = 0, DivAdc = 0, zero_crossing = 0;
float cos_value = 0, sin_value = 0;
int half_wave_charging = 0, HWcount = 0, Pmax = 1600;
float V_square_sum = 0, Vac_RMS = 0;
int VsampleCount = 0, vacsign = 0, selected_mode = 0;
float theta_prev = 0;
int zero_cross_pos = 0, zero_cross_neg = 0, zero_cross_event = 0, zc_now = 0, zc_prev = 0;
float vac_abs = 0, Vac_peak_hold = 0, Vac_peak = 0;
float Vac_filiter = 0, Vac_filiter_1 = 0;
float DFF_prev = 1;
int VctrlHalfState = 0, DFF_direction = 0, DFF_direction_prev = 0, VcStateLOCK = 0, ResPWMSetLOCK = 0;
float invVac = 0, inv_vbusref = 0;
float VctrlResonant = 0;
int led_tick_count = 0;
//----------------------------------------------------------------
Uint16 AutomaticStartFlag = 0, OFFcount = 0, ONcount = 0;
Uint16 halfTBPRD_50Kset = 0, TBPRD_50Kset = 900, dutyset = 0, dutyinvert = 0;
Uint16 TBPRD_100Kset = 450, halfTBPRD_100Kset = 0, ResonantDutyset = 0, ResonantDutyinvert = 0;
Uint16 DoubleDutyCycle = 0, VfiveCtrl = 0, DoubleDutyMinutONE = 0, ONEMinusDoubleDuty = 0;
//----------------------------------------------------------------PFC運算電流迴圈Kp參數
#define L  				1 * 10e-3		//1mH
#define Tsw  			20 * 10e-6		//1/50kHz	//20us
#define L_over_Tsw  	50		//L_over_Tsw=L/Tsw

float Ikp1 = 0, Kp_Gain = 0.5;
//----------------------------------------------------------------PFC模式設定
int modulationSET = 102;    ////強制模式設定 101:低壓三階 102:全喬單極性 103:高壓五階 104:高壓五階變形(低三+單極性) 105:高壓五階變形(低三+單極性+高壓四階) 0:自動模式選擇////

//----------------------------------------------------------------SPI設定
//SPI設定
//----------------------------------------------------------------
void SPI_Config4Wire(volatile struct SPI_REGS *v)
{
    v->SPICCR.bit.SPISWRESET    = 0;    // force reset
    v->SPICCR.bit.CLKPOLARITY   = 0;    // 0 = rising edge // 1 = falling edge
    v->SPICCR.bit.SPILBK        = 0;    // loopback mode disable
    v->SPICCR.bit.SPICHAR       = 7;    // 8bit char length
    v->SPICTL.bit.OVERRUNINTENA = 0;    // overrun INT disable
    v->SPICTL.bit.CLK_PHASE     = 1;    // 0 = without delay // 1 = with delay
    v->SPICTL.bit.MASTER_SLAVE  = 1;    // SPI master mode
    v->SPICTL.bit.TALK          = 1;    // transmit is enabled
    v->SPICTL.bit.SPIINTENA     = 0;    // INT disable
    // for SPIBRR >= (3-127):
    /* SPI BAUD RATE = LSPCLK(default is 22,5MHz) / (SPIBRR +1) */
    // for SPIBRR < 3:
    /* SPI BAUD RATE = LSPCLK(default is 22,5MHz) / 4 */
    v->SPIBRR                   = 3;    // sets SPI Clk up to 3,75MHz
    v->SPIPRI.bit.TRIWIRE       = 0;    // 4-wire mode
    v->SPIPRI.bit.FREE          = 1;    // breakpoints don't disturb xmission
    v->SPICCR.bit.SPISWRESET    = 1;    // take out of reset

    // TX FIFO
    v->SPIFFTX.bit.SPIRST       = 1;    // use FIFO
    v->SPIFFTX.bit.SPIFFENA     = 1;    // use FIFO
    v->SPIFFTX.bit.TXFIFO       = 0;    // reset FIFO ptr.
    v->SPIFFTX.bit.TXFIFO       = 1;    // re-enable FIFO
    v->SPIFFTX.bit.TXFFIENA     = 0;    // FIFO INT disable
    v->SPIFFTX.bit.TXFFINTCLR   = 1;    // clear INT flag

    // RX FIFO
    v->SPIFFRX.bit.RXFFOVFCLR   = 1;    // clear overflow flag
    v->SPIFFRX.bit.RXFIFORESET  = 0;    // reset FIFO ptr.
    v->SPIFFRX.bit.RXFIFORESET  = 1;    // re-enable FIFO
    v->SPIFFRX.bit.RXFFOVFCLR   = 1;    // clear INT flag
    v->SPIFFRX.bit.RXFFIENA     = 0;    // INT disable
    v->SPIFFRX.bit.RXFFIL       = 2;    // INT Flag after 2 words in FIFO
    //
    v->SPIFFCT.all              = 0x00; // no use of this register
}

//----------------------------------------------------------------main主程式
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
//    PieVectTable.EPWM1_INT = &epwm1_timer_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;

    EDIS;

    IER |= M_INT1;
    IER |= M_INT13;
    InitEPwmTimer();

//    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
//    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

    EINT;
    ERTM;
//----------------------------------------------------------------GPIO設定
    EALLOW;

    GpioCtrlRegs.GPAMUX2.bit.GPIO19=0;      //LED1
    GpioCtrlRegs.GPADIR.bit.GPIO19=1;
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO20=0;      //LED2
    GpioCtrlRegs.GPADIR.bit.GPIO20=1;
    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;    //LED3
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO23=0;      //LED4
    GpioCtrlRegs.GPADIR.bit.GPIO23=1;
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO15=0;      //LED5
    GpioCtrlRegs.GPADIR.bit.GPIO15=1;
    GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO26=0;      //LED6
    GpioCtrlRegs.GPADIR.bit.GPIO26=1;
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO27=0;      //LED7
    GpioCtrlRegs.GPADIR.bit.GPIO27=1;
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO28=0;      //SW1
    GpioCtrlRegs.GPADIR.bit.GPIO28=0;
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO29=0;      //LED8
    GpioCtrlRegs.GPADIR.bit.GPIO29=1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO30=0;      //SW2
    GpioCtrlRegs.GPADIR.bit.GPIO30=0;
    GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO31=0;      //LED9
    GpioCtrlRegs.GPADIR.bit.GPIO31=1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

    GpioCtrlRegs.GPBMUX1.bit.GPIO33=0;      //LED10
    GpioCtrlRegs.GPBDIR.bit.GPIO33=1;
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;

    EDIS;
//----------------------------------------------------------------記憶
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();
//----------------------------------------------------------------TIMERS
// TIMERS
//----------------------------------------------------------------
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 90MHz CPU Freq, 1 second Period (in uSeconds)
    //ConfigCpuTimer(&CpuTimer0, 90, 4500);//4.5ms
    ConfigCpuTimer(&CpuTimer1, 90, 1300);//4.5ms	//1.3ms

    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the below

    // Use write-only instruction to set TSS bit = 0
    //CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;

//----------------------------------------------------------------SPI
// SPI
//----------------------------------------------------------------
	InitSpibGpio();
	SPI_Config4Wire(&SpibRegs);

    #ifdef RX_EN
//----------------------------------------------------------------
// nRF24 RX settings
//----------------------------------------------------------------
		nRF_RX.cePin  = 16; //16
        nRF_RX.csnPin = 18; //18
        nRF_RX.irqPin = 21;
        nRF_RX.SPI_Regs = &SpibRegs;

        // set GPIOs here cause there is no function so far in init
        GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // CE low   //16
        GpioDataRegs.GPASET.bit.GPIO18   = 1;   // CSN high //18
        GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;    // LED low  //03

        EALLOW;
        GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;     // CE pin output  //16
        GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;     // CSN pin output //18
        GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // LED pin output //03
        EDIS;

        // Step 1: get all initial values from nRF24 and make basic setting
        nRF24_Init(&nRF_RX);

        // Step 2: set address width, RX & TX address afterwards
        nRF_RX.SETUP_AW.bit.AW = nRF_5Bytes;
        nRF24_SPI_Write(&nRF_RX, nRF24_SETUP_AW);

        nRF_RX.TX_ADDR[0] = 0xDE;
        nRF_RX.TX_ADDR[1] = 0xAD;
        nRF_RX.TX_ADDR[2] = 0xBE;
        nRF_RX.TX_ADDR[3] = 0xEF;
        nRF_RX.TX_ADDR[4] = 0xF0;
        nRF24_SetTxAddress(&nRF_RX);

        // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
        // needs to listen to the TX addr on pipe#0 to receive them.
        nRF_RX.RX_ADDR_P0[0] = 0xDE;
        nRF_RX.RX_ADDR_P0[1] = 0xAD;
        nRF_RX.RX_ADDR_P0[2] = 0xBE;
        nRF_RX.RX_ADDR_P0[3] = 0xEF;
        nRF_RX.RX_ADDR_P0[4] = 0xF0;
        nRF24_SetRxAddress(&nRF_RX, nRF24_PIPE_0);

        // Step 3: open used pipes & set payload width
        nRF_RX.EN_RXADDR.bit.ERX_P0 = 1;    // enable PIPE 0
        nRF24_SPI_Write(&nRF_RX, nRF24_EN_RXADDR);

        nRF_RX.RX_PW_P0.all = (uint16_t)nRF24_PAYLOAD_WIDTH;
        nRF24_SPI_Write(&nRF_RX, nRF24_RX_PW_P0);

        // Step 4: speed, power & channel
        nRF24_SetDataRate(&nRF_RX, nRF_2Mbps);
        nRF_RX.RF_SETUP.bit.RF_PWR =nRF_minus0dBm;
        nRF24_SPI_Write(&nRF_RX, nRF24_RF_SETUP);
        nRF_RX.RF_CH.bit.RF_CH = 120;
        nRF24_SPI_Write(&nRF_RX, nRF24_RF_CH);

        // Step 5: power up and set in RX control mode
        nRF24_ActivateRx(&nRF_RX);

        // only for debug
        nRF24_GetRegValues(&nRF_RX);
    #endif
//----------------------------------------------------------------ADC設定
    EALLOW;

    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  //Enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    //ADCINT1 trips after AdcResults latch
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    //Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    //Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 2;    //setup EOC2 to trigger ADCINT1 to fire

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;    // set SOC0 channel select to ADCINA0
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;    // set SOC1 channel select to ADCINA1
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 2;    // set SOC2 channel select to ADCINA2
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 3;    // set SOC2 channel select to ADCINA3
    AdcRegs.ADCSOC8CTL.bit.CHSEL    = 8;    // set SOC2 channel select to ADCINA8
    AdcRegs.ADCSOC9CTL.bit.CHSEL    = 9;    // set SOC2 select to ADCINA9
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;    // set SOC2 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;    // set SOC3 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 5;    // set SOC6 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 5;    // set SOC6 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 6;    // set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 6;    // set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC8CTL.bit.ACQPS    = 6;    // set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC9CTL.bit.ACQPS    = 6;    // set SOC2 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    EDIS;

    InitAdc();
//----------------------------------------------------------------
    c5d = c5a = cos(0.08722);
    c5q = c5b = cos(0.08722);
    s5d = s5a = sin(0.08722);
    s5q = s5b = sin(0.08722);
//----------------------------------------------------------------
    while(1)
    {
//----------------------------------------------------------------保護protect
        protect();
//----------------------------------------------------------------Switch
        debuns();
//----------------------------------------------------------------鎖相檢查旗標check
        CheckPLLLock();
//----------------------------------------------------------------欠壓保護UVLO
        UVLO_protect();
//----------------------------------------------------------------start ready flag
        if(SW1 && !start_ready && fabsf(bata) < 2 && PLL_flag && !protectFLAG && !UVLOflag)
        {
            start_ready = 1;
        }
        else if(!SW1 || !PLL_flag || UVLOflag)
        {
            MOSFET_off();

            start_ready = 0;

            GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;   //LED4
            GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;   //LED5
            GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;   //LED6
        }
        startupFLAG = 1;	//開DSP啟動完成初始後，旗標建立
    }
}

void CheckPLLLock(void)
{
//----------------------------------------------------------------UVLO欠壓保護旗標發生時，直接強制解鎖
    if(UVLOflag)
    {
        PLLcount = 0;
        PLLERRcount = 0;
        PLL_flag = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        return;
    }
//----------------------------------------------------------------PLL未鎖到判斷(錯誤)
    if(fabsf(d1) > 1 || integral1 < -0.2 || integral1 > 6.5)
    {
        PLLcount = 0;

        if(PLLERRcount < 11)
            PLLERRcount++;
        else
        {
            PLL_flag = 0;	//PLL未鎖到flag
            SW1 = 0;
        }
        GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
        return;
    }
//----------------------------------------------------------------PLL確認鎖相(正確)
    if(fabsf(d1) < 0.5 && q1 >= 5 && q1 < 400 && integral1 >= 0 && integral1 < 6.3)
    {
        PLLERRcount = 0;

        if(PLLcount < 1000)
        {
            PLLcount++;
        }
        else
        {
            PLL_flag = 1;	//PLL確認鎖到flag
            GpioDataRegs.GPASET.bit.GPIO19 = 1;
        }
        return;
    }
    PLLcount = 0;	//中間不穩定區 (避免卡死)
}

void InitEPwmTimer()
{
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Gpio();

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;        //每次觸發

//----------------------------------------------------------------AC to DC PWM set
    EPwm1Regs.TBPRD = TBPRD_50Kset; // Period = 1200 TBCLK counts  //1800 //2250  //25000kHz  //TBPRD_50Kset
    EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode

    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = TB_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = TB_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.DBCTL.bit.OUT_MODE = DBA_ENABLE; // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HI; // Active Hi complementary
//    EPwm1Regs.DBFED = 50; // FED = 50 TBCLKs
//    EPwm1Regs.DBRED = 50;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

    InitEPwm2Gpio();
    EPwm2Regs.TBPRD = TBPRD_50Kset; // Period = 1200 TBCLK counts
    EPwm2Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBFED = 50; // FED = 50 TBCLKs=555ns  //42  //45
    EPwm2Regs.DBRED = 50;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM2A
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    InitEPwm3Gpio();
    EPwm3Regs.TBPRD = TBPRD_50Kset; // Period = 1200 TBCLK counts
    EPwm3Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBFED = 50; // FED = 50 TBCLKs
    EPwm3Regs.DBRED = 50;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

//----------------------------------------------------------------DC to AC PWM set
    InitEPwm4Gpio();
    EPwm4Regs.TBPRD = TBPRD_100Kset; // Period = 1200 TBCLK counts
    EPwm4Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBFED = 36; // FED = 50 TBCLKs	//待調整
    EPwm4Regs.DBRED = 36;						//待調整
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    InitEPwm5Gpio();
    EPwm5Regs.TBPRD = TBPRD_100Kset; // Period = 1200 TBCLK counts
    EPwm5Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm5Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBFED = 36; // FED = 50 TBCLKs	//待調整
    EPwm5Regs.DBRED = 36;						//待調整
    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    InitEPwm6Gpio();
    EPwm6Regs.TBPRD = TBPRD_100Kset; // Period = 1200 TBCLK counts
    EPwm6Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;   // Clock ratio to SYSCLKOUT
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_RED_DBB_FED;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBFED = 36; // FED = 50 TBCLKs	//待調整
    EPwm6Regs.DBRED = 36;						//待調整
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM3A
    EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}

__interrupt void adc_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO20 = 1;     //LED2

	Resonant_PWM_Settings();	//諧振PWM設置&緩啟動		//TEST
//----------------------------------------------------------------取樣
    adc_voltage = AdcResult.ADCRESULT0 - 1972; // 轉換為 0-3.3V
    iac_adc = AdcResult.ADCRESULT2 - 2056;
    vbus_adc = AdcResult.ADCRESULT3 - 2050;
    vc2_adc = AdcResult.ADCRESULT8 - 419;
    vc1_adc = AdcResult.ADCRESULT9 - 411;

    iac_real = iac_adc * 0.015953f;          //iac_adc/204.8/0.3293
   // vbus_real = vbus_adc * 0.297539;      //vbus_adc/204.8/0.016411   //改使用兩電容取樣數值相加
    vc2_real = vc2_adc * 0.11224837f;        //vc2_adc/204.8/0.016411
    vc1_real = vc1_adc * 0.1128373f;         //vc1_adc/204.8/0.016411
    vbus_real = vc1_real + vc2_real;
    bata = adc_voltage * 0.223839f;          // 市電大小值
//----------------------------------------------------------------保護
    if(startupFLAG)		//開DSP啟動完成初始後，旗標建立(僅一次)
    {
    	protect();
    }
//----------------------------------------------------------------除頻50kHz->25kHz
    ClockDivider();
//----------------------------------------------------------------判斷要進入哪個模式50kHz
    update_mode_request();
//----------------------------------------------------------------主控制50kHz
    if(SW1 && PLL_flag && start_ready)   //SW1 == 1 && PLL_flag == 1 && start_ready == 1
    {
        if(zero_cross_pos)	//(bata1 > 0 && bata1 < 0.5f && integral1 > 4.69f)  //zero_crossing零交越點  //正半波過零
        {
            zero_crossing = 1;
            mode_selection();
        }
        else
        {
            zero_crossing = 0;
        }

        if(modulation)     //modulation != 0
        {
            IrefSET();
            PIduty();
            switch_modulation();
        }
//----------------------------------------------------------------
        if(!modulWaitFLAG)
        {
            if(zero_cross_event)		//(modulWait < 1200 && fabsf(bata1) < 0.5f)
            {
                modulWait++;
            }
            else if(modulWait >= 1200)
            {
                modulWaitFLAG = 1;
            }
        }
    }
//----------------------------------------------------------------end

    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;       //LED2

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;     //Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}

__interrupt void cpu_timer1_isr(void)
{
	GpioDataRegs.GPASET.bit.GPIO22 = 1;		//LED3
////    CpuTimer1.InterruptCount++;
////#ifdef RX_EN
//    // read out status; polling...
//    nRF24_SPI_Read(&nRF_RX, nRF24_STATUS);
//
//    if(nRF_RX.STATUS.bit.RX_DR)
//    {
//        // show RX via LED
//        nRF24_SPI_ReadRxPayload(&nRF_RX);
//        nRF_RX.STATUS.bit.RX_DR = 1;            // clear flag
//        nRF24_SPI_Write(&nRF_RX, nRF24_STATUS);
//
//        nRF24_SPI_Read(&nRF_RX, nRF24_FIFO_STATUS);
//        if(!nRF_RX.FIFO_STATUS.bit.RX_EMPTY)
//            nRF24_SPI_ReadRxPayload(&nRF_RX);
//    }
////#endif
//
//    SPIValue = nRF_RX.rxBuffer[0] * 256 + nRF_RX.rxBuffer[1];	//TEST
//    VSPIValue = nRF_RX.rxBuffer[2] * 256 + nRF_RX.rxBuffer[3];	//TEST
//    Iout_real = SPIValue * 0.01;
//    Vout_real = VSPIValue * 0.01;
//    // The CPU acknowledges the interrupt
//    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;		//LED3
}

void Resonant_PWM_Settings (void)
{
	if(!ResPWMSetLOCK)
	{
		EPwm4Regs.AQCTLA.bit.CAU = AQ_SET; // clear actions for EPWM3A
		EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
		EPwm4Regs.AQCTLB.bit.CBU = AQ_SET; // clear actions for EPWM3A
		EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;

		EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
		EPwm5Regs.AQCTLA.bit.CAD = AQ_CLEAR;
		EPwm5Regs.AQCTLB.bit.CBU = AQ_SET; // clear actions for EPWM3A
		EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;

		EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // clear actions for EPWM3A
		EPwm6Regs.AQCTLA.bit.CAD = AQ_CLEAR;
		EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR; // clear actions for EPWM3A
		EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;

		ResPWMSetLOCK = 1;
	}
    ///////TEST///////
    halfTBPRD_100Kset = TBPRD_100Kset >> 1;		//TBPRD_100Kset/2
    if(vbus_real >= 100 && ResPWMSetLOCK)
    {
    	if(ONcount > 50)
    	{
    		AutomaticStartFlag = 1;
    		GpioDataRegs.GPASET.bit.GPIO29 = 1;   //LED8
    	}
    	else
    	{
    		ONcount++;
    	}
    	 OFFcount = 0;
    }
    else if(vbus_real < 10)
    {
//    	GpioDataRegs.GPASET.bit.GPIO27 = 1;		//LED7TEST
    	if(OFFcount > 20000)
    	{
    		AutomaticStartFlag = 0;
    		GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;   //LED8
    	}
    	else
    	{
    		OFFcount++;
    	}
    	ONcount = 0;
    }
//    if(vbus_real > 15)
//    {
//    	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	//LED7TEST
//    }

    if(AutomaticStartFlag)
    {
//----------------------------------------------------------------方波緩啟動
		if(VctrlResonant < halfTBPRD_100Kset)		//開到50%
		{
			VctrlResonant = VctrlResonant + 0.0006;
			GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	//LED7TEST
		}
		else
		{
			VctrlResonant = halfTBPRD_100Kset;
			if(++led_tick_count >= 3000)		//幾次中斷才運算移動一次
			{
				GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;	//LED7TEST
			}
		}
    }
    else
    {
    	VctrlResonant = 0;
    	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	//LED7TEST
    }
//----------------------------------------------------------------數值運算
//    ResonantDutyset = 100;		//TEST固定Duty	//強制設定
    ResonantDutyset = VctrlResonant;		//D
    ResonantDutyinvert = TBPRD_100Kset - ResonantDutyset;	//1-D

//----------------------------------------------------------------方波輸出
	EPwm4Regs.CMPA.half.CMPA = ResonantDutyinvert;
	EPwm4Regs.CMPB = ResonantDutyset;
	EPwm5Regs.CMPA.half.CMPA = 0;
	EPwm5Regs.CMPB = ResonantDutyset;
	EPwm6Regs.CMPA.half.CMPA = 0;
	EPwm6Regs.CMPB = ResonantDutyinvert;

}

void ClockDivider(void)
{
    DivAdc ^= 1;   //XOR反轉  //0↔1 切換（1 個指令）  //50kHz 除頻到 25kHz
//----------------------------------------------------------------除頻50kHz->25kHz
    if(DivAdc)     //Div2  //50kHz->25kHz  //DivAdc==1
    {
        PLL();
//----------------------------------------------------------------zero_cross過零點計算/記錄事件
        zero_cross_pos = (theta_prev < 1.57f && integral1 >= 1.57f);  // 正半波過零
        zero_cross_neg = (theta_prev < 4.71f && integral1 >= 4.71f);  // 負半波過零

        zc_now = (zero_cross_pos || zero_cross_neg);

        /* 邊緣偵測：只 1 次 */
        zero_cross_event = (zc_now && !zc_prev);
        zc_prev = zc_now;

        theta_prev = integral1;     //儲存上一個theta數值
    }
//----------------------------------------------------------------除頻50kHz->25kHz
    else	//if(!DivAdc)	//Div2  //50kHz->25kHz  //DivAdc==0
    {
        VoltageEstimation();

        UpdateMaxCurrent();
//----------------------------------------------------------------存數值count
        if(v_index < N)
        {
            v_index++;  //25k
        }
        if(v_index >= N && zero_cross_pos)	//&&(integral1 > 4.69 && bata1 < 0.1)
        {
            v_index = 0;
        }
    }
//----------------------------------------------------------------除頻end
}

void VoltageEstimation (void)
{
//----------------------------------------------------------------Vac_rms calculate市電電壓均方根值計算
  /*
    V_square_sum = V_square_sum + (bata * bata);
    VsampleCount++;
    if(zero_cross_pos == 1 || zero_cross_neg == 1)
    {
        Vac_RMS = sqrtf(V_square_sum / VsampleCount);
        V_square_sum = 0;
        VsampleCount = 0;
    }
  */
//----------------------------------------------------------------Vac_peak calculate市電電壓最大值計算
    vac_abs = fabsf(bata);       // vac_abs = (bata >= 0.0f) ? bata : -bata;
    if(vac_abs > Vac_peak_hold)
    {
        Vac_peak_hold = vac_abs;
    }

    if(zero_cross_event)        //zero_cross_event==1
    {
        ////alpha = 0.20747; //fc=5 //wc=2*pi*fc //alpha = (wc * 8.333E-3) / (1 + wc * 8.333E-3)////
        Vac_filiter = Vac_filiter_1 + 0.21 * (Vac_peak_hold - Vac_filiter_1);
        Vac_filiter_1 =  Vac_filiter;

        Vac_peak = Vac_filiter;     //最大值更新
        Vac_peak_hold = 0;
    }
}
void UpdateMaxCurrent (void)
{
//----------------------------------------------------------------ImaxSET電流最大值設定
    if (Vac_peak < 50)
    {
        ImaxSET = 26;
    }
    else
    {												//(iac_rms*1.414)=pmax/(vac_peak*1.414)
    	invVac = __einvf32(Vac_peak);				//硬體快速倒數 (TI intrinsic)		//1/Vac_peak
        ImaxSET = (Pmax + Pmax) * invVac;      		//iac_peak=2*pmax/vac_peak    	//Pmax=1600		//改成這樣便超快，賺2.8us
        if(ImaxSET > 26)
        {
            ImaxSET = 26;
        }
    }
}

void update_mode_request (void)
{
    if(!zero_cross_pos)
    	return;   //先過濾非過零點

    switch(modulationSET)
    {
        case 101:
        	selected_mode = 1;
        	return;
        case 102:
        	selected_mode = 2;
        	return;
        case 103:
        	selected_mode = 3;
        	return;
        case 104:
        	selected_mode = 4;
        	return;
        case 105:
        	selected_mode = 5;
        	return;
        case 0:                        //modulationSET==0
            if(q1 <= 187)
            	selected_mode = 1;
            else if(q1 > 189 && q1 <= 254)
            	selected_mode = 2;  // 188~254
            else if(q1 > 256)
            	selected_mode = 3;   // 255+
            break;
        default:
        	break;
    }
}
void UVLO_protect (void)    //欠壓保護UVLO
{
    if(fabsf(bata) < 70)     //50Vrms
    {
        if(UVLOcountERR < 800)
        {
            UVLOcountERR++;
//            GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3
        }
        else
        {
            UVLOflag = 1;
            SW1 = 0;
//            GpioDataRegs.GPASET.bit.GPIO22 = 1;     //LED3
        }
    }
    else
    {
        UVLOflag = 0;
        UVLOcountERR = 0;
    }
}

void MOSFET_off (void)
{
    EPwm1Regs.CMPA.half.CMPA = 0;
    EPwm1Regs.CMPB = 0;

    EPwm2Regs.CMPB = 0;
    EPwm3Regs.CMPB = 0;
    Vctrl = 0;
//----------------------------------------------------------------Half-wave charging(低壓三階限定，模式一)
    if(((vbus_real < 5 || half_wave_charging) && ((q1 <= 187 && modulationSET == 0) || modulationSET == 101)) && !protectFLAG)
    {
        if(HWcount > 10000)
        {
            EPwm2Regs.CMPA.half.CMPA = 0;
            EPwm3Regs.CMPA.half.CMPA = 0;
            half_wave_charging = 1;
        }
        else
        {
            HWcount++;
            EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
            EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
            half_wave_charging = 0;
        }
    }
    else
    {
        HWcount = 0;
        EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
        EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
        half_wave_charging = 0;
    }
//----------------------------------------------------------------
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // clear actions for EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET; // clear actions for EPWM2A
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    EPwm3Regs.AQCTLA.bit.CAU = AQ_SET; // clear actions for EPWM3A
    EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET; // clear actions for EPWM3A
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    modulation = 0;
    modulationLOCK = 0;
    modulWait = 0;
    modulWaitFLAG = 0;
    Ierr_1 = 0;
    Ierr = 0;
    vbus_ref = 0;
    feedforward = 0;
    Ipi_1 = 0;
    Ipi = 0;
    Ikp =0;
    Iki = 0;
    IkiTs = 0;
    Ipi_temp = 0;
    start_ready = 0;
    SW1 = 0;
    PWMflag = 0;
    Vctrl_temp = 0;

    Vbus_filiter = 0;
    Vbus_filiter_1 = 0;
    Vpi = 0;
    Vpi_1 = 0;
    Imax = 0.1;
    Imax_Upper_limit = 0.1;
    VfbLock = 0;
    Verr = 0;
    Verr_1 = 0;
    zero_crossing = 0;
    dutyset = 0;
    dutyinvert= 0;
    VctrlHalfState = 0;
    DoubleDutyCycle = 0;
    DoubleDutyMinutONE = 0;
    ONEMinusDoubleDuty = 0;
    DFF_direction = 0;
    DFF_direction_prev = 0;
    DFF_prev = 1;
}

void protect (void)
{
    if(protectFLAG)
        goto ProtectLED;	// 一旦保護觸發，後面全部不跑，直接進保護階段

    if(vbus_real > 640)
    {
        MOSFET_off();
        vbus_protect_value = vbus_real;     //紀錄

        protectFLAG = 1;
        Vbus_protectFLAG = 1;
    }
    else if(fabsf(iac_real) > 26.0f && start_ready && modulWaitFLAG) //&& SW1_ONcount > 100 //((iac_real > 26 || iac_real < -26) && start_ready && modulWait >= 1000)
    {
        MOSFET_off();
        iac_protect_value = iac_real;       //紀錄

        protectFLAG = 1;
        iac_protectFLAG = 1;
    }
//    else if(PLL_flag == 0)
//    {
//        MOSFET_off();
//
//        protectFLAG = 1;
//        PLL_protectFLAG = 1;
//
//    }

    ProtectLED:
		if(protectFLAG)
		{
			MOSFET_off();
			GpioDataRegs.GPBSET.bit.GPIO33 = 1;     //LED10
		}
}

void debuns(void)
{
//----------------------------------------------------------------SW1
	sw1_input = GpioDataRegs.GPADAT.bit.GPIO28;
	if(sw1_input)
	{
	    SW1_ONcount = (SW1_ONcount < 1000) ? SW1_ONcount + 1 : 1000;
	    SW1_count = 0;

	    if(SW1_ONcount == 1000 && !shutdown1)
	    {
	        SW1 = 1;
	        shutdown1 = 1;
	    }
	}
	else
	{
	    SW1_count = (SW1_count < 10000) ? SW1_count + 1 : 10000;
	    SW1_ONcount = 0;

	    if(SW1_count == 10000)
	    {
	        shutdown1 = 0;

	        if(bata1 > -0.4f && bata1 < -0.1f)	//零交越點才能關閉訊號
	            SW1 = 0;
	    }
	}
//----------------------------------------------------------------SW2
    sw2_input = GpioDataRegs.GPADAT.bit.GPIO30;
    if(sw2_input)
    {
        SW2_SETcount = (SW2_SETcount < 4000) ? SW2_SETcount + 1 : 4000;
        SW2_count = 0;

        if(SW2_SETcount == 4000)
        {
            SW2 = 1;
            shutdown2 = 1;
        }
    }
    else
    {
        SW2_count = (SW2_count < 10000) ? SW2_count + 1 : 10000;
        SW2_SETcount = 0;

        if(SW2_count == 10000)
        {
            SW2 = 0;
            shutdown2 = 0;
        }
    }
}

void PLL(void)
{
    cos_value = cos(integral1);
    sin_value = sin(integral1);
//----------------------------------------------------------------dq 變換
    switch (state)
    {
        case 0:
            if(integral1 > 0)
            {
                state = 1;
            }
            d = (c5a * a + s5b * bata);
            q = ((-s5a * a) + c5b * bata);
            a = (c5d * d1 + (-s5q * q1)); // 反 dq 變換生成 a
            bata1 = (s5d * d1 + c5q * q1);
            break;
        case 1:
            d = (cos_value * a + sin_value * bata);
            q = ((-sin_value * a) + cos_value * bata);
            a = (cos_value * d1 + (-sin_value * q1)); // 反 dq 變換生成 a
            bata1 = (sin_value * d1 + cos_value * q1);
            state = 1;
            break;
    }
//----------------------------------------------------------------低通濾波器
////wc = 6.283185 * fc;  //2 * 3.14159 * fc  //z = wc * dt / (wc * dt + 1)////
////z = alpha;////
    d1 = z * d + (1 - z) * prevoutput;
    prevoutput = d1;

    q1 = z * q + (1 - z) * prevoutput1;
    prevoutput1 = q1;
//----------------------------------------------------------------PI控制
    //PI1// PI 控制器（鎖相，目標 d = 0）
    error = -d1; // 使用 -d 使 d 趨向 0
    integral = integral + error * ki * dt; // 積分項
    w = (error * kp) + integral + 376.9911184; // 角頻率

    if(w > 408.41)       //w > 2*PI*65	//65Hz
    {
        w = 408.41;
    }
    if(w < 282.74)       //w < 2*PI*45	//45Hz
    {
        w = 282.74;
    }
//----------------------------------------------------------------取徑度0 ~ 2pi
////    integral1 = fmod(integral1 + w * dt, 6.28); // 更新相位//end/////改成下面版本
    integral1 = integral1 + w * dt;     //原本使用fmod取餘數，改成判斷式+減法
    if(integral1 >= 6.283185)
    {
        integral1 = integral1 - 6.283185;
    }
}

void mode_selection(void)
{
//----------------------------------------------------------------若已鎖定或模式未改變，直接返回
    if(modulationLOCK)       //modulationLOCK != 0    //selected_mode == 0 || modulation == selected_mode || modulationLOCK != 0
    {
        return;     //離開副程式
    }
//----------------------------------------------------------------初始化旗標
    half_wave_charging = 0;
    HWcount = 0;
    modulWait = 0;
    modulWaitFLAG = 0;
    PWMflag = 0;
//----------------------------------------------------------------依照模式設定參數
    switch (selected_mode)
    {
        case 1:     //低壓三階
//            Ikp = 0.06; Iki = 0.0013;
            Vkp = 0.15; Vki = 0.14;
            vbus_ref = 405;		//605
            Vctrl_Upper_limit = 0.98;

            EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
            EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
            EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            EPwm2Regs.CMPA.half.CMPA = 0;
            EPwm3Regs.CMPA.half.CMPA = 0;
            EPwm2Regs.CMPB = 0;
            EPwm3Regs.CMPB = 0;

            GpioDataRegs.GPASET.bit.GPIO23 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
            break;

        case 2:     //全橋單極性
//            Ikp = 0.051; Iki = 0.003;  //1150102原本Ikp = 0.056; Iki = 0.003;
            Vkp = 0.12; Vki = 0.11;
            vbus_ref = 405; 	//605
            Vctrl_Upper_limit = 0.98;

            EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
            EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
            EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

            EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
            EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
            GpioDataRegs.GPASET.bit.GPIO15 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
            break;

        case 3:     //高壓五階
//            Ikp = 0.049; Iki = 20;  //未調整
//            IkiTs = 0.0005;		//20*0.00002
            Vkp = 0.1; Vki = 0.09;  //未調整
            vbus_ref = 405;     //605
            Vctrl_Upper_limit = 0.99;

            EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
            EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
            EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
            GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;
            GpioDataRegs.GPASET.bit.GPIO26 = 1;
            break;

        case 4:     //高壓五階變形(低壓三階+單極性)      //尚未調整及測試		//暫時不使用
            Ikp = 0.055; Iki = 0.00006;  //未調整
            Vkp = 0.1; Vki = 0.09;  //未調整
            vbus_ref = 400;
            Vctrl_Upper_limit = 0.98;

            EPwm2Regs.CMPA.half.CMPA = 0;
            EPwm2Regs.CMPB = TBPRD_50Kset;

            EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
            EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
            EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
            EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

            EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
            EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
            EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
            EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;

            GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
            GpioDataRegs.GPASET.bit.GPIO15 = 1;
            GpioDataRegs.GPASET.bit.GPIO26 = 1;
            break;

//        case 5:     //高壓五階變形(低壓三階+單極性+高壓四階) //尚未調整及測試
//            Ikp = 0.055; Iki = 0.00006;  //未調整
//            Vkp = 0.1; Vki = 0.09;  //未調整
//            vbus_ref = 400;
//
//            EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
//            EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
//            EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
//            EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
//
//            EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
//            EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
//            EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
//            EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;
//
//            GpioDataRegs.GPASET.bit.GPIO23 = 1;
//            GpioDataRegs.GPASET.bit.GPIO15 = 1;
//            GpioDataRegs.GPASET.bit.GPIO26 = 1;
//            break;
    }
//----------------------------------------------------------------依照模式設定Ikp參數
	Calc_CurrentLoop_Kp();
//----------------------------------------------------------------
    PWMflag = 1;
    modulation = selected_mode;
    modulationLOCK = 1;
}

void Calc_CurrentLoop_Kp(void)
{
    inv_vbusref = __einvf32(vbus_ref);		//硬體快速倒數 (TI intrinsic)	//1/vbus_ref
    switch (selected_mode)
    {
        case 1:     //低壓三階
            Kp_Gain = 0.65;		//確認OK
            Ikp = L_over_Tsw * inv_vbusref * Kp_Gain;		//L/Ts/vbus*gain  //確認OK，之後再移至下方switch外
            break;

        case 2:     //全橋單極性
            Kp_Gain = 0.55;		//確認OK
            Ikp = L_over_Tsw * inv_vbusref * Kp_Gain;		//L/Ts/vbus*gain  //確認OK，之後再移至下方switch外
            break;

        case 3:     //高壓五階
            Kp_Gain = 0.52;		//確認OK
            Ikp = L_over_Tsw * inv_vbusref * Kp_Gain;		//L/Ts/vbus*gain  //確認OK，之後再移至下方switch外
            break;

        case 4:     //高壓五階變形(低壓三階+單極性)      //尚未調整及測試		//暫時不使用
//            Ikp = 0.055; //未調整
        	Kp_Gain = 0.8;//未調整
//            Ikp1 = L_over_Tsw * inv_vbusref * Kp_Gain;		//L/Ts/vbus*gain

            break;
    }
    Ikp1 = L_over_Tsw * inv_vbusref * Kp_Gain;		//L/Ts/vbus*gain
}

void switch_modulation(void)
{
//    Vctrl = 100; //TEST////	//指定固定哲任週期
//---------------------------------------------數值處理
    halfTBPRD_50Kset = TBPRD_50Kset >> 1;       //0.5*PRD  //450

    dutyset = (Uint16)(Vctrl * TBPRD_50Kset + 0.5f);	//D	//Duty*PRD後取成無號整數在寫入CMPA，更省CLK	//+0.5做四捨五入
    dutyinvert = TBPRD_50Kset - dutyset;        //1-D
    DoubleDutyCycle = dutyset << 1;				//2*D

    ONEMinusDoubleDuty = TBPRD_50Kset - DoubleDutyCycle;	//1-2*D
    if(ONEMinusDoubleDuty >= TBPRD_50Kset)
    {
    	ONEMinusDoubleDuty = TBPRD_50Kset;
    }
	else if(ONEMinusDoubleDuty <= 0)
	{
		ONEMinusDoubleDuty = 0;
	}

    DoubleDutyMinutONE = DoubleDutyCycle - TBPRD_50Kset;	//2*D-1
    if(DoubleDutyMinutONE >= TBPRD_50Kset)
    {
    	DoubleDutyMinutONE = TBPRD_50Kset;
    }
	else if(DoubleDutyMinutONE <= 0)
	{
		DoubleDutyMinutONE = 0;
	}

//---------------------------------------------Crossing Detect（邊緣偵測）+ LockFLAG 機制 (仿磁滯電路設計)
	ModeSelect_CrossDetect();
//----------------------------------------------------------------正半週
    if(bata1 > 0)        //正半週(integral1 < 1.5707 || integral1 > 4.7123)
    {
        EPwm1Regs.CMPA.half.CMPA = TBPRD_50Kset;    //TBPRD_50Kset
        EPwm1Regs.CMPB = 0;
        switch(modulation)
        {
            case 1:            //低壓三階
                EPwm2Regs.CMPA.half.CMPA = 0;
                EPwm2Regs.CMPB = 0;
                EPwm3Regs.CMPA.half.CMPA = dutyset;
                EPwm3Regs.CMPB = dutyset;
                break;
            case 2:        //全橋單極性
                EPwm2Regs.CMPB = dutyset;
                EPwm3Regs.CMPB = dutyset;
                break;
            case 3:        //高壓五階
                if(dutyset < halfTBPRD_50Kset)        //duty < 0.5
                {
                    VfiveCtrl = ONEMinusDoubleDuty;		//TBPRD_50Kset - DoubleDutyCycle;     //1-2*Vctrl
                    EPwm2Regs.CMPA.half.CMPA = VfiveCtrl;    //900-2*Vctrl
                    EPwm2Regs.CMPB = VfiveCtrl;
                    EPwm3Regs.CMPA.half.CMPA = 0;
                    EPwm3Regs.CMPB = 0;
//                    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //LED7TEST
//                    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3
                }
                else if(dutyset > halfTBPRD_50Kset)       //duty > 0.5
                {
                    VfiveCtrl = DoubleDutyMinutONE;		//DoubleDutyCycle - TBPRD_50Kset;     //2*Vctrl-900
                    EPwm2Regs.CMPA.half.CMPA = 0;
                    EPwm2Regs.CMPB = 0;
                    EPwm3Regs.CMPA.half.CMPA = VfiveCtrl;
                    EPwm3Regs.CMPB = VfiveCtrl;
//                    GpioDataRegs.GPASET.bit.GPIO27 = 1;   //LED7TEST
//                    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3TEST
                }
//				else
//				{
//					EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
//					EPwm2Regs.CMPB = 0;
//					EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
//					EPwm3Regs.CMPB = 0;
//					GpioDataRegs.GPASET.bit.GPIO22 = 1;   //LED3TEST
//				}
                break;
            case 4:        //高壓五階變形(低壓三階+單極性)		//暫時不使用
                if(!VctrlHalfState)        //duty < 0.5
                {
//                    VfiveCtrl = DoubleDutyCycle;     //2*Vctrl
                    EPwm2Regs.CMPA.half.CMPA = 0;    //900-2*Vctrl
                    EPwm2Regs.CMPB = dutyset;
                    EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
                    EPwm3Regs.CMPB = dutyset;
                }
                else if(VctrlHalfState)       //duty > 0.5
                {
                    VfiveCtrl = DoubleDutyMinutONE;		//DoubleDutyCycle - TBPRD_50Kset;     //2*Vctrl-900
                    EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
                    EPwm2Regs.CMPB = TBPRD_50Kset;
                    EPwm3Regs.CMPA.half.CMPA = VfiveCtrl;
                    EPwm3Regs.CMPB = VfiveCtrl;
                }
                break;
//            case 5:        //高壓五階變形(低壓三階+單極性+高壓四階)
//                if(dutyset < halfTBPRD_50Kset)        //duty < 0.5
//                {
////                    VfiveCtrl = DoubleDutyCycle;     //2*Vctrl
//                    EPwm2Regs.CMPA.half.CMPA = 0;    //900-2*Vctrl
//                    EPwm2Regs.CMPB = dutyset;
//                    EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
//                    EPwm3Regs.CMPB = dutyset;
//                }
//                else if(dutyset > halfTBPRD_50Kset)       //duty > 0.5
//                {
//                    VfiveCtrl = DoubleDutyCycle - TBPRD_50Kset;     //2*Vctrl-900
//                    EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
//                    EPwm2Regs.CMPB = TBPRD_50Kset;
//                    EPwm3Regs.CMPA.half.CMPA = VfiveCtrl;
//                    EPwm3Regs.CMPB = VfiveCtrl;
//                }
//                break;
        }
    }
//----------------------------------------------------------------負半週
    if(bata1 < 0)        //負半週(integral1 > 1.5707 && integral1 < 4.7123)
    {
        EPwm1Regs.CMPB = TBPRD_50Kset;
        EPwm1Regs.CMPA.half.CMPA = 0;

        switch(modulation)
        {
            case 1:             //低壓三階
                EPwm2Regs.CMPA.half.CMPA = dutyset;
                EPwm2Regs.CMPB = dutyset;
                EPwm3Regs.CMPA.half.CMPA = 0;
                EPwm3Regs.CMPB = 0;
                break;
            case 2:            //全橋單極性
                EPwm2Regs.CMPB = dutyinvert;
                EPwm3Regs.CMPB = dutyinvert;
                break;
            case 3:            //高壓五階
                if(dutyset < halfTBPRD_50Kset)        //duty < 0.5
                {
                    VfiveCtrl = ONEMinusDoubleDuty;		//TBPRD_50Kset - DoubleDutyCycle;     //1-2*Vctrl
                    EPwm2Regs.CMPA.half.CMPA = 0;
                    EPwm2Regs.CMPB = 0;
                    EPwm3Regs.CMPA.half.CMPA = VfiveCtrl;
                    EPwm3Regs.CMPB = VfiveCtrl;
//                    GpioDataRegs.GPASET.bit.GPIO27 = 1;   //LED7TEST
//                    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3TEST

                }
                else if(dutyset > halfTBPRD_50Kset)       //duty > 0.5
                {
                    VfiveCtrl = DoubleDutyMinutONE;     //2*Vctrl-900
                    EPwm2Regs.CMPA.half.CMPA = VfiveCtrl;
                    EPwm2Regs.CMPB = VfiveCtrl;
                    EPwm3Regs.CMPA.half.CMPA = 0;
                    EPwm3Regs.CMPB = 0;
//                    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //LED7TEST
//                    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3TEST
                }
//				else
//				{
//					EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset;
//					EPwm2Regs.CMPB = 0;
//					EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;
//					EPwm3Regs.CMPB = 0;
////                GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;   //TEST
//					GpioDataRegs.GPASET.bit.GPIO22 = 1;   //LED3TEST
//				}
                break;
            case 4:            //高壓五階變形(低壓三階+單極性)		//暫時不使用
                if(!VctrlHalfState)        //duty < 0.5
                {
                    EPwm2Regs.CMPA.half.CMPA = 0;
                    EPwm2Regs.CMPB = dutyinvert;
                    EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;     //900-2*Vctrl
                    EPwm3Regs.CMPB = dutyinvert;     //900-2*Vctrl

                }
                else if(VctrlHalfState)       //duty > 0.5
                {
                    VfiveCtrl = DoubleDutyCycle - TBPRD_50Kset;     //2*Vctrl-900
                    EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset - VfiveCtrl;
                    EPwm2Regs.CMPB = TBPRD_50Kset - VfiveCtrl;
                    EPwm3Regs.CMPA.half.CMPA = 0;
                    EPwm3Regs.CMPB = 0;
                }
                break;
//            case 5:            //高壓五階變形(低壓三階+單極性+高壓四階)
//                if(dutyset < halfTBPRD_50Kset)        //duty < 0.5
//                {
////                    VfiveCtrl = DoubleDutyCycle;     //2*Vctrl
//                    EPwm2Regs.CMPA.half.CMPA = 0;
//                    EPwm2Regs.CMPB = dutyinvert;
//                    EPwm3Regs.CMPA.half.CMPA = TBPRD_50Kset;     //900-2*Vctrl
//                    EPwm3Regs.CMPB = dutyinvert;     //900-2*Vctrl
//
//                }
//                else if(dutyset > halfTBPRD_50Kset)       //duty > 0.5
//                {
//                    VfiveCtrl = DoubleDutyCycle - TBPRD_50Kset;     //2*Vctrl-900
//                    EPwm2Regs.CMPA.half.CMPA = TBPRD_50Kset - VfiveCtrl;
//                    EPwm2Regs.CMPB = TBPRD_50Kset - VfiveCtrl;
//                    EPwm3Regs.CMPA.half.CMPA = 0;
//                    EPwm3Regs.CMPB = 0;
//                }
//                break;
        }
    }
}
//---------------------------------------------Crossing Detect（邊緣偵測）+ LockFLAG機制
void ModeSelect_CrossDetect(void)
{
//---------------------------------------------零交越區域不解鎖
    if(fabsf(bata1) < 5)		//(bata1 > -5 && bata1 < 5)
        return;									//離開副程式
//---------------------------------------------判斷DFF是在上升還是下降
    if(feedforward > DFF_prev)
    {
        DFF_direction = 1;                      //DFF上升
    }
    else if(feedforward < DFF_prev)
    {
        DFF_direction = 0;                     	//DFF下降
    }
    DFF_prev = feedforward;                     //Xdff[n-1]紀錄
//---------------------------------------------上鎖or解鎖(DFF狀態變化判斷)
    if(DFF_direction != DFF_direction_prev)
    {
        VcStateLOCK = 0;                        // 一旦變化就解鎖
        DFF_direction_prev = DFF_direction;     //儲存狀態
    }
    if(VcStateLOCK)
    {
        return;                                 //離開副程式
    }
//---------------------------------------------Crossing Detect（邊緣偵測）
    if(DFF_direction && dutyset > halfTBPRD_50Kset)      //上升穿越0.5	//Vctrl > 0.5	//dutyset > 450
    {
        VctrlHalfState = 1;                     // 0 : below 0.5, 1 : above 0.5
        VcStateLOCK = 1;                        //上鎖
//        GpioDataRegs.GPASET.bit.GPIO22 = 1;   //LED3TEST
    }

    else if(!DFF_direction && dutyset < halfTBPRD_50Kset)     //下降穿越0.5	//Vctrl < 0.5	//dutyset < 450
    {
        VctrlHalfState = 0;                     // 0 : below 0.5, 1 : above 0.5
        VcStateLOCK = 1;                        //上鎖
//        GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;   //LED3TEST
    }
}

void IrefSET (void)
{
//---------------------------------------------Vbus filiter
//////wc=2*pi*fc  //alpha = (wc * 2E-5) / (1 + wc * 2E-5)//////
//////fc=5 -> alpha = 0.000628  fc=10 -> alpha = 0.001255//////
    Vbus_filiter = Vbus_filiter_1 + 0.000628 * (vbus_real - Vbus_filiter_1);
    Vbus_filiter_1 =  Vbus_filiter;
//---------------------------------------------Voltage Loop電壓迴圈
    if(zero_cross_pos)          //零交越點一周一次(負轉正)   //bata1 > -0.5 && bata1 < 0.5 && integral1 > 4.5(原)
    {
        if(!VfbLock)            //VfbLock == 0
        {
            Verr_1 = Verr;
            Verr = vbus_ref - Vbus_filiter_1;
            Vpi = Vpi_1 + (Vkp + Vki * 0.01667) * Verr - Vkp * Verr_1;
            Imax = Vpi;
            VfbLock = 1;
        }
        if(Imax >= Imax_Upper_limit)
        {
            Imax = Imax_Upper_limit;
        }
        else if(Imax <= 0.1)
        {
            Imax = 0.1;
        }

        Vpi_1 = Imax;

//        ///test///////
//
//        if(v_index < N)
//        {
//            v_index++;      //60Hz
//        }
//        /////TEST/////

        if(Imax_Upper_limit >= ImaxSET)
        {
            Imax_Upper_limit = ImaxSET;
        }
        else
        {
            Imax_Upper_limit = Imax_Upper_limit + 0.006;
        }

    }
    else
    {
        VfbLock = 0;
    }

    if(fabsf(Verr) < 0.3)	//(Verr < 0.3 && Verr > -0.3)
    {
        GpioDataRegs.GPASET.bit.GPIO31 = 1;   //LED9
    }
    else
    {
        GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;     //LED9
    }
}
//---------------------------------------------Current Loop電流迴圈
void PIduty(void)
{
//---------------------------------------------數值處理
    Iacref = Imax * cos_value;		//iac_ref

    vacsign = (bata1 >= 0) ? 1 : -1;  //判別正負半週  //正半週為1  //負半週為-1  //cos_value

    vac_real = vacsign * bata1;

    Iref = vacsign * Iacref;
    Ifb = -vacsign * iac_real;

//---------------------------------------------Duty feedforward(DFF前饋)
//    inv_vbusref = __einvf32(vbus_ref);		//硬體快速倒數 (TI intrinsic)	//1/vbus_ref	//整體DFF部分省下2.56us  //搬至上方算KP處
    if(modulation == 1)
    {
        feedforward = 1 - ((vac_real + vac_real) * inv_vbusref);   //Computing speed slow->fast    //1-(vac/(vbus/2))=1-(vac/(vbus*0.5))=1-((vac*2)/vbus)=1-((vac+vac)/vbus)
    }
    else if(modulation == 2 || modulation == 3 || modulation == 4)
    {
        feedforward = 1 - (vac_real * inv_vbusref);
    }

    if(feedforward >= 0.99)
    {
        feedforward = 0.99;
    }
    else if(feedforward <= 0)
    {
        feedforward = 0;
    }
/*
//---------------------------------------------電流迴圈PI控制器
    Ierr_1 = Ierr;
    Ierr = Iref - Ifb;

    Ipi = Ipi_1 + (Ikp + Iki * 0.00002) * Ierr - Ikp * Ierr_1;      //0.00004
   // Ipi = Ikp * Ierr;     //純用KP不用KI

//---------------------------------------------Vctrl&上下限
    Vctrl = feedforward + Ipi;

    if(Vctrl >= Vctrl_Upper_limit)
    {
        Vctrl = Vctrl_Upper_limit;
        Ipi = Vctrl - feedforward;
    }
    else if(Vctrl <= 0)
    {
        Vctrl = 0;
        Ipi = - feedforward;
    }
    Ipi_1 = Ipi;

    */

//---------------------------------------------error
    Ierr = Iref - Ifb;	// 電流誤差
//---------------------------------------------KP、KI

//    Ipi_temp = Ipi_1 + (Ikp + Iki * 0.00002) * Ierr - Ikp * Ierr_1;	//KP+KI
//	Ipi_temp = Ipi_1 + (Ikp + IkiTs) * Ierr - Ikp * Ierr_1;		//KP+KI
    Ipi_temp = Ikp * Ierr;		//純KP
//    Ipi_temp = Ikp1 * Ierr;		//純KP	//TESTTESTTEST

    Vctrl_temp = feedforward + Ipi_temp;

//---------------------------------------------Vctrl&上下限
    if(Vctrl_temp > Vctrl_Upper_limit)
    {
        Vctrl = Vctrl_Upper_limit;
        Ipi_1 = Vctrl - feedforward;
    }
    else if(Vctrl_temp < 0)
    {
        Vctrl = 0;
        Ipi_1 = 0 - feedforward;
    }
    else
    {
    	Vctrl = Vctrl_temp;
    	Ipi_1 = Ipi_temp;
    }

    Ierr_1 = Ierr;
//---------------------------------------------存取數值
    if(SW2)
    {
    	v1[v_index] = Ipi_temp;
        v2[v_index] = Iref;
        v3[v_index] = Ifb;
        v4[v_index] = Vctrl;
        v5[v_index] = feedforward;
        v6[v_index] = Ipi_1;
        v7[v_index] = VfiveCtrl;
        v8[v_index] = Vbus_filiter_1;
    }
}
