#include "DSP28x_Project.h"
#include "F2806x_EPwm_defines.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "SFO_V6.h"
#include "nRF24L01.h"

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

//電流、電壓參數
Uint32 Iout = 2000;
float Vout = 0;
static float Iout_filt = 2000;
float alpha_I = 0.0314; //LPF
Uint16 LPFcount = 0;
Uint16 Rerf_I = 1000;
float IoutREF = -20.85;  //(A)
int Iout_adc = 0, Vout_adc = 0;
float Iout_real = 0, Iout_adc_LPF = 0;
float Vout_real = 0, Vout_adc_LPF = 0;
int16 Io_val = 0;	//通訊數值處理用
Uint16 Vo_val = 0;	//通訊數值處理用

//頻率濾波參數
static float f_filt = 0;
static float alpha_f = 0.1;

//protect保護
Uint16 Verror = 0;
Uint16 Ierror = 0;
Uint16 protectFLAG = 0;
float Iout_protect_value, IoutADC_protect_value, Vout_protect_value, VoutADC_protect_value;


Uint16 ZVS = 0;
int16 Z1 = 0;

//phase shift	//相移
float phi1 = 0;
float phi2 = 0;
Uint16 phi1_int = 0, phi2_int = 0;
float PHSsafe = 8, PHSdiff1 = 0, PHSdiff2 = 0;
Uint16 CENTER = 225;

//控制要用到的變數
volatile Uint16 duty_enable = 0;
volatile Uint16 sync_step = 0;  //從內部轉到外部同步
Uint16 state = 0;
float d1 = 0, PHSrate = 0, PBSzero = 0;
float duty_inv = 0, base = 0, delta = 0;
float duty_now = 0.0f;

//PI
volatile Uint32 duty_deg = 1000;      //每幾次中斷才移動一次
volatile Uint32 duty_count = 0;       //計數器
volatile Uint32 pi_deg = 1000;        //每幾次中斷(20)才計算pi
volatile Uint32 pi_count = 0;         //計數器
Uint16 led_tick_count = 0;

float delta_alpha =0;   //alpha
float alpha = 0;
float a1 = 0;
float drx = 0; // 國科會

volatile Uint32 f_count = 0;

static Uint16 f_buf[2] = {900,900};//filter
Uint16 a = 0;
Uint16 b = 0;
Uint16 c = 0;
Uint16 d = 0;

static Uint16 last_hardware_prd = 450;
Uint16 DBvalue = 100;	//DBvalue*5.5ns
Uint16 new_prd = 0;
Uint32 f0 = 0;

//數值存取
#define N 417   // 陣列大小417
volatile float v1[N], v2[N], v3[N], v4[N], v5[N];
int M_index = 0;

Uint16 SW1 = 0, SW2 = 0;
//////////////////////////////確認緩啟動無問題

void Adc(void);       		//輸出電壓電流取樣
void EPWM_S7S8(void); 		//開關S7跟S8
void EPWM_S5S6(void); 		//開關S5跟S6
void GPIO_SET(void);
void eCAP1(void);
void eCAP2(void);
void eCAP3(void);
void f_cal(void);
void p_cal(void);
void Adc_LPF(void);
void protect(void);
void Rectifier_Ctrl(void);
void Phase_Shift_Calc(void);

interrupt void EPWM1_ISR(void);
__interrupt void cpu_timer0_isr(void);

int MEP_ScaleFactor = 0;

volatile struct EPWM_REGS *ePWM[] =
{
    0,               // index 0 不使用（為了與 ePWM1~對齊）
    &EPwm1Regs,      // ePWM1
    &EPwm2Regs       // ePWM2
};

//----------------------------------------------------------------SPI
#define TX_EN   1       // select device is TX and / or
//#define RX_EN   1     // RX

nRF24L01_Vars_t nRF_TX = NRF24L01_DEFAULTS;
//nRF24L01_Vars_t nRF_RX = NRF24L01_DEFAULTS;

//uint16_t rxData;
uint16_t txData;

// ----------------------------------------------------------------------
// Configure Spi Register with FIFO use and no interrupt // 4 Wire
// ----------------------------------------------------------------------
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

void main(void)
{
    InitSysCtrl();
    DINT;
    InitPieCtrl();

    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();
    InitGpio();
    InitAdc();
    InitEPwm();
    InitCpuTimers();

    while(SFO() == 0)
    {

    }

    int status;
    status = SFO();
    if(status==2) {ESTOP0;}

// ----------------------------------------------------------------------
// TIMERS
// ----------------------------------------------------------------------
	InitCpuTimers();   // For this example, only initialize the Cpu Timers

	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 90MHz CPU Freq, 1 second Period (in uSeconds)
	ConfigCpuTimer(&CpuTimer0, 90, 1300);           //1.3ms

	// To ensure precise timing, use write-only instructions to write to the
	// entire register. Therefore, if any of the configuration bits are changed
	// in ConfigCpuTimer and InitCpuTimers (in F2806x_CpuTimers.h), the below

	// Use write-only instruction to set TSS bit = 0
	CpuTimer0Regs.TCR.all = 0x4000;

// ----------------------------------------------------------------------
// SPI
// ----------------------------------------------------------------------
    InitSpiaGpio();
//    InitSpibGpio();
    SPI_Config4Wire(&SpiaRegs);
//    SPI_Config4Wire(&SpibRegs);

#ifdef TX_EN
// ----------------------------------------------------------------------
// nRF24 TX settings
// ----------------------------------------------------------------------
    nRF_TX.cePin  = 16; //50
    nRF_TX.csnPin = 20; //51
    nRF_TX.irqPin = 22;
    nRF_TX.SPI_Regs = &SpiaRegs;

// set GPIOs here cause there is no function so far in init
    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;   // CE low   //16
    GpioDataRegs.GPASET.bit.GPIO20   = 1;   // CSN high //18

    EALLOW;
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;     // CE pin output  //16
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;     // CSN pin output //18
    EDIS;

// Step 1: get all initial values from nRF24 and make basic setting
    nRF24_Init(&nRF_TX);

// Step 2: set address width, RX & TX address afterwards
    nRF_TX.SETUP_AW.bit.AW = nRF_5Bytes;
    nRF24_SPI_Write(&nRF_TX, nRF24_SETUP_AW);

    nRF_TX.TX_ADDR[0] = 0xDE;
    nRF_TX.TX_ADDR[1] = 0xAD;
    nRF_TX.TX_ADDR[2] = 0xBE;
    nRF_TX.TX_ADDR[3] = 0xEF;
    nRF_TX.TX_ADDR[4] = 0xF0;
    nRF24_SetTxAddress(&nRF_TX);

// Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
// needs to listen to the TX addr on pipe#0 to receive them.
    nRF_TX.RX_ADDR_P0[0] = 0xDE;
    nRF_TX.RX_ADDR_P0[1] = 0xAD;
    nRF_TX.RX_ADDR_P0[2] = 0xBE;
    nRF_TX.RX_ADDR_P0[3] = 0xEF;
    nRF_TX.RX_ADDR_P0[4] = 0xF0;
    nRF24_SetRxAddress(&nRF_TX, nRF24_PIPE_0);

// Step 3: open used pipes & set payload width
    nRF_TX.EN_RXADDR.bit.ERX_P0 = 1;    // enable PIPE 0
    nRF24_SPI_Write(&nRF_TX, nRF24_EN_RXADDR);

    nRF_TX.RX_PW_P0.all = (uint16_t)nRF24_PAYLOAD_WIDTH;
    nRF24_SPI_Write(&nRF_TX, nRF24_RX_PW_P0);

// Step 4: speed, power & channel setting
    nRF24_SetDataRate(&nRF_TX, nRF_2Mbps);
    nRF_TX.RF_SETUP.bit.RF_PWR =nRF_minus0dBm;
    nRF24_SPI_Write(&nRF_TX, nRF24_RF_SETUP);
    nRF_TX.RF_CH.bit.RF_CH = 120;
    nRF24_SPI_Write(&nRF_TX, nRF24_RF_CH);

// Step 5: power up and set to standby
    nRF24_Standby(&nRF_TX);

// only for debug
    nRF24_GetRegValues(&nRF_TX);
#endif
//----------------------------------------------------------------

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 0;
    EDIS;

    EALLOW;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.EPWM1_INT = &EPWM1_ISR;
    EDIS;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  //ePWM1中斷
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    IER |= M_INT3;
    IER |= M_INT1;
    IER |= M_INT13;

    EINT;
    ERTM;

    Adc();
    eCAP1();
    EPWM_S5S6();
    EPWM_S7S8();
    GPIO_SET();

    EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA = 1;		// 1: Force Continuous Low on EPWM1A
    EPwm1Regs.AQCSFRC.bit.CSFB = 1;		// 1: Force Continuous Low on EPWM1B
    EPwm2Regs.AQCSFRC.bit.CSFA = 2;		// 2: Force Continuous High on EPWM2A
    EPwm2Regs.AQCSFRC.bit.CSFB = 2;		// 2: Force Continuous High on EPWM2B
    EDIS;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    for(;;)
    {
        drx = (((phi1-delta_alpha+26)*100) / (last_hardware_prd));	//未知 不懂
        alpha = (((d1 + 51 + a1) *180)/ last_hardware_prd );		//未知 不懂
        ZVS = (phi1 > (float)d1);									//未知 不懂

        SW1 = GpioDataRegs.GPADAT.bit.GPIO27;
    }
}

interrupt void EPWM1_ISR(void)
{
    GpioDataRegs.GPASET.bit.GPIO19 = 1;			//LED6
//----------------------------------------------------------------取樣
    Iout = AdcResult.ADCRESULT5;
    Vout = AdcResult.ADCRESULT4;

    Iout_adc = AdcResult.ADCRESULT5 - 1991;
    Vout_adc = AdcResult.ADCRESULT4 - 7;
    Adc_LPF();		//低通濾波器LPF
    Iout_real = Iout_adc_LPF * 0.015906;
    Vout_real = Vout_adc_LPF * 0.020806;
//----------------------------------------------------------------保護
    protect();			// --- protect保護 (state = 1) ---
//----------------------------------------------------------------主控制區塊
    if(state != 1)		// --- 2. 控制區塊 ---
    {
    	Rectifier_Ctrl();
    }
//----------------------------------------------------------------END
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;		//LED6
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

void protect(void)
{
    // --- protect保護 (state = 1) ---
    if(((Iout_filt > 1) && (Iout_filt < 419)) || (Vout_real > 64))	//約-25A、62.3V	//原(Vout > 3000)
    {
    	//PWM訊號強制變動
        EALLOW;
        EPwm1Regs.AQCSFRC.bit.CSFA = 1;		// 1: Force Continuous Low on EPWM1A
        EPwm1Regs.AQCSFRC.bit.CSFB = 1;		// 1: Force Continuous Low on EPWM1B
        EPwm2Regs.AQCSFRC.bit.CSFA = 2;		// 2: Force Continuous High on EPWM2A
        EPwm2Regs.AQCSFRC.bit.CSFB = 2;		// 2: Force Continuous High on EPWM2B
        EDIS;

        //紀錄保護時數值
        Iout_protect_value = Iout_real;
		IoutADC_protect_value = Iout_filt;
		Vout_protect_value = Vout_real;
		VoutADC_protect_value = Vout;

		//紀錄保護時旗標
        if((Iout_filt > 1) && (Iout_filt < 400))
        {
            Ierror = 1;
        }
        if(Vout_real > 64)
        {
            Verror = 1;
        }

        //數值清理
        duty_enable = 0;
        state = 1;		//進保護就死機
        protectFLAG = 1;

        //燈號變動
        GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;			//LED7
        GpioDataRegs.GPBSET.bit.GPIO33 = 1;				//LED1
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;			//LED3TEST
    }
}
void Rectifier_Ctrl(void)
{
//----------------------------------------------------------------待機狀態
    if(ECap1Regs.TSCTR > 2000)				// === 失去同步訊號 待機狀態===
    {
    	//PWM訊號強制變動
        EALLOW;
        EPwm1Regs.AQCSFRC.bit.CSFA = 1;		// 1: Force Continuous Low on EPWM1A
        EPwm1Regs.AQCSFRC.bit.CSFB = 1;		// 1: Force Continuous Low on EPWM1B
        EPwm2Regs.AQCSFRC.bit.CSFA = 2;		// 2: Force Continuous High on EPWM2A
        EPwm2Regs.AQCSFRC.bit.CSFB = 2;		// 2: Force Continuous High on EPWM2B
        EDIS;

        //數值清理
        duty_enable = 0;
        state = 2;
        sync_step = 0;
        d1 = 0;
        duty_now = 0;
        EPwm1Regs.TBPHS.half.TBPHS = 0;
        EPwm2Regs.TBPHS.half.TBPHS = 0;

        //燈號變動
        GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;		//LED7
        GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;		//LED3TEST
        GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;		//LED4TEST
        duty_count = 0;								// 重置計算變數
    }
//----------------------------------------------------------------工作狀態
    else							// === 工作狀態 ===
    {
        f_cal();					//變頻算頻率
        p_cal();					//角度計算

        // 同步邏輯 (轉成外部同步)
        if(!sync_step)				//PWM 同步啟動（Synchronous Startup，重啟時執行一次
        {
            EALLOW;
            EPwm1Regs.TBPRD = last_hardware_prd;
            EPwm2Regs.TBPRD = last_hardware_prd;
            EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;		//master:0 Slave:1 module
            EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;		//master:0 Slave:1 module
            EDIS;
            sync_step = 1;
        }

        // 狀態機
        switch(state)
        {
            case 2:					//待機等待緩啟動
                if(!GpioDataRegs.GPADAT.bit.GPIO14)		//GpioDataRegs.GPADAT.bit.GPIO14 == 0
                {
                    duty_enable = 1;
                    state = 3;		//進入控制模式
                    EALLOW;
                    EPwm1Regs.AQCSFRC.all = 0;			//解開「軟體強制鎖定」
                    EPwm2Regs.AQCSFRC.all = 0;			//解開「軟體強制鎖定」
                    EDIS;
                }
                if(++led_tick_count >= 10000)			//幾次中斷才運算移動一次
                {
                	led_tick_count = 0;
                	GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1;	//LED7
                }
                break;

            case 3:					//GPIO14按下後，進入狀態3，計算重疊角度
                if(++duty_count >= duty_deg)			//幾次中斷才運算移動一次
                {
                    duty_count = 0;
                    if(duty_now < 0.85)
                    {
                    	duty_now += 0.006;				//duty_now < 0.83	//+= 0.01	//+= 0.005
                    	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;	//LED3TEST
                    }
                    else
                    {
//                    	if(!GpioDataRegs.GPADAT.bit.GPIO14)		//手動跳case
//                    	{
//                    		state = 4;
//                    	}
                    	if(++led_tick_count >= 50)		//幾次中斷才運算移動一次
                    	{
                    		led_tick_count = 0;
                    		state = 4;
                    		GpioDataRegs.GPATOGGLE.bit.GPIO29 = 1;	//LED3TEST
                    	}
                    }
                    GpioDataRegs.GPASET.bit.GPIO13 = 1;	//LED7
                }
                break;

            case 4:										//微調修正&回授控制在額定輸出電流
            	GpioDataRegs.GPASET.bit.GPIO29 = 1;		//LED3TEST
                if(++pi_count >= pi_deg)				//幾次中斷才運算移動一次
                {
                    pi_count = 0;

                    if(IoutREF < Iout_real)
                    {
                    	duty_now += 0.0002f;			//0.005
                    }
                    else if(IoutREF > Iout_real)
                    {
                    	duty_now -= 0.0002f;			//0.005
                    }


                    if(duty_now > 1.0f)					//上限
                    {
                    	duty_now = 1.0f;
                    	GpioDataRegs.GPASET.bit.GPIO23 = 1;			//LED4TEST
                    }
                    else
                    {
                    	GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;		//LED4TEST
                    }

                    if(duty_now < 0.2f) 				//下限
					{
                    	duty_now = 0.2f;
//                    	GpioDataRegs.GPASET.bit.GPIO21 = 1;			//LED5TEST
					}
//                    else
//                    {
//                    	GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;		//LED5TEST
//                    }

                    //TEST紀錄數值
        			if(M_index < N)
        				M_index++;
        			if(M_index >= N)
        				M_index = 0;
                }
                break;
        }

//----------------------------------------------------------------更新相位暫存器(只在訊號正常時更新)
        Phase_Shift_Calc();
//----------------------------------------------------------------存數值count
		if(SW1)
		{
			v1[M_index] = Iout_real;
			v2[M_index] = phi2_int;
			v3[M_index] = duty_now;
			v4[M_index] = Iout_adc_LPF;
			v5[M_index] = Vout_real;
//			if(M_index < N)
//				M_index++;
//			if(M_index >= N)
//				M_index = 0;
		}
    }
}

void f_cal(void)
{
//----------------------------------------------------------------失去同步訊號
    if(ECap1Regs.TSCTR > 2000)		// === 失去同步訊號 待機狀態 直接離開副程式===
    {
        return;
    }
//----------------------------------------------------------------正常工作狀態
    //1.取樣頻率
    a = ECap1Regs.CAP2;
    b = f_buf[0];
    c = f_buf[1];

    //2.突波濾除
    if((a <= b && b <= c) || (c <= b && b <= a))
        d = b;
    else if((b <= a && a <= c) || (c <= a && a <= b))
        d = a;
    else
        d = c;

	f_buf[1] = b;
	f_buf[0] = a;

    //3.一階低通濾波
    f_filt += (alpha_f * (float)(d - f_filt));
    new_prd = (Uint16)(f_filt * 0.5f);

    //4.變頻率
    if(++f_count >= 1)		//分頻機制1010		//兩次算一次
    {
        f_count = 0;
        // 1. 如果新計算的值比目前硬體值大，步長加 1	//約減少222Hz
        if(new_prd > last_hardware_prd)
        {
           last_hardware_prd += 1;
        }

        // 2. 如果新計算的值比目前硬體值小，步長減 1	//約增加222Hz
        if(new_prd < last_hardware_prd)
        {
           last_hardware_prd -= 1;
        }

//        變頻寫入1
//        EPwm1Regs.TBPRD = last_hardware_prd;
//        EPwm2Regs.TBPRD = last_hardware_prd;
//        EPwm1Regs.CMPA.half.CMPA       = (last_hardware_prd >> 1);
//        EPwm2Regs.CMPA.half.CMPA       = (last_hardware_prd >> 1);
//        PHSsafe   = last_hardware_prd * 0.02;

        //變頻寫入2
        //        EPwm1Regs.TBPRD = last_hardware_prd;
        //        EPwm2Regs.TBPRD = last_hardware_prd;
        //        CENTER = last_hardware_prd >> 1;
        //        EPwm1Regs.CMPA.half.CMPA       = CENTER;
        //        EPwm2Regs.CMPA.half.CMPA       = CENTER;
        //        PHSsafe   = last_hardware_prd * 0.02;

        //定頻寫入
        last_hardware_prd = 450;	//TEST
        EPwm1Regs.TBPRD = 450;
        EPwm2Regs.TBPRD = 450;
        CENTER = last_hardware_prd >> 1;
        EPwm1Regs.CMPA.half.CMPA = CENTER;
        EPwm2Regs.CMPA.half.CMPA = CENTER;
        PHSsafe   = last_hardware_prd * 0.023;
    }
}
//角度計算
void p_cal(void)
{
    if(!duty_enable)						//duty_enable==0
    {
        EPwm2Regs.TBPHS.half.TBPHS = (last_hardware_prd - DBvalue +2);
    }
    d1 = (CENTER - (DBvalue >> 1) + 1);   	//重疊中心點
    a1 = (last_hardware_prd * 0.0138888);   //(alpha/180)	//alpha=14˙(0.0777778)	//5˙
}

void Adc_LPF(void)
{
    Iout_filt = (alpha_I * Iout)+ ((1.0 - alpha_I) * Iout_filt);	//電流平均

    if(++ LPFcount >= 10)					//降頻Div10	//100kHz->10kHz
    {
    	LPFcount = 0;

		Iout_adc_LPF = (0.0062439 * Iout_adc)+ ((0.9937561) * Iout_adc_LPF);		//電流平均	//fc=10Hz

		Vout_adc_LPF = (0.0062439 * Vout_adc)+ ((0.9937561) * Vout_adc_LPF);		//fc=10Hz
    }
}

void Phase_Shift_Calc(void)
{
//----------------------------------------------------------------更新相位暫存器(只在訊號正常時更新)
//      PHSrate = 1.0f - duty_now * 0.1;
//		phi1 = d1 * PHSrate - ((1.0f - duty_now) * d1 * PHSrate) + a1;
//		phi2 = d1 * PHSrate + ((1.0f - duty_now) * d1 * PHSrate) + a1;

      	if(duty_now == 1)
      	{
      		PBSzero = 0;
      		PHSrate = 0.9;
      	}
      	else
      	{
      		PBSzero = ((1.0f - duty_now) * (d1 * PHSrate));
      		PHSrate = 1.0f - duty_now * 0.1;
      	}
		phi1 = d1 * PHSrate + a1 - PBSzero;
		phi2 = d1 * PHSrate + a1 + PBSzero;

//		PHSrate = 1.0f - duty_now * 0.1;
//		duty_inv = 1.0f - duty_now;
//		base = d1 * PHSrate;
//		delta = duty_inv * base;
//		phi1 = base - delta + a1;
//		phi2 = base + delta + a1;

//----------------------------------------------------------------相位保護
//		CENTER = last_hardware_prd >> 1;
//		PHSsafe   = last_hardware_prd * 0.02;
		PHSdiff1 = phi1 - CENTER;
		if(fabsf(PHSdiff1) < PHSsafe)
		{
			phi1 += (PHSdiff1 >= 0) ? PHSsafe : -PHSsafe;
		}

		PHSdiff2 = phi2 - CENTER;
		if(fabsf(PHSdiff2) < PHSsafe)
		{
			phi2 += (PHSdiff2 >= 0) ? PHSsafe : -PHSsafe;
		}

//----------------------------------------------------------------相位下限
		if(phi1 < 0.0f)		//下限
			phi1 = 0.0f;
		if(phi2 < 0.0f)		//下限
			phi2 = 0.0f;

//----------------------------------------------------------------寫入暫存器
		phi1_int = (Uint16)(phi1 + 0.5f);	//四捨五入轉整數
		phi2_int = (Uint16)(phi2 + 0.5f);	//四捨五入轉整數

		EPwm1Regs.TBPHS.half.TBPHS = phi1_int;
		EPwm2Regs.TBPHS.half.TBPHS = phi2_int;
}


void Adc(void)
{
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;    	// Enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS   = 1;    	// ADCINT1 trips after AdcResults latch
    AdcRegs.INTSEL1N2.bit.INT1E       = 1;    	// enable ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT    = 0;    	// Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL     = 4;    	// setup EOC1 to trigger ADCINT1 to fire

    //ADCINA0
    AdcRegs.ADCSOC4CTL.bit.CHSEL      = 4;    	// ADCINA4
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL    = 5;    	// trigger on EPWM1A
    AdcRegs.ADCSOC4CTL.bit.ACQPS      = 9;    	// 10 ADCCLK
    //ADCINA1
    AdcRegs.ADCSOC5CTL.bit.CHSEL      = 5;    	// ADCINA5
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL    = 5;    	// trigger on EPWM1A
    AdcRegs.ADCSOC5CTL.bit.ACQPS      = 9;    	// 10 ADCCLK

    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;		  	// 啟用 SOCA 觸發
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTRU_CMPA;
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     	// 每次事件都觸發一次採樣
    EDIS;
}

void eCAP1(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO24    = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO24   = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO24    = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24  = 0;
    GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 10; //取樣週期

    ECap1Regs.ECCTL1.bit.CAP1POL = 1; 		// Event1 down edge
    ECap1Regs.ECCTL1.bit.CAP2POL = 0; 		// Event2 rise edge
    ECap1Regs.ECCTL1.bit.CTRRST1 = 0; 		// Event1 reset counter
    ECap1Regs.ECCTL1.bit.CTRRST2 = 1; 		// Event2 reset counter
    ECap1Regs.ECCTL1.bit.CTRRST3 = 0; 		// Event3 reset counter
    ECap1Regs.ECCTL1.bit.CTRRST4 = 0; 		// Event4 reset counter
    ECap1Regs.ECCTL1.bit.CAPLDEN = 1; 		// Enable CAP register loads

    ECap1Regs.ECCTL2.bit.CONT_ONESHT = 0; 	// Continuous mode
    ECap1Regs.ECCTL2.bit.TSCTRSTOP   = 1; 	// Enable counter
    ECap1Regs.ECCTL2.bit.SYNCI_EN    = 0; 	// Enable sync in
    ECap1Regs.ECCTL2.bit.SYNCO_SEL   = 0; 	// Pass through sync signal
    ECap1Regs.ECCTL2.bit.STOP_WRAP   = 1;

    ECap1Regs.ECEINT.bit.CEVT1 = 0;      	// 不開中斷on capture event 1
    ECap1Regs.ECCLR.all        = 65535;  	// Clear all interrupt flags
    ECap1Regs.ECEINT.all = 0;
    EDIS;
}

void eCAP2(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO25   = 1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO25  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO25   = 0;
    EDIS;

    EALLOW;
    ECap2Regs.ECCTL1.bit.CAP1POL = 0; 		// Event1 rising edge
    ECap2Regs.ECCTL1.bit.CAP2POL = 1; 		// Event2 rising edge
    ECap2Regs.ECCTL1.bit.CTRRST1 = 0; 		// Event1 reset counter
    ECap2Regs.ECCTL1.bit.CTRRST2 = 1; 		// Event2 reset counter
    ECap2Regs.ECCTL1.bit.CAPLDEN = 1; 		// Enable CAP register loads

    ECap2Regs.ECCTL2.bit.CONT_ONESHT = 0; 	// Continuous mode
    ECap2Regs.ECCTL2.bit.TSCTRSTOP   = 1; 	// Enable counter
    ECap2Regs.ECCTL2.bit.SYNCI_EN    = 1; 	// Enable sync in
    ECap2Regs.ECCTL2.bit.SYNCO_SEL   = 0; 	// Pass through sync signal

    ECap2Regs.ECEINT.bit.CEVT2 = 1;      	// Enable interrupt on capture event 2
    ECap2Regs.ECCLR.all        = 65535;  	// Clear all interrupt flags
    ECap2Regs.ECEINT.all       = 4;      	// Enable CEVT2 interrupt (bit2)
    EDIS;
}

void eCAP3(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO26   = 1;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPAMUX2.bit.GPIO26  = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO26   = 0;
    EDIS;

    EALLOW;
    ECap3Regs.ECCTL1.bit.CAP1POL = 0;		// Event1 rise edge
    ECap3Regs.ECCTL1.bit.CAP2POL = 1;		// Event2 down edge
    ECap3Regs.ECCTL1.bit.CTRRST1 = 0;		// Event1 reset counter
    ECap3Regs.ECCTL1.bit.CTRRST2 = 1;		// Event2 reset counter
    ECap3Regs.ECCTL1.bit.CAPLDEN = 1;		// Enable CAP register loads

    ECap3Regs.ECCTL2.bit.CONT_ONESHT = 0;	// Continuous mode
    ECap3Regs.ECCTL2.bit.TSCTRSTOP   = 1; 	// Enable counter
    ECap3Regs.ECCTL2.bit.SYNCI_EN    = 1; 	// Enable sync in
    ECap3Regs.ECCTL2.bit.SYNCO_SEL   = 0; 	// Pass through sync signal

    ECap3Regs.ECEINT.bit.CEVT2 = 1;     	// Enable interrupt on capture event 2
    ECap3Regs.ECCLR.all        = 0xFFFF;	// Clear all interrupt flags
    ECap3Regs.ECEINT.all       = 0x0004;	// Enable CEVT2 interrupt (bit2)
    EDIS;
}

void EPWM_S5S6(void)
{
    //ePWM1 GPIO 00 01 設定
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0  = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO1  = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
//    //TZ1 GPIO12 設定
//    GpioCtrlRegs.GPAPUD.bit.GPIO12   = 0;
//    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 0;
//    GpioCtrlRegs.GPAMUX1.bit.GPIO12  = 0;

    //TB設定
    EPwm1Regs.TBPRD               = 450;
    EPwm1Regs.TBPHS.half.TBPHS    = 0;
    EPwm1Regs.TBCTR               = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;;    	//Up-down-count mode
    EPwm1Regs.TBCTL.bit.PHSEN     = TB_DISABLE;    			//master:0 Slave:1 module
    EPwm1Regs.TBCTL.bit.PRDLD     = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;    			//Synchronization Output Select.
    EPwm1Regs.TBCTL.bit.SWFSYNC   = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV    = TB_DIV1;
    EPwm1Regs.TBCTL.bit.PHSDIR    = 0;
    //CC設定
    EPwm1Regs.CMPA.half.CMPA       = 225;
    EPwm1Regs.CMPB                 = 225;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;			// 0: Load CMPA from shadow on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;			// 0: Load CMPB from shadow on CTR=Zero
    //AQ設定
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CAD = AQ_SET;
    //DB設定
    EPwm1Regs.DBCTL.bit.HALFCYCLE = 1;                		// 1: Enable half-cycle clocking for higher resolution
    EPwm1Regs.DBCTL.bit.OUT_MODE  = DB_FULL_ENABLE;   		// 3: Dead-band fully enabled for both rising and falling edge
    EPwm1Regs.DBCTL.bit.POLSEL    = DB_ACTV_HIC;      		// 2: Active Hi Complementary (ePWMxB is inverted)
    EPwm1Regs.DBCTL.bit.IN_MODE   = DBA_ALL;          		// 2: ePWMA is the source for both rising and falling edge
    EPwm1Regs.DBFED              = DBvalue;  				// 1 clock:5.5ns
    EPwm1Regs.DBRED              = DBvalue;
    //ET設定
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;				// 1: Select event CTR = Zero for interrupt
    EPwm1Regs.ETSEL.bit.INTEN  = 1;     					// 開PWM中斷
    EPwm1Regs.ETPS.bit.INTPRD  = ET_1ST;					// 1: Generate interrupt on the 1st event
    EPwm1Regs.ETCLR.bit.INT    = 1;
    //TZ設定
    EPwm1Regs.TZSEL.all       = 0;
    EPwm1Regs.TZSEL.bit.CBC1  = 0;
    EPwm1Regs.TZCTL.bit.TZA   = TZ_FORCE_LO; 				// 2: Force EPWM1A to Low (Safe state)	//輸出為low
    EPwm1Regs.TZCTL.bit.TZB   = TZ_FORCE_LO; 				// 2: Force EPWM1B to Low (Safe state)	//輸出為low
    EPwm1Regs.TZCLR.bit.CBC   = 1;							// Clear Cycle-by-Cycle flag
    EPwm1Regs.TZCLR.bit.INT   = 1;							// Clear Interrupt flag
    //啟用HRPWM
//    EPwm1Regs.HRCNFG.all          = 0;
//    EPwm1Regs.HRCNFG.bit.EDGMODE  = 3;      				// Both Edge
//    EPwm1Regs.HRCNFG.bit.CTLMODE  = 0;     	 			// CMPAHR 控制
//    EPwm1Regs.HRCNFG.bit.HRLOAD   = 0;      				// Zero 時更新
//    EPwm1Regs.HRCNFG.bit.AUTOCONV = 1;      				// 啟用自動轉換
//    EPwm1Regs.HRPCTL.bit.HRPE     = 1;      				// HRPWM Enable
    EDIS;
}

void EPWM_S7S8(void)
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

    //TB設定
    EPwm2Regs.TBPRD               = 450;
    EPwm2Regs.TBPHS.half.TBPHS    = 0;   //相位
    EPwm2Regs.TBCTR               = 0;
    EPwm2Regs.TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;		//Up-down-count mode
    EPwm2Regs.TBCTL.bit.PHSEN     = TB_ENABLE;
    EPwm2Regs.TBCTL.bit.PRDLD     = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL  = TB_SYNC_IN;   			//Synchronization Output Select.
    EPwm2Regs.TBCTL.bit.SWFSYNC   = 0;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV    = TB_DIV1;
    EPwm2Regs.TBCTL.bit.PHSDIR    = 0;   					//Count down:0 up:1
    //CC設定
    EPwm2Regs.CMPA.half.CMPA       = 225;
    EPwm2Regs.CMPB                 = 225;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;			// 0: Load CMPA from shadow on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;			// 0: Load CMPB from shadow on CTR=Zero
    //AQ設定
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLB.bit.CAD = AQ_SET;
    //DB設定
    EPwm2Regs.DBCTL.bit.HALFCYCLE 	= 1;                	// 1: Enable half-cycle clocking for higher resolution
    EPwm2Regs.DBCTL.bit.OUT_MODE  	= DB_FULL_ENABLE;   	// 3: Dead-band fully enabled for both rising and falling edge
    EPwm2Regs.DBCTL.bit.POLSEL    	= DB_ACTV_HIC;      	// 2: Active Hi Complementary (ePWMxB is inverted)
    EPwm2Regs.DBCTL.bit.IN_MODE   	= DBA_ALL;          	// 2: ePWMA is the source for both rising and falling edge
    EPwm2Regs.DBFED              	= DBvalue;
    EPwm2Regs.DBRED              	= DBvalue;
    //ET設定
    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;				// 1: Select event CTR = Zero for interrupt
    EPwm2Regs.ETSEL.bit.INTEN  = 1;
    EPwm2Regs.ETPS.bit.INTPRD  = ET_1ST;					// 1: Generate interrupt on the 1st event
    EPwm2Regs.ETCLR.bit.INT    = 1;
    //TZ設定
    EPwm2Regs.TZSEL.all      = 0;
    EPwm2Regs.TZSEL.bit.CBC1 = 0;
    EPwm2Regs.TZCTL.bit.TZA  = TZ_FORCE_LO; 				// 2: Force EPWM2A to Low (Safe state)	//輸出為low
    EPwm2Regs.TZCTL.bit.TZB  = TZ_FORCE_LO; 				// 2: Force EPWM2A to Low (Safe state)	//輸出為low
    EPwm2Regs.TZCLR.bit.CBC  = 1;							// Clear Cycle-by-Cycle flag
    EPwm2Regs.TZCLR.bit.INT  = 1;							// Clear Interrupt flag

    //啟用HRPWM
//    EPwm2Regs.HRCNFG.all          = 0;
//    EPwm2Regs.HRCNFG.bit.EDGMODE  = 3;
//    EPwm2Regs.HRCNFG.bit.CTLMODE  = HR_CMP;
//    EPwm2Regs.HRCNFG.bit.HRLOAD   = HR_CTR_ZERO;
//    EPwm2Regs.HRCNFG.bit.AUTOCONV = 1;
//    EPwm2Regs.HRPCTL.bit.HRPE     = 1;
    EDIS;
}
void GPIO_SET(void)
{
    EALLOW;

    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;		//LED7
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;		//SW2
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;			//一次側緩啟動完成
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;
    GpioDataRegs.GPASET.bit.GPIO14 = 1;   		// 設high

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;		//LED6
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;		//LED5
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;		//LED4
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;		//SW1
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO29=0;			//LED3
    GpioCtrlRegs.GPADIR.bit.GPIO29=1;
    GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;

    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;		//LED2
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;			//SYNCIN
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;		//外部同步輸入
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 2;

    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;		//LED1
    GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;

    EDIS;
}

__interrupt void cpu_timer0_isr(void)
{
//    CpuTimer0.InterruptCount++;
#ifdef TX_EN
    Io_val = (int16)(Iout_real * 100);			//輸出電流有正有負值
    Vo_val = (Uint16)(Vout_real * 100);			//輸出電壓不會有負值

    nRF_TX.txBuffer[0] = (Io_val >> 8);			//Iout_real/256
    nRF_TX.txBuffer[1] = (Io_val & 0xFF);		//使用遮罩

    nRF_TX.txBuffer[2] = (Vo_val >> 8);			//Vout_real/256
    nRF_TX.txBuffer[3] = (Vo_val & 0xFF);		//使用遮罩

    nRF24_SPI_WriteTxPayload(&nRF_TX);
    nRF24_ActivateTx(&nRF_TX);

#endif
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
