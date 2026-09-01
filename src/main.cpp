// GPS-DO
// Teensy 4.0 + u-blox LEA-5T (FW 6.02)

#include <Arduino.h>
#include <imxrt.h>
#include "gnss_ubx5.h"
#include "LiquidCrystal_I2C.h"

#include "DAC8550.h"
#include "Teensy_PWM.h"

// =====================================================================================================================
// Options / Configuration
// =====================================================================================================================
// #define PWM_OSC_TUNE
#define DAC_OSC_TUNE

#if (defined(PWM_OSC_TUNE) && defined(DAC_OSC_TUNE))
#error Only select PWM_OSC_TUNE or DAC_OSC_TUNE not both
#endif  // OSC TUNE SELECTION

static const bool     k_enableSurveyIn         = true;
static const uint32_t k_surveyInMinDur_s       = 600;
static const uint32_t k_surveyInVarLimit       = 900000;
static const uint32_t k_requiredGoodCyclesLock = 10;
static const uint16_t k_tenMhzAvgWindow_s      = 999;
static const uint16_t k_tenMhzAvgWindowMin     = 99;
static const uint32_t k_lcdCyclePeriod_s       = 3;

// =====================================================================================================================
// Global State Flags
// =====================================================================================================================
static bool     g_svinSubscribed    = false;
static bool     g_svinStartedByUs   = false;
static bool     g_tuningCycle       = false;
static uint32_t g_tLastPps_ms       = 0;
static bool     g_oscRxErr          = false;
static bool     g_ppsCaptured       = false;
static uint32_t g_tPpsCapture_ms    = 0;
static uint32_t g_latched10MhzCount = 0;
static bool     g_haveNewTimTp      = false;

// =====================================================================================================================
// Latest decoded NMEA/UBX message cache
// =====================================================================================================================
struct GpsMsgs
{
    NmeaGga  gga;
    uint32_t tLastGga_ms = 0;

    NmeaRmc  rmc;
    uint32_t tLastRmc_ms = 0;

    NmeaGsa  gsa;
    uint32_t tLastGsa_ms = 0;

    UbxTimTp timTp;
    uint32_t tLastTimTp_ms = 0;

    UbxNavTimeUtc timeUtc;
    uint32_t      tLastTimeUtc_ms = 0;
};

GpsMsgs g_gpsMsgs;

// =====================================================================================================================
// GSV epoch aggregator (collect one full GSV epoch)
// =====================================================================================================================
static SatInfo g_gsvSats[64];
static int     g_gsvCount    = 0;
static int     g_gsvInView   = -1;
static int     g_gsvTotal    = 0;
static bool    g_gsvComplete = false;

// =====================================================================================================================
// Pinout
// =====================================================================================================================
// 10 MHz is on PIN 19
// 1 PPS is on PIN 18

#define GNSS_SERIAL         Serial5
#define CONSOLE             Serial
#define LOCKED_LED_PIN      15
#define PPS_LED_PIN         14
#define OSC_TUNE_DAC_CS_PIN 10
// DAC CLK is on Pin   13
// DAC DAT is on Pin   11
#define OSC_TUNE_PWM_PIN 3

// =====================================================================================================================
// DAC / Analog config
// =====================================================================================================================
#ifdef DAC_OSC_TUNE
#define OSC_TUNE_VREF   3.3f
#define OSC_TUNE_COUNTS 65536U
static const float k_oscTuneZeroVal = OSC_TUNE_VREF / 2.0f;             // volts
static const float k_oscTuneLsb     = OSC_TUNE_VREF / OSC_TUNE_COUNTS;  // volts per count

DAC8550 g_oscTuneDac(OSC_TUNE_DAC_CS_PIN);
#endif  // DAC_OSC_TUNE

#ifdef PWM_OSC_TUNE
Teensy_PWM*        g_oscTunePwm;
#define OSC_TUNE_VREF   3.3f
#define OSC_TUNE_COUNTS 65536U
static const float k_clkTunePwmFreq = 1000.0f;
static const float k_oscTuneZeroVal = 0.0f;                             // volts
static const float k_oscTuneLsb     = OSC_TUNE_VREF / OSC_TUNE_COUNTS;  // volts per count
#endif  // PWM_OSC_TUNE

// =====================================================================================================================
// LCDisplay Config
// =====================================================================================================================
#define LCD_I2C_ADDR 0x27
#define LCD_COLS     20
#define LCD_ROWS     4
static LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// =====================================================================================================================
// Uptime
// =====================================================================================================================
struct Uptime
{
    uint32_t day;
    uint8_t  hr;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  tenms;
};

Uptime g_uptime;

// =====================================================================================================================
// Rolling average state for 10 MHz counter (1 Hz samples)
// =====================================================================================================================
struct TenMhzAvgState
{
    double      buf[k_tenMhzAvgWindow_s];
    uint16_t    n     = 0;
    uint16_t    idx   = 0;
    long double sum   = 0.0;  // sum of Hz (as long double for precision)
    long double sumsq = 0.0;  // sum of Hz^2
};

// =====================================================================================================================
// GPSDO operational state
// =====================================================================================================================
enum class GpsdoState : uint8_t
{
    WARMUP    = 0,  // Initializing / waiting for first valid PPS & GPS fix
    ACQUIRING = 1,  // Disciplining, coarse/fine pull-in towards 10 MHz
    LOCKED    = 2,  // Disciplined & locked within target ppb
    HOLDOVER  = 3,  // GPS PPS or fix lost after being locked; DAC frozen at vIntegral
    FAULT     = 4   // 10 MHz oscillator clock missing on Pin 19
};

// =====================================================================================================================
// GPSDO controller state (config, internals, metrics)
// =====================================================================================================================
struct GpsdoCtrl
{
    // Config
    double ocxoHzPerV       = 10.0;           // OCXO EFC sensitivity (Hz/V)
    double vMin             = 0.0;            // V
    double vMax             = OSC_TUNE_VREF;  // V
    double slewMaxVPerS     = 0.005;          // V/s (max change per update during acquisition)
    double slewMaxVPerSLock = 0.0002;         // V/s (gentle max change during lock)
    double f0_hz            = 10000000.000;   // expected frequency

    // Lock & Quality gates
    double qErrMax_ns      = 25.0;  // Maximum allowable GPS PPS qErr
    double lockEnterPpbReq = 1.5;   // enter lock when |avgErr_ppb| <= 1.5
    double lockExitPpbReq  = 3.0;   // exit lock when |avgErr_ppb| > 3.0

    // Multi-stage Adaptive Gains
    // Stage 1: Fast acquisition pull-in (Error > 0.05 Hz / 5 ppb)
    double KpFast = 0.08;
    double KiFast = 0.025;

    // Stage 2: Fine pull-in (0.015 < Error <= 0.05 Hz / 1.5 to 5 ppb)
    double KpMed = 0.04;
    double KiMed = 0.008;

    // Stage 3: Locked mode (Error <= 1.5 ppb & qualified by 99s rolling window)
    double KpLock     = 0.01;      // Gentler proportional damping in lock
    double KiLock     = 0.0001;    // Ultra-fine integral frequency term in lock
    double KphaseLock = 0.000002;  // Ultra-gentle absolute phase alignment in lock (<= 0.1 ppb)

    // Operational State & Holdover tracking
    GpsdoState state             = GpsdoState::WARMUP;
    bool       holdover          = false;
    uint32_t   holdoverSec       = 0;
    uint32_t   tHoldoverStart_ms = 0;
    bool       everLocked        = false;
    bool       ppsAlive          = false;
    bool       gnssFixValid      = false;
    uint32_t   tLastGoodFix_ms   = 0;

    // Internals
    uint32_t tenMhzCount_hz   = 0;    // 1 s gate → Hz
    double   dCyclesRaw       = 0.0;  // Raw cycle delta: (count - 10,000,000)
    double   dCycles          = 0.0;  // Corrected cycle delta (fractional cycle via differential qErr)
    double   phaseErr_s       = 0.0;  // seconds (single second phase error)
    double   phaseAccum_s     = 0.0;  // seconds (accumulated phase error relative to GPS UTC)
    double   vIntegral        = 2.0;  // V (integrator state)
    double   vProp            = 0.0;  // V (proportional term)
    double   target_v         = 2.0;  // V (current commanded output voltage)
    double   ditherAccum      = 0.0;  // Fractional sub-LSB dither accumulator
    uint8_t  goodCycles       = 0;
    bool     locked           = false;

    // Differential qErr tracking
    float lastQErr_ns  = NAN;
    bool  havePrevQErr = false;

    // Metrics
    double delta_hz = NAN;  // Hz
    double delta_v  = NAN;  // V
    bool   haveAvg  = false;
    bool   goodPps  = false;

    // 10 MHz frequency averaging (rolling window)
    TenMhzAvgState avgState;
    double         avg_hz     = NAN;  // Hz
    double         rms_hz     = NAN;  // Hz (sample stddev)
    double         avgErr_ppb = NAN;  // ppb
    uint16_t       avgNSamp   = 0;    // samples in window

    // Long-term baseline tracking
    uint64_t totalCycles     = 0;
    uint32_t totalPpsSeconds = 0;
};

GpsdoCtrl g_gpsDoCtrl;

// =====================================================================================================================
// Timer helpers: PPS-latched read
// =====================================================================================================================

/* _poll10MHzCount — Check for PPS capture; on flag, return 1 s count. */
static bool _poll10MHzCount(uint32_t* out_capt)
{
    static uint32_t countPrev = 0;

    IMXRT_TMR_t* TMRx = &IMXRT_TMR3;

    if (TMRx->CH[0].SCTRL & TMR_SCTRL_IEF)
    {
        uint32_t count = TMRx->CH[0].CAPT | (TMRx->CH[2].CAPT << 16);
        TMRx->CH[0].SCTRL &= ~TMR_SCTRL_IEF;
        TMRx->CH[2].SCTRL &= ~TMR_SCTRL_IEF;

        uint32_t countOutput = count - countPrev;
        countPrev            = count;

        if (out_capt)
        {
            *out_capt = countOutput;
        }
        return true;
    }
    return false;
}

// =====================================================================================================================
/* _counterInit — Configure QuadTimer channels for 10 MHz count and 1 PPS capture. */
// =====================================================================================================================
static void _counterInit(void)
{
    // Enable QTIMER3 clock
    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);

    // Pin mux:
    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_00 = 0b10001;  // QT3 TIMER0 on pin 19
    IOMUXC_QTIMER3_TIMER0_SELECT_INPUT  = 0b01;

    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_01 = 0b10001;  // QT3 TIMER1 on pin 18
    IOMUXC_QTIMER3_TIMER1_SELECT_INPUT  = 0b00;

    IMXRT_TMR_t* TMRx = &IMXRT_TMR3;

    // ---------------- CH0: 10 MHz low word ----------------
    TMRx->CH[0].CTRL   = 0;  // stop
    TMRx->CH[0].CNTR   = 0;
    TMRx->CH[0].LOAD   = 0;  // reload value after trigger
    TMRx->CH[0].COMP1  = 0xFFFF;
    TMRx->CH[0].CMPLD1 = 0xFFFF;
    TMRx->CH[0].SCTRL  = TMR_SCTRL_CAPTURE_MODE(1);
    TMRx->CH[0].FILT   = 0;
    TMRx->CH[0].CSCTRL = 0;

    // ---------------- CH2: 10 MHz high word ----------------
    TMRx->CH[2].CTRL   = 0;  // stop
    TMRx->CH[2].CNTR   = 0;
    TMRx->CH[2].LOAD   = 0;  // reload value after trigger
    TMRx->CH[2].COMP1  = 0;
    TMRx->CH[2].CMPLD1 = 0;
    TMRx->CH[2].SCTRL  = TMR_SCTRL_CAPTURE_MODE(1);
    TMRx->CH[2].FILT   = 0;
    TMRx->CH[2].CSCTRL = 0;

    // CH1: keep input path alive; used as secondary capture source (PPS)
    TMRx->CH[1].CTRL   = 0;  // stop
    TMRx->CH[1].CNTR   = 0;
    TMRx->CH[1].LOAD   = 0;  // reload value after trigger
    TMRx->CH[1].COMP1  = 0xFFFF;
    TMRx->CH[1].CMPLD1 = 0xFFFF;
    TMRx->CH[1].SCTRL  = 0;
    TMRx->CH[1].FILT   = 0;
    TMRx->CH[1].CSCTRL = 0;

    // Start CH0: count external pin (PCS=0), capture via CH1 (SCS=01)
    TMRx->CH[0].CTRL = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0b0000) | TMR_CTRL_SCS(0b01) | TMR_CTRL_LENGTH;

    // CH2: chain ripple from CH0 overflow (PCS=0100), capture with same SCS
    TMRx->CH[2].CTRL = TMR_CTRL_CM(7) | TMR_CTRL_PCS(0b0100) | TMR_CTRL_SCS(0b01);
}

// =====================================================================================================================
/* _tenMhzAvgReset — Clear rolling average state and exported metrics. */
// =====================================================================================================================
static inline void _tenMhzAvgReset()
{
    auto& s = g_gpsDoCtrl.avgState;
    s.n     = 0;
    s.idx   = 0;
    s.sum   = 0;
    s.sumsq = 0.0L;

    g_gpsDoCtrl.avg_hz     = NAN;
    g_gpsDoCtrl.rms_hz     = NAN;
    g_gpsDoCtrl.avgErr_ppb = NAN;
    g_gpsDoCtrl.avgNSamp   = 0;
}

// =====================================================================================================================
/* _tenMhzAvgReady — True when the rolling window is full and avg is finite. */
// =====================================================================================================================
static inline bool _tenMhzAvgReady()
{
    return (g_gpsDoCtrl.avgNSamp >= k_tenMhzAvgWindowMin) && isfinite(g_gpsDoCtrl.avgErr_ppb);
}

// =====================================================================================================================
/* _tenMhzAvgPush — Add a 1-second frequency sample; update mean/RMS/PPB metrics. */
// =====================================================================================================================
static inline void _tenMhzAvgPush(double count_hz)
{
    auto& s = g_gpsDoCtrl.avgState;

    if (s.n < k_tenMhzAvgWindow_s)
    {
        s.buf[s.idx] = count_hz;
        s.sum += (long double)count_hz;
        s.sumsq += (long double)count_hz * (long double)count_hz;
        s.idx = (s.idx + 1) % k_tenMhzAvgWindow_s;
        s.n++;
    }
    else
    {
        double old   = s.buf[s.idx];
        s.buf[s.idx] = count_hz;

        s.sum += (long double)count_hz - (long double)old;
        s.sumsq += (long double)count_hz * (long double)count_hz - (long double)old * (long double)old;

        s.idx = (s.idx + 1) % k_tenMhzAvgWindow_s;
    }

    g_gpsDoCtrl.avgNSamp = s.n;

    if (s.n > 0)
    {
        long double n      = (long double)s.n;
        long double mu     = s.sum / n;  // mean Hz
        g_gpsDoCtrl.avg_hz = (double)mu;

        if (s.n >= 2)
        {
            long double var = (s.sumsq - n * mu * mu) / (n - 1.0L);
            if (var < 0)
            {
                var = 0;
            }
            g_gpsDoCtrl.rms_hz = (double)sqrt((double)var);
        }
        else
        {
            g_gpsDoCtrl.rms_hz = NAN;
        }

        if (isfinite(g_gpsDoCtrl.avg_hz) && g_gpsDoCtrl.f0_hz != 0.0)
        {
            g_gpsDoCtrl.avgErr_ppb = (g_gpsDoCtrl.avg_hz - g_gpsDoCtrl.f0_hz) / g_gpsDoCtrl.f0_hz * 1e9;
        }
        else
        {
            g_gpsDoCtrl.avgErr_ppb = NAN;
        }
    }
    else
    {
        g_gpsDoCtrl.avg_hz = g_gpsDoCtrl.rms_hz = g_gpsDoCtrl.avgErr_ppb = NAN;
    }
}

// =====================================================================================================================
// GNSS helpers
// =====================================================================================================================
static bool _gnssMsgCurrent(uint32_t tMsg_ms)
{
    return (millis() - tMsg_ms < 1500);
}

/* _gsvReset — Reset GSV epoch aggregation state. */
static void _gsvReset()
{
    g_gsvCount    = 0;
    g_gsvInView   = -1;
    g_gsvTotal    = 0;
    g_gsvComplete = false;
}

/* _gsvIngest — Accumulate a GSV sentence into the current epoch aggregation. */
static void _gsvIngest(const NmeaGsv& part)
{
    if (part.msgNo == 1)
    {
        _gsvReset();
    }
    g_gsvTotal = part.totalMsg;
    if (part.inView >= 0)
    {
        g_gsvInView = part.inView;
    }
    for (int i = 0; i < part.satsCount && g_gsvCount < 64; ++i)
    {
        g_gsvSats[g_gsvCount++] = part.sats[i];
    }
    if (part.msgNo >= g_gsvTotal || (g_gsvInView > 0 && g_gsvCount >= g_gsvInView))
    {
        g_gsvComplete = true;
    }
}

// =====================================================================================================================
// OSC Tuning Helpers
// =====================================================================================================================

/* _getTuneValDiscrete — Convert desired voltage to DAC code (counts). */
static int _getTuneValDiscrete(float desired_v)
{
    return (desired_v - k_oscTuneZeroVal) / k_oscTuneLsb;
}

static void _setOscTuneVoltage(float desired_v)
{
#ifdef DAC_OSC_TUNE
    g_oscTuneDac.setValue(_getTuneValDiscrete(desired_v));
#endif  // DAC_OSC_TUNE

#ifdef PWM_OSC_TUNE
    g_oscTunePwm->setPWM_Int(OSC_TUNE_PWM_PIN, k_clkTunePwmFreq, _getTuneValDiscrete(desired_v));
#endif  // PWM_OSC_TUNE
}

// =====================================================================================================================
// Setup / Initialization
// =====================================================================================================================

void setup()
{
    CONSOLE.begin(115200);
    while (!CONSOLE && millis() < 2000)
    {
        // wait for USB
    }

    CONSOLE.printf("Teensy GPSDO v2 (Optimized Control Loop)\r\n");

    pinMode(PPS_LED_PIN, OUTPUT);
    digitalWrite(PPS_LED_PIN, LOW);

    pinMode(LOCKED_LED_PIN, OUTPUT);
    analogWrite(LOCKED_LED_PIN, 0);

    lcd.init();
    lcd.backlight();
    lcd.print("Initializing...");

    // Initialize GNSS serial at default 9600 baud
    GNSS_SERIAL.begin(9600);
    gnssInit(&GNSS_SERIAL, NULL);
    delay(50);

    // Switch u-blox UART1 to 115200 baud to eliminate serial transmission latency
    gnssSetBaudRate(1, 115200);
    delay(100);
    GNSS_SERIAL.begin(115200);
    delay(50);

    // Enable NMEA sentences we decode
    gnssSendPubx40("GGA", true);
    delay(50);
    gnssSendPubx40("RMC", true);
    delay(50);
    gnssSendPubx40("GSA", true);
    delay(50);
    gnssSendPubx40("GSV", true);
    delay(50);

    // Ensure UBX TIM-TP on UART1 (rate=1)
    gnssSendUbxCfgMsg(0x0D, 0x01, /*UART1*/ 1, /*rate*/ 1);
    delay(50);

    // Enable UBX-NAV-TIMEUTC (0x01,0x21) on UART1
    gnssSendUbxCfgMsg(0x01, 0x21, /*UART1*/ 1, /*rate*/ 1);
    delay(50);

    _gsvReset();
    gnssPollCfgTmode();

#ifdef DAC_OSC_TUNE
    CONSOLE.printf("Initializing 16-bit DAC (DAC8550)...\r\n");
    SPI.begin();
    g_oscTuneDac.begin();
    g_gpsDoCtrl.vIntegral = g_gpsDoCtrl.target_v;
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
    CONSOLE.printf("DAC Initialized to nominal %.3f V\r\n", g_gpsDoCtrl.target_v);
#endif  // DAC_OSC_TUNE

#ifdef PWM_OSC_TUNE
    g_oscTunePwm = new Teensy_PWM(OSC_TUNE_PWM_PIN, k_clkTunePwmFreq, 50.0);
    g_gpsDoCtrl.vIntegral = g_gpsDoCtrl.target_v;
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
#endif  // PWM_OSC_TUNE

    CONSOLE.printf("Beginning QuadTimer Pulse Counting...\r\n");
    _counterInit();
}

// =====================================================================================================================
// GNSS / Console / PPS / Control Handlers
// =====================================================================================================================

/* _handleGnss — Read and decode GNSS streams (NMEA/UBX); update message cache. */
static void _handleGnss(uint32_t tNow_ms)
{
    // Feed stream
    gnssReadSerial();

    // Pop & parse all NMEA that arrived
    char line[160];
    while (gnssPopLastRawNmea(line, sizeof(line)))
    {
        NmeaType t;
        if (!gnssClassifyNmea(line, t))
        {
            continue;
        }

        if (t == NmeaType::GGA)
        {
            NmeaGga gga;
            if (gnssParseGga(line, gga))
            {
                g_gpsMsgs.gga         = gga;
                g_gpsMsgs.tLastGga_ms = tNow_ms;
            }
        }
        else if (t == NmeaType::RMC)
        {
            NmeaRmc rmc;
            if (gnssParseRmc(line, rmc))
            {
                g_gpsMsgs.rmc         = rmc;
                g_gpsMsgs.tLastRmc_ms = tNow_ms;
            }
        }
        else if (t == NmeaType::GSA)
        {
            NmeaGsa gsa;
            if (gnssParseGsa(line, gsa))
            {
                g_gpsMsgs.gsa         = gsa;
                g_gpsMsgs.tLastGsa_ms = tNow_ms;
            }
        }
        else if (t == NmeaType::GSV)
        {
            NmeaGsv gsv;
            if (gnssParseGsv(line, gsv))
            {
                _gsvIngest(gsv);
            }
        }
    }

    // Pop any UBX frames; pick out TIM-TP (qErr) if present
    UbxFrame fr;
    while (gnssPopLastUbx(fr))
    {
        UbxType t;
        if (!gnssClassifyUbx(fr, t))
        {
            continue;
        }

        switch (t)
        {
            case UbxType::CFG_TMODE:
            {
                UbxCfgTmode tm;
                if (gnssDecodeCfgTmode(fr, tm))
                {
                    const char* modeStr = "Unknown";
                    if (tm.timeMode == 0)
                    {
                        modeStr = "Disabled";
                    }
                    else if (tm.timeMode == 1)
                    {
                        modeStr = "Survey-In";
                    }
                    else if (tm.timeMode == 2)
                    {
                        modeStr = "Fixed";
                    }

                    CONSOLE.printf("[TMODE] %s\r\n", modeStr);

                    if (tm.timeMode == 2)
                    {
                        gnssSendUbxCfgMsg(0x0D, 0x04, /*UART1*/ 1, /*rate*/ 0);  // disable TIM-SVIN
                        g_svinSubscribed  = false;
                        g_svinStartedByUs = false;
                        CONSOLE.println("[SVIN] Unsubscribed TIM-SVIN (Fixed confirmed).");
                    }
                    else
                    {
                        if (k_enableSurveyIn && !g_svinSubscribed)
                        {
                            gnssEnableSurveyIn(k_surveyInMinDur_s, k_surveyInVarLimit);
                            g_svinStartedByUs = true;
                            gnssSendUbxCfgMsg(0x0D, 0x04, /*UART1*/ 1, /*rate*/ 1);  // TIM-SVIN
                            CONSOLE.printf("[SVIN] Started Survey-In MinDur %u, VarLimit %u\r\n",
                                           (unsigned)k_surveyInMinDur_s,
                                           (unsigned)k_surveyInVarLimit);
                        }
                    }
                }
                break;
            }

            case UbxType::TIM_SVIN:
            {
                UbxTimSvin sv;
                if (gnssDecodeTimSvin(fr, sv))
                {
                    CONSOLE.printf("[SVIN] t:%lu s  obs:%lu  var:%lu  active:%c  valid:%c\r\n",
                                   (unsigned long)sv.durSec,
                                   (unsigned long)sv.obs,
                                   (unsigned long)sv.meanV_mm2,
                                   sv.active ? 'Y' : 'N',
                                   sv.valid ? 'Y' : 'N');

                    if (g_svinStartedByUs && !sv.active && sv.valid)
                    {
                        CONSOLE.println("[SVIN] COMPLETE → polling TMODE to confirm Fixed...");
                        gnssPollCfgTmode();
                    }
                }
                break;
            }

            case UbxType::TIM_TP:
            {
                if (gnssDecodeTimTp(fr, g_gpsMsgs.timTp))
                {
                    g_gpsMsgs.tLastTimTp_ms = tNow_ms;
                    g_haveNewTimTp          = true;
                }
                break;
            }

            case UbxType::NAV_TIMEUTC:
            {
                if (gnssDecodeNavTimeUtc(fr, g_gpsMsgs.timeUtc))
                {
                    g_gpsMsgs.tLastTimeUtc_ms = tNow_ms;
                }
                break;
            }
            default:
            {
                break;
            }
        }
    }

    // Evaluate GNSS fix validity
    bool ggaValid = _gnssMsgCurrent(g_gpsMsgs.tLastGga_ms) && (g_gpsMsgs.gga.fixQ > 0);
    bool rmcValid = _gnssMsgCurrent(g_gpsMsgs.tLastRmc_ms) && g_gpsMsgs.rmc.valid;
    g_gpsDoCtrl.gnssFixValid = ggaValid || rmcValid;
    if (g_gpsDoCtrl.gnssFixValid)
    {
        g_gpsDoCtrl.tLastGoodFix_ms = tNow_ms;
    }
}

// =====================================================================================================================
// Print helpers
// =====================================================================================================================

/* _printUtcFromHhmmss — Print hhmmss.sss format time as HH:MM:SS.sss. */
static void _printUtcFromHhmmss(double hhmmss)
{
    if (!(hhmmss > 0.0))
    {
        CONSOLE.print("--:--:--");
        return;
    }
    int    hh = (int)(hhmmss / 10000.0);
    int    mm = (int)((hhmmss - hh * 10000.0) / 100.0);
    double ss = hhmmss - hh * 10000.0 - mm * 100.0;
    CONSOLE.printf("%02d:%02d:%06.3f", hh, mm, ss);
}

static inline void _printUptimeStamp()
{
    CONSOLE.printf("<%lu:%02u:%02u:%02u> ",
                   (unsigned long)g_uptime.day,
                   g_uptime.hr,
                   g_uptime.min,
                   g_uptime.sec);
}

/* _handleConsole — Once per second, print grouped status summary. */
static void _handleConsole(uint32_t tNow_ms)
{
    static unsigned long lastPrint_ms = 0;
    if (tNow_ms - lastPrint_ms < 1000)
    {
        return;
    }
    lastPrint_ms = tNow_ms;

    bool haveGga     = _gnssMsgCurrent(g_gpsMsgs.tLastGga_ms);
    bool haveRmc     = _gnssMsgCurrent(g_gpsMsgs.tLastRmc_ms);
    bool haveGsa     = _gnssMsgCurrent(g_gpsMsgs.tLastGsa_ms);
    bool haveTimTp   = _gnssMsgCurrent(g_gpsMsgs.tLastTimTp_ms);
    bool haveTimeUtc = _gnssMsgCurrent(g_gpsMsgs.tLastTimeUtc_ms);

    // UTC
    double utcHhmmss = 0.0;
    bool   haveUtc   = false;
    if (haveGga && (g_gpsMsgs.gga.utcHhmmss > 0.0))
    {
        utcHhmmss = g_gpsMsgs.gga.utcHhmmss;
        haveUtc   = true;
    }
    else if (haveRmc && (g_gpsMsgs.rmc.utcHhmmss > 0.0))
    {
        utcHhmmss = g_gpsMsgs.rmc.utcHhmmss;
        haveUtc   = true;
    }

    // Sats used
    int satsUsed = -1;
    if (haveGga && g_gpsMsgs.gga.sats >= 0)
    {
        satsUsed = g_gpsMsgs.gga.sats;
    }
    else if (haveGsa && g_gpsMsgs.gsa.used > 0)
    {
        satsUsed = g_gpsMsgs.gsa.used;
    }

    // DOPs
    double pdop = (haveGsa && !isnan(g_gpsMsgs.gsa.pdop)) ? g_gpsMsgs.gsa.pdop : NAN;
    double hdop = NAN;
    if (haveGsa && !isnan(g_gpsMsgs.gsa.hdop))
    {
        hdop = g_gpsMsgs.gsa.hdop;
    }
    else if (haveGga && !isnan(g_gpsMsgs.gga.hdop))
    {
        hdop = g_gpsMsgs.gga.hdop;
    }
    double vdop = (haveGsa && !isnan(g_gpsMsgs.gsa.vdop)) ? g_gpsMsgs.gsa.vdop : NAN;

    // Position / Altitude
    double latDeg = NAN, lonDeg = NAN, altM = NAN;
    if (haveGga && !isnan(g_gpsMsgs.gga.latDeg) && !isnan(g_gpsMsgs.gga.lonDeg))
    {
        latDeg = g_gpsMsgs.gga.latDeg;
        lonDeg = g_gpsMsgs.gga.lonDeg;
        if (!isnan(g_gpsMsgs.gga.altM))
        {
            altM = g_gpsMsgs.gga.altM;
        }
    }
    else if (haveRmc && !isnan(g_gpsMsgs.rmc.latDeg) && !isnan(g_gpsMsgs.rmc.lonDeg))
    {
        latDeg = g_gpsMsgs.rmc.latDeg;
        lonDeg = g_gpsMsgs.rmc.lonDeg;
    }

    int      inView     = (g_gsvInView >= 0) ? g_gsvInView : 0;
    float    qErr_ns    = haveTimTp ? g_gpsMsgs.timTp.qErrNs : NAN;
    uint32_t tAcc_ns    = haveTimeUtc ? g_gpsMsgs.timeUtc.tAccNs : 0;
    char     goodPps_c  = g_gpsDoCtrl.goodPps ? 'Y' : 'N';
    char     avgReady_c = _tenMhzAvgReady() ? 'Y' : 'N';

    const char* stateStr = "WARMUP";
    if (g_gpsDoCtrl.state == GpsdoState::LOCKED)
    {
        stateStr = "LOCKED";
    }
    else if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
    {
        stateStr = "HOLDOVER";
    }
    else if (g_gpsDoCtrl.state == GpsdoState::ACQUIRING)
    {
        stateStr = "ACQUIRING";
    }
    else if (g_gpsDoCtrl.state == GpsdoState::FAULT)
    {
        stateStr = "FAULT";
    }

    // [TIME]
    _printUptimeStamp();
    CONSOLE.print("[TIME] UTC:");
    if (haveUtc)
    {
        _printUtcFromHhmmss(utcHhmmss);
    }
    else
    {
        CONSOLE.print("--:--:--");
    }
    CONSOLE.printf(" | qErr=%.1f ns | tAcc=%lu ns | goodPps=%c | avgReady=%c | ppsAlive=%c | state=%s\r\n",
                   haveTimTp ? qErr_ns : 0.0f,
                   (unsigned long)tAcc_ns,
                   goodPps_c,
                   avgReady_c,
                   g_gpsDoCtrl.ppsAlive ? 'Y' : 'N',
                   stateStr);

    // [GPS]
    _printUptimeStamp();
    CONSOLE.printf("[GPS] fixQ=%d | fixType=%d | sats=%d | inView=%d | PDOP=%.2f | HDOP=%.2f | VDOP=%.2f\r\n",
                   haveGga ? g_gpsMsgs.gga.fixQ : 0,
                   haveGsa ? g_gpsMsgs.gsa.fixType : 1,
                   satsUsed,
                   inView,
                   pdop,
                   hdop,
                   vdop);
    if (!isnan(latDeg) && !isnan(lonDeg))
    {
        _printUptimeStamp();
        CONSOLE.printf("[GPS] lat=%.7f | lon=%.7f | alt=%.2f m\r\n",
                       latDeg,
                       lonDeg,
                       isnan(altM) ? 0.0 : altM);
    }

    // [OSC] Line 1: Frequency, Cycle Errors, Phase
    _printUptimeStamp();
    CONSOLE.printf(
        "[OSC] f_hz=%lu | dCyc_raw=%.2f | dCyc_corr=%.3f | phErr_ns=%.2f | phAccum_us=%.3f\r\n",
        (unsigned long)g_gpsDoCtrl.tenMhzCount_hz,
        g_gpsDoCtrl.dCyclesRaw,
        g_gpsDoCtrl.dCycles,
        g_gpsDoCtrl.phaseErr_s * 1e9,
        g_gpsDoCtrl.phaseAccum_s * 1e6);

    // [OSC] Line 2: Control Voltages and Lock
    _printUptimeStamp();
    if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
    {
        CONSOLE.printf(
            "[OSC] V_target=%.6f V | V_int=%.6f V (FROZEN) | holdover=%lu s | state=HOLDOVER\r\n",
            g_gpsDoCtrl.target_v,
            g_gpsDoCtrl.vIntegral,
            (unsigned long)g_gpsDoCtrl.holdoverSec);
    }
    else
    {
        CONSOLE.printf(
            "[OSC] V_target=%.6f V | V_int=%.6f V | V_prop=%.6f V | lock=%d | fA=%.4f Hz | err=%.3f ppb | rms=%.3f Hz\r\n",
            g_gpsDoCtrl.target_v,
            g_gpsDoCtrl.vIntegral,
            g_gpsDoCtrl.vProp,
            g_gpsDoCtrl.locked ? 1 : 0,
            g_gpsDoCtrl.avg_hz,
            g_gpsDoCtrl.avgErr_ppb,
            g_gpsDoCtrl.rms_hz);
    }
}

// =====================================================================================================================
/* _handlePps — Handle 1 PPS capture from hardware counter */
// =====================================================================================================================
static void _handlePps(uint32_t tNow_ms)
{
    uint32_t tenMhzCount = 0;

    if (_poll10MHzCount(&tenMhzCount))
    {
        g_tLastPps_ms       = tNow_ms;
        g_tPpsCapture_ms    = tNow_ms;
        g_latched10MhzCount = tenMhzCount;
        g_ppsCaptured       = true;

        if (tenMhzCount == 0)
        {
            CONSOLE.println("Error: Clock not detected on Pin 19!");
            g_gpsDoCtrl.tenMhzCount_hz = (uint32_t)-1;
            g_oscRxErr                 = true;
        }
        else if (tenMhzCount > 10000500 || tenMhzCount < 9999500)
        {
            CONSOLE.printf("Warning: 10MHz Clock count out of range: %lu\r\n", (unsigned long)tenMhzCount);
            g_gpsDoCtrl.tenMhzCount_hz = 0;
            g_oscRxErr                 = true;
        }
        else
        {
            g_gpsDoCtrl.tenMhzCount_hz = tenMhzCount;
            g_oscRxErr                 = false;
        }
    }

    g_gpsDoCtrl.ppsAlive = ((tNow_ms - g_tLastPps_ms) <= 1500) && (g_tLastPps_ms != 0);
}

// =====================================================================================================================
/* _handleOscTuning — Synchronized PI Disciplining Loop with Differential qErr Compensation */
// =====================================================================================================================
static void _handleOscTuning(uint32_t tNow_ms)
{
    // If in holdover mode, maintain frozen DAC voltage without any disturbance
    if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
    {
        g_gpsDoCtrl.target_v = g_gpsDoCtrl.vIntegral;
        g_gpsDoCtrl.vProp    = 0.0;
        _setOscTuneVoltage(g_gpsDoCtrl.target_v);
        return;
    }

    // Execute tuning only when PPS is captured and we have the matching TIM-TP (or a 250ms timeout)
    if (!g_ppsCaptured || g_oscRxErr)
    {
        return;
    }

    // If TIM-TP hasn't arrived yet and < 250 ms since PPS, wait a short moment for UART
    if (!g_haveNewTimTp && (tNow_ms - g_tPpsCapture_ms < 250))
    {
        return;
    }

    // Consume PPS event
    g_ppsCaptured  = false;
    g_haveNewTimTp = false;
    g_tuningCycle  = true;

    // ---- Differential GPS Quantization Jitter Compensation ----
    const bool  haveQerr = _gnssMsgCurrent(g_gpsMsgs.tLastTimTp_ms);
    const float qErr_ns  = haveQerr ? g_gpsMsgs.timTp.qErrNs : 0.0f;

    double deltaQErr_ns = 0.0;
    if (haveQerr && g_gpsDoCtrl.havePrevQErr)
    {
        deltaQErr_ns = (double)(qErr_ns - g_gpsDoCtrl.lastQErr_ns);
    }

    if (haveQerr)
    {
        g_gpsDoCtrl.lastQErr_ns  = qErr_ns;
        g_gpsDoCtrl.havePrevQErr = true;
    }
    else
    {
        g_gpsDoCtrl.havePrevQErr = false;
    }

    // 1 cycle of 10 MHz = 100 ns -> 0.01 cycles / ns
    const double fracCycles   = deltaQErr_ns * 0.01;
    const int64_t dCycles_raw = (int64_t)g_gpsDoCtrl.tenMhzCount_hz - (int64_t)g_gpsDoCtrl.f0_hz;

    g_gpsDoCtrl.dCyclesRaw = (double)dCycles_raw;
    g_gpsDoCtrl.dCycles    = (double)dCycles_raw - fracCycles;

    // Good PPS gate (requires valid qErr AND valid active GNSS fix)
    g_gpsDoCtrl.goodPps = haveQerr && g_gpsDoCtrl.gnssFixValid && (fabs((double)qErr_ns) <= g_gpsDoCtrl.qErrMax_ns);

    if (g_gpsDoCtrl.goodPps)
    {
        const double est_hz = g_gpsDoCtrl.f0_hz + g_gpsDoCtrl.dCycles;
        _tenMhzAvgPush(est_hz);
    }

    // Phase error for this second (seconds)
    g_gpsDoCtrl.phaseErr_s = g_gpsDoCtrl.dCycles / g_gpsDoCtrl.f0_hz;

    // Accumulate phase error only while locked to avoid historical cold-start drift
    if (g_gpsDoCtrl.locked)
    {
        g_gpsDoCtrl.phaseAccum_s += g_gpsDoCtrl.phaseErr_s;
    }
    else
    {
        g_gpsDoCtrl.phaseAccum_s = 0.0;
    }

    // Clamp phase accumulator to ±2.0 µs (±20 cycles) to prevent large steering offsets
    const double phaseMax_s = 2.0e-6;
    if (g_gpsDoCtrl.phaseAccum_s > phaseMax_s)
    {
        g_gpsDoCtrl.phaseAccum_s = phaseMax_s;
    }
    if (g_gpsDoCtrl.phaseAccum_s < -phaseMax_s)
    {
        g_gpsDoCtrl.phaseAccum_s = -phaseMax_s;
    }

    g_gpsDoCtrl.haveAvg = _tenMhzAvgReady();

    // If PPS or fix is not qualified, do not corrupt the integral state
    if (!g_gpsDoCtrl.goodPps)
    {
        g_gpsDoCtrl.vProp = 0.0;
        _setOscTuneVoltage(g_gpsDoCtrl.target_v);
        return;
    }

    // ---- Multi-Stage Adaptive Gains & Slew Limits ----
    double Kp_use;
    double Ki_use;
    double maxSlew;

    const double absCycleErr = fabs(g_gpsDoCtrl.dCycles);

    if (g_gpsDoCtrl.locked)
    {
        // Stage 3: Locked Mode (Ultra-low noise, high time-constant filter)
        Kp_use  = g_gpsDoCtrl.KpLock;
        Ki_use  = g_gpsDoCtrl.KiLock;
        maxSlew = g_gpsDoCtrl.slewMaxVPerSLock;
    }
    else if (absCycleErr > 0.05)
    {
        // Stage 1: Fast Acquisition Pull-In (Error > 5 ppb / 0.05 Hz)
        Kp_use  = g_gpsDoCtrl.KpFast;
        Ki_use  = g_gpsDoCtrl.KiFast;
        maxSlew = g_gpsDoCtrl.slewMaxVPerS;
    }
    else
    {
        // Stage 2: Fine Pull-In (Error 1.5 to 5 ppb)
        Kp_use  = g_gpsDoCtrl.KpMed;
        Ki_use  = g_gpsDoCtrl.KiMed;
        maxSlew = 0.001;  // V/s
    }

    // 1) Integral Term (Volts): accumulates physical crystal bias with zero steady-state error
    if (g_gpsDoCtrl.ocxoHzPerV > 0.0)
    {
        double dV_int = -(Ki_use * g_gpsDoCtrl.dCycles) / g_gpsDoCtrl.ocxoHzPerV;

        if (g_gpsDoCtrl.locked)
        {
            // In lock, gently steer absolute accumulated phase back to 0
            double dV_phase = -(g_gpsDoCtrl.KphaseLock * g_gpsDoCtrl.phaseAccum_s * g_gpsDoCtrl.f0_hz) / g_gpsDoCtrl.ocxoHzPerV;
            dV_int += dV_phase;
        }

        g_gpsDoCtrl.vIntegral += dV_int;

        // Anti-windup clamping
        if (g_gpsDoCtrl.vIntegral < g_gpsDoCtrl.vMin)
        {
            g_gpsDoCtrl.vIntegral = g_gpsDoCtrl.vMin;
        }
        if (g_gpsDoCtrl.vIntegral > g_gpsDoCtrl.vMax)
        {
            g_gpsDoCtrl.vIntegral = g_gpsDoCtrl.vMax;
        }
    }

    // 2) Proportional Term (Volts): immediate frequency damping
    if (g_gpsDoCtrl.ocxoHzPerV > 0.0)
    {
        g_gpsDoCtrl.vProp = -(Kp_use * g_gpsDoCtrl.dCycles) / g_gpsDoCtrl.ocxoHzPerV;
    }
    else
    {
        g_gpsDoCtrl.vProp = 0.0;
    }

    // Total commanded voltage = Integral + Proportional
    double commanded_v = g_gpsDoCtrl.vIntegral + g_gpsDoCtrl.vProp;

    // Slew rate limiting relative to previous target
    double deltaV  = commanded_v - g_gpsDoCtrl.target_v;
    if (deltaV > maxSlew)
    {
        deltaV = maxSlew;
    }
    if (deltaV < -maxSlew)
    {
        deltaV = -maxSlew;
    }

    g_gpsDoCtrl.target_v += deltaV;
    if (g_gpsDoCtrl.target_v < g_gpsDoCtrl.vMin)
    {
        g_gpsDoCtrl.target_v = g_gpsDoCtrl.vMin;
    }
    if (g_gpsDoCtrl.target_v > g_gpsDoCtrl.vMax)
    {
        g_gpsDoCtrl.target_v = g_gpsDoCtrl.vMax;
    }

    // Apply to DAC with software delta-sigma dithering
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
}

// =====================================================================================================================
/* _handleLockLogic — Lock qualification & Holdover state machine */
// =====================================================================================================================
static void _handleLockLogic(uint32_t tNow_ms)
{
    // Fault condition: 10 MHz reference missing on Pin 19
    if (g_oscRxErr)
    {
        g_gpsDoCtrl.state      = GpsdoState::FAULT;
        g_gpsDoCtrl.locked     = false;
        g_gpsDoCtrl.holdover   = false;
        g_gpsDoCtrl.goodCycles = 0;
        return;
    }

    const bool ppsAlive   = g_gpsDoCtrl.ppsAlive;
    const bool fixValid   = g_gpsDoCtrl.gnssFixValid;
    const bool signalLost = !ppsAlive || !fixValid;

    // Handle Loss of PPS or GPS Fix
    if (signalLost)
    {
        if (g_gpsDoCtrl.everLocked)
        {
            if (g_gpsDoCtrl.state != GpsdoState::HOLDOVER)
            {
                // Transition into Holdover mode
                g_gpsDoCtrl.state             = GpsdoState::HOLDOVER;
                g_gpsDoCtrl.holdover          = true;
                g_gpsDoCtrl.locked            = false;
                g_gpsDoCtrl.tHoldoverStart_ms = tNow_ms;
                g_gpsDoCtrl.holdoverSec       = 0;
                g_gpsDoCtrl.goodCycles        = 0;

                // Freeze DAC at pure steady-state integrator voltage
                g_gpsDoCtrl.target_v = g_gpsDoCtrl.vIntegral;
                g_gpsDoCtrl.vProp    = 0.0;
                _setOscTuneVoltage(g_gpsDoCtrl.target_v);

                CONSOLE.printf("[HOLDOVER] Lost %s -> Entering Holdover. DAC frozen at %.6f V\r\n",
                               !ppsAlive ? "1PPS Signal" : "GNSS Fix",
                               g_gpsDoCtrl.vIntegral);
            }
            else
            {
                // Already in Holdover: update elapsed holdover time
                g_gpsDoCtrl.holdoverSec = (tNow_ms - g_gpsDoCtrl.tHoldoverStart_ms) / 1000UL;
            }
        }
        else
        {
            // Lost signal before ever locking
            g_gpsDoCtrl.state      = GpsdoState::WARMUP;
            g_gpsDoCtrl.locked     = false;
            g_gpsDoCtrl.holdover   = false;
            g_gpsDoCtrl.goodCycles = 0;
        }
        return;
    }

    // If recovering from holdover: clear holdover and prepare to re-acquire
    if (g_gpsDoCtrl.holdover)
    {
        g_gpsDoCtrl.holdover     = false;
        g_gpsDoCtrl.state        = GpsdoState::ACQUIRING;
        g_gpsDoCtrl.goodCycles   = 0;
        g_gpsDoCtrl.phaseAccum_s = 0.0;
        g_gpsDoCtrl.havePrevQErr = false;
        CONSOLE.println("[RECOVERY] Signal Restored -> Resuming disciplining...");
    }

    // Evaluate Lock criteria only on fresh 1-second capture cycles
    if (!g_tuningCycle)
    {
        return;
    }

    const bool   avgReady = _tenMhzAvgReady();
    const bool   goodPps  = g_gpsDoCtrl.goodPps;
    const double absPpb   = fabs(g_gpsDoCtrl.avgErr_ppb);

    const bool freqEnter = avgReady && (absPpb <= g_gpsDoCtrl.lockEnterPpbReq);
    const bool freqHold  = avgReady && (absPpb <= g_gpsDoCtrl.lockExitPpbReq);

    const bool inBandForEnter   = freqEnter && goodPps;
    const bool outOfBandForExit = !freqHold || !goodPps;

    if (inBandForEnter)
    {
        if (g_gpsDoCtrl.goodCycles < k_requiredGoodCyclesLock)
        {
            g_gpsDoCtrl.goodCycles++;
        }
    }
    else if (outOfBandForExit)
    {
        g_gpsDoCtrl.goodCycles = 0;
    }

    bool wasLocked     = g_gpsDoCtrl.locked;
    bool nowLocked     = (avgReady && (g_gpsDoCtrl.goodCycles >= (k_requiredGoodCyclesLock - 1)));
    g_gpsDoCtrl.locked = nowLocked;

    if (nowLocked)
    {
        g_gpsDoCtrl.state       = GpsdoState::LOCKED;
        g_gpsDoCtrl.everLocked  = true;
        g_gpsDoCtrl.holdoverSec = 0;

        // Reset phase accumulator to 0 upon entering lock so historical startup drift is not steered
        if (!wasLocked)
        {
            g_gpsDoCtrl.phaseAccum_s = 0.0;
            CONSOLE.println("[LOCK] Entered Lock -> Phase accumulator zeroed for tracking.");
        }
    }
    else
    {
        g_gpsDoCtrl.state = GpsdoState::ACQUIRING;
    }
}

// =====================================================================================================================
/* _handleLeds — Blink PPS LED and show lock/holdover status on LOCK LED. */
// =====================================================================================================================
static void _handleLeds(uint32_t tNow_ms)
{
    static uint32_t lastBlink_ms = 0;
    static bool     blinkState   = false;

    if (tNow_ms - lastBlink_ms >= 500)
    {
        lastBlink_ms = tNow_ms;
        blinkState   = !blinkState;
    }

    // LOCK LED behavior
    if (g_gpsDoCtrl.state == GpsdoState::LOCKED)
    {
        analogWrite(LOCKED_LED_PIN, 5);  // Solid glow when locked
    }
    else if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
    {
        // 1 Hz blink in Holdover mode
        analogWrite(LOCKED_LED_PIN, blinkState ? 5 : 0);
    }
    else
    {
        analogWrite(LOCKED_LED_PIN, 0);  // Off when unlocked / warmup / fault
    }

    // PPS LED: pulse 25ms on actual PPS events
    static bool     ppsLedState = false;
    static uint32_t ppsLedOn_ms = 0;

    if (g_tuningCycle && !ppsLedState)
    {
        digitalWrite(PPS_LED_PIN, HIGH);
        ppsLedState = true;
        ppsLedOn_ms = tNow_ms;
    }
    else if (ppsLedState && (tNow_ms - ppsLedOn_ms >= 25))
    {
        digitalWrite(PPS_LED_PIN, LOW);
        ppsLedState = false;
        ppsLedOn_ms = 0;
    }
}

// =====================================================================================================================
// _handleLCD — 20x4 LCD Status Display (1 Hz refresh)
// =====================================================================================================================

static inline void _lcdPrintRow(uint8_t row, const char* text)
{
    char line[LCD_COLS + 1];
    size_t i = 0;
    for (; i < LCD_COLS && text[i] != '\0'; ++i)
    {
        line[i] = text[i];
    }
    while (i < LCD_COLS)
    {
        line[i++] = ' ';
    }
    line[LCD_COLS] = '\0';
    lcd.setCursor(0, row);
    lcd.print(line);
}

static inline String _utcStrCompact()
{
    double hhmmss = 0.0;
    if (_gnssMsgCurrent(g_gpsMsgs.tLastGga_ms) && (g_gpsMsgs.gga.utcHhmmss > 0.0))
    {
        hhmmss = g_gpsMsgs.gga.utcHhmmss;
    }
    else if (_gnssMsgCurrent(g_gpsMsgs.tLastRmc_ms) && (g_gpsMsgs.rmc.utcHhmmss > 0.0))
    {
        hhmmss = g_gpsMsgs.rmc.utcHhmmss;
    }
    if (!(hhmmss > 0.0))
    {
        return String("--:--:--");
    }
    int    hh = (int)(hhmmss / 10000.0);
    int    mm = (int)((hhmmss - hh * 10000.0) / 100.0);
    double ss = hhmmss - hh * 10000.0 - mm * 100.0;

    char tbuf[16];
    snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02.0f", hh, mm, ss);
    return String(tbuf);
}

static inline uint8_t _satsUsed()
{
    if (_gnssMsgCurrent(g_gpsMsgs.tLastGga_ms) && g_gpsMsgs.gga.sats >= 0)
    {
        return (uint8_t)g_gpsMsgs.gga.sats;
    }
    if (_gnssMsgCurrent(g_gpsMsgs.tLastGsa_ms) && g_gpsMsgs.gsa.used > 0)
    {
        return (uint8_t)g_gpsMsgs.gsa.used;
    }
    return 0;
}

static inline void _fmtLatLonLines(char* line2, size_t sz2, char* line3, size_t sz3)
{
    double lat = NAN, lon = NAN, alt_m = NAN;

    if (_gnssMsgCurrent(g_gpsMsgs.tLastGga_ms) && !isnan(g_gpsMsgs.gga.latDeg) && !isnan(g_gpsMsgs.gga.lonDeg))
    {
        lat   = g_gpsMsgs.gga.latDeg;
        lon   = g_gpsMsgs.gga.lonDeg;
        alt_m = !isnan(g_gpsMsgs.gga.altM) ? g_gpsMsgs.gga.altM : NAN;
    }
    else if (_gnssMsgCurrent(g_gpsMsgs.tLastRmc_ms) && !isnan(g_gpsMsgs.rmc.latDeg) && !isnan(g_gpsMsgs.rmc.lonDeg))
    {
        lat   = g_gpsMsgs.rmc.latDeg;
        lon   = g_gpsMsgs.rmc.lonDeg;
        alt_m = NAN;
    }

    if (isfinite(lat) && isfinite(lon))
    {
        char latbuf[24];
        char lonbuf[24];
        snprintf(latbuf, sizeof(latbuf), "Lat=%+.6f", lat);
        if (isfinite(alt_m))
        {
            int alt_i = (int)lround(alt_m);
            snprintf(lonbuf, sizeof(lonbuf), "Lon=%+.6f A:%dm", lon, alt_i);
        }
        else
        {
            snprintf(lonbuf, sizeof(lonbuf), "Lon=%+.6f", lon);
        }

        latbuf[20] = '\0';
        lonbuf[20] = '\0';
        snprintf(line2, sz2, "%s", latbuf);
        snprintf(line3, sz3, "%s", lonbuf);
    }
    else
    {
        snprintf(line2, sz2, "Lat=--");
        snprintf(line3, sz3, "Lon=--");
    }
}

static inline void _fmtTopSatsTwoRows(char* row2, size_t sz2, char* row3, size_t sz3)
{
    int idx[6];
    int snr[6];
    for (int i = 0; i < 6; ++i)
    {
        idx[i] = -1;
        snr[i] = -1;
    }

    for (int i = 0; i < g_gsvCount; ++i)
    {
        int s = g_gsvSats[i].snr;
        if (s < 0)
        {
            continue;
        }
        for (int k = 0; k < 6; ++k)
        {
            if (s > snr[k])
            {
                for (int m = 5; m > k; --m)
                {
                    snr[m] = snr[m - 1];
                    idx[m] = idx[m - 1];
                }
                snr[k] = s;
                idx[k] = i;
                break;
            }
        }
    }

    char tmp2[32];
    char tmp3[32];
    int  used = 0;

    {
        char* p   = tmp2;
        int   rem = (int)sizeof(tmp2);
        int   n   = snprintf(p, rem, "SV");
        if (n < 0)
        {
            n = 0;
        }
        if (n >= rem)
        {
            n = rem - 1;
        }
        p += n;
        rem -= n;

        for (int k = 0; k < 3; ++k)
        {
            if (idx[k] >= 0)
            {
                int prn = g_gsvSats[idx[k]].prn % 100;
                int ss  = snr[k] % 100;
                n       = snprintf(p, rem, " %02d/%02d", prn, ss);
                if (n < 0)
                {
                    n = 0;
                }
                if (n >= rem)
                {
                    n = rem - 1;
                }
                p += n;
                rem -= n;
                used++;
            }
        }
        tmp2[20] = '\0';
    }

    {
        char* p   = tmp3;
        int   rem = (int)sizeof(tmp3);
        int   n   = snprintf(p, rem, "SV");
        if (n < 0)
        {
            n = 0;
        }
        if (n >= rem)
        {
            n = rem - 1;
        }
        p += n;
        rem -= n;

        for (int k = 3; k < 6; ++k)
        {
            if (idx[k] >= 0)
            {
                int prn = g_gsvSats[idx[k]].prn % 100;
                int ss  = snr[k] % 100;
                n       = snprintf(p, rem, " %02d/%02d", prn, ss);
                if (n < 0)
                {
                    n = 0;
                }
                if (n >= rem)
                {
                    n = rem - 1;
                }
                p += n;
                rem -= n;
                used++;
            }
        }
        tmp3[20] = '\0';
    }

    if (used == 0)
    {
        snprintf(tmp2, sizeof(tmp2), "SV --");
        snprintf(tmp3, sizeof(tmp3), "SV --");
    }

    snprintf(row2, sz2, "%s", tmp2);
    snprintf(row3, sz3, "%s", tmp3);
}

static void _fmtUpTime(const Uptime& u, char* out, size_t outsz)
{
    snprintf(out,
             outsz,
             "Up=%lud %02u:%02u:%02u",
             (unsigned long)u.day,
             (unsigned)u.hr,
             (unsigned)u.min,
             (unsigned)u.sec);
}

/* _handleLCD — update the LCD with UTC time and oscillator status. */
static void _handleLCD(uint32_t tNow_ms)
{
    static unsigned long lastUpdate_ms = 0;
    static uint32_t      lastCycle_ms  = 0;
    static uint8_t       cycleIdx      = 0;

    if ((tNow_ms - lastUpdate_ms) < 1000UL)
    {
        return;
    }
    lastUpdate_ms = tNow_ms;

    if ((tNow_ms - lastCycle_ms) >= (k_lcdCyclePeriod_s * 1000UL))
    {
        lastCycle_ms = tNow_ms;
        cycleIdx     = (uint8_t)((cycleIdx + 1) % 3);
    }

    // Row 0: UTC / State + Tuning Voltage
    {
        char row0[24];
        if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
        {
            uint32_t hs = g_gpsDoCtrl.holdoverSec;
            uint8_t  hh = (uint8_t)(hs / 3600UL);
            uint8_t  mm = (uint8_t)((hs % 3600UL) / 60UL);
            uint8_t  ss = (uint8_t)(hs % 60UL);
            snprintf(row0, sizeof(row0), "H:%02u:%02u:%02u Vt:%0.4f", (unsigned)hh, (unsigned)mm, (unsigned)ss, g_gpsDoCtrl.target_v);
        }
        else if (!g_gpsDoCtrl.ppsAlive)
        {
            snprintf(row0, sizeof(row0), "NO PPS   Vt:%0.4fV", g_gpsDoCtrl.target_v);
        }
        else if (!g_gpsDoCtrl.gnssFixValid)
        {
            snprintf(row0, sizeof(row0), "NO FIX   Vt:%0.4fV", g_gpsDoCtrl.target_v);
        }
        else
        {
            String utc = _utcStrCompact();
            snprintf(row0, sizeof(row0), "%s Vt:%0.4fV", utc.c_str(), g_gpsDoCtrl.target_v);
        }
        _lcdPrintRow(0, row0);
    }

    // Row 1: Avg frequency (Hz) or Status
    {
        char row1[32];
        if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
        {
            double favg = isfinite(g_gpsDoCtrl.avg_hz) ? g_gpsDoCtrl.avg_hz : 10000000.0;
            snprintf(row1, sizeof(row1), "fA:[HOLD] %.4fHz", favg);
        }
        else if (!g_gpsDoCtrl.ppsAlive)
        {
            snprintf(row1, sizeof(row1), "fA: NO 1PPS SIGNAL");
        }
        else
        {
            double favg_hz = (isfinite(g_gpsDoCtrl.avg_hz) ? g_gpsDoCtrl.avg_hz : (double)g_gpsDoCtrl.tenMhzCount_hz);
            snprintf(row1, sizeof(row1), "fA: %.4f Hz", favg_hz);
        }
        _lcdPrintRow(1, row1);
    }

    // Rows 2 & 3: cycle pages
    if (cycleIdx == 0)
    {
        char r2[32];
        char r3[32];
        char upTimeS[24];
        _fmtUpTime(g_uptime, upTimeS, sizeof(upTimeS));

        snprintf(r2, sizeof(r2), "%s N:%u", upTimeS, g_gpsDoCtrl.avgNSamp);

        uint32_t tAcc_ns = 0;
        if (_gnssMsgCurrent(g_gpsMsgs.tLastTimeUtc_ms))
        {
            tAcc_ns = g_gpsMsgs.timeUtc.tAccNs;
        }

        const uint8_t su = _satsUsed();
        char lock_c = 'N';
        if (g_gpsDoCtrl.state == GpsdoState::LOCKED)
        {
            lock_c = 'Y';
        }
        else if (g_gpsDoCtrl.state == GpsdoState::HOLDOVER)
        {
            lock_c = 'H';
        }
        else if (g_gpsDoCtrl.state == GpsdoState::FAULT)
        {
            lock_c = 'F';
        }

        snprintf(r3, sizeof(r3), "SU:%u tA:%luns L:%c", (unsigned)su, (unsigned long)tAcc_ns, lock_c);

        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
    else if (cycleIdx == 1)
    {
        char r2[32];
        char r3[32];
        _fmtLatLonLines(r2, sizeof(r2), r3, sizeof(r3));
        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
    else
    {
        char r2[32];
        char r3[32];
        _fmtTopSatsTwoRows(r2, sizeof(r2), r3, sizeof(r3));
        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
}

// =====================================================================================================================
/* _handleUptime - Rollover safe uptime */
// =====================================================================================================================
static void _handleUptime(uint32_t tNow_ms)
{
    static bool     init    = false;
    static uint32_t last_ms = 0;
    static uint64_t acc_ms  = 0;

    if (!init)
    {
        init    = true;
        last_ms = tNow_ms;
    }

    uint32_t delta_ms = tNow_ms - last_ms;
    last_ms           = tNow_ms;
    acc_ms += (uint64_t)delta_ms;

    uint64_t t = acc_ms;

    g_uptime.day = (uint32_t)(t / 86400000ULL);
    t            = t % 86400000ULL;

    g_uptime.hr = (uint8_t)(t / 3600000ULL);
    t           = t % 3600000ULL;

    g_uptime.min = (uint8_t)(t / 60000ULL);
    t            = t % 60000ULL;

    g_uptime.sec = (uint8_t)(t / 1000ULL);
    t            = t % 1000ULL;

    g_uptime.tenms = (uint8_t)(t / 10ULL);
}

// =====================================================================================================================
// Main loop
// =====================================================================================================================

void loop()
{
    uint32_t tLoop_ms = millis();

    _handleUptime(tLoop_ms);

    _handlePps(tLoop_ms);
    _handleGnss(tLoop_ms);
    _handleOscTuning(tLoop_ms);
    _handleLockLogic(tLoop_ms);

    _handleLeds(tLoop_ms);
    _handleConsole(tLoop_ms);
    _handleLCD(tLoop_ms);

    g_tuningCycle = false;
}
