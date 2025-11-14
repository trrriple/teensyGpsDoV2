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
#endif // OSC TUNE SELECTION

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
static bool     g_svinSubscribed  = false;
static bool     g_svinStartedByUs = false;
static bool     g_tuningCycle     = false;
static uint32_t g_tLastPps_ms     = 0;
static bool     g_oscRxErr        = false;

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

// =====================================================================================================================
// PWM Option instead of discrete DAC
// =====================================================================================================================
#ifdef PWM_OSC_TUNE
Teensy_PWM* g_oscTunePwm;
#define OSC_TUNE_VREF   3.3f
#define OSC_TUNE_COUNTS 65536U
static const float k_clkTunePwmFreq = 1000.0f;
static const float k_oscTuneZeroVal = 0.0f;                             // volts
static const float k_oscTuneLsb     = OSC_TUNE_VREF / OSC_TUNE_COUNTS;  // volts per count
#endif                                                                  // PWM_OSC_TUNE

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
// GPSDO controller state (config, internals, metrics)
// =====================================================================================================================
struct GpsdoCtrl
{
    // Config
    double ocxoHzPerV       = 10.0;               // OCXO EFC sensitivity (Hz/V)
    double vMin             = 0.0;                // V
    double vMax             = OSC_TUNE_VREF;      // V
    double slewMaxVPerS     = 0.001;              // V/s (max change per update)
    double slewMaxVPerSLock = 0.0001;
    double f0_hz            = 10000000.000;       // expected frequency

    // ---- Slow FLL + lock/tunnel parameters ----
    uint32_t fllUpdate_s     = 60;    // run slow FLL every N seconds
    double   fllThresh_ppb   = 0.5;   // only correct if |avgErr_ppb| >= 0.5
    double   qErrMax_ns      = 10;    // Maximum allowable gps PPS qerr
    double   lockEnterPpbReq = 1.5;   // enter lock when |avgErr_ppb| <= 1.5
    double   lockExitPpbReq  = 3.0;   // exit lock when |avgErr_ppb| > 3.0

    // Gains (base = unlocked)
    double Kp = 0.05;
    double Ki = 0.0002;

    // Locked-mode scaling (gentler gains in lock)
    double KpLockScale = 0.2;   // locked Kp = Kp * 0.2 = 0.01
    double KiLockScale = 0.25;  // locked Ki = Ki * 0.25 = 0.00005

    // Internals
    uint32_t tenMhzCount_hz = 0;    // 1 s gate → Hz
    double   phaseErr_s     = 0.0;  // seconds
    double   dCycles        = 0.0;
    double   integ          = 0.0;  // seconds (phase integrator)
    double   target_v       = 2.0;  // V
    uint8_t  goodCycles     = 0;
    bool     locked         = false;
    double   fllResid_v     = 0.0;  // accumulated sub-LSB residual

    // Metrics
    double delta_hz = NAN;  // Hz
    double delta_v  = NAN;  // V
    double p_hz     = NAN;  // Hz (P contribution)
    double i_hz     = NAN;  // Hz (I contribution)
    bool   haveAvg  = false;
    bool   goodPps  = false;

    // 10 MHz frequency averaging (rolling window, 1 sample = 1 s gate)
    TenMhzAvgState avgState;
    double         avg_hz     = NAN;  // Hz
    double         rms_hz     = NAN;  // Hz (sample stddev)
    double         avgErr_ppb = NAN;  // ppb
    uint16_t       avgNSamp   = 0;    // samples in window

    // ---- Debug / logging helpers ----
    double dvFll_v_last      = 0.0;    // last FLL voltage term (V)
    double deltaVTotal_last  = 0.0;    // last combined PI+FLL command after slew (V)
    double lastStep_v        = 0.0;    // last DAC step actually applied (V)
    float  lastQErr_ns       = NAN;    // last TIM-TP qErr used by tuning
};

GpsdoCtrl g_gpsDoCtrl;


// =====================================================================================================================
// Timer helpers: raw read and PPS-latched read
// =====================================================================================================================

/* _readCountRaw — Read live 32-bit counter delta (since last call). */
static uint32_t _readCountRaw()
{
    static uint32_t countPrev = 0;

    IMXRT_TMR_t* TMRx = &IMXRT_TMR3;

    uint32_t count = TMRx->CH[1].CNTR | (TMRx->CH[2].HOLD << 16);  // atomic

    uint32_t countOutput = count - countPrev;
    countPrev            = count;

    return countOutput;
}

/* _poll10MHzCount — Check for PPS capture; on flag, return 1 s count. */
static bool _poll10MHzCount(uint32_t* out_capt)
{
    static uint32_t countPrev = 0;

    IMXRT_TMR_t* TMRx = &IMXRT_TMR3;

    if ((TMRx->CH[0].SCTRL & TMR_SCTRL_IEF) && (TMRx->CH[2].SCTRL & TMR_SCTRL_IEF))
    {
        uint32_t count = TMRx->CH[0].CAPT | (TMRx->CH[2].CAPT << 16);
        TMRx->CH[0].SCTRL &= ~TMR_SCTRL_IEF;
        TMRx->CH[2].SCTRL &= ~TMR_SCTRL_IEF;

        uint32_t countOutput = count - countPrev;
        countPrev            = count;

        if (out_capt)
            *out_capt = countOutput;
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
    if(millis() - tMsg_ms < 1100)
    {
        return true;
    }

    return false;
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

/* _getDacVal — Convert desired voltage to DAC code (counts). */
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
#endif // PWM_OSC_TUNE

}

// =====================================================================================================================
// Setup / Initialization
// =====================================================================================================================

/* setup — Initialize IO, GNSS, DAC, and the QTIMER counter. */
void setup()
{
    CONSOLE.begin(115200);
    while (!CONSOLE && millis() < 2000)
    {
        // wait for USB
    }

    CONSOLE.printf("Teensy GPSDO! \r\n");


    pinMode(PPS_LED_PIN, OUTPUT);
    digitalWrite(PPS_LED_PIN, LOW);

    pinMode(LOCKED_LED_PIN, OUTPUT);
    analogWrite(LOCKED_LED_PIN, 0);

    lcd.init();
    lcd.backlight();
    lcd.print("Initializing.");


    GNSS_SERIAL.begin(9600);
    gnssInit(&GNSS_SERIAL, NULL);

    // Enable the NMEA sentences we decode
    gnssSendPubx40("GGA", true);
    delay(100);
    gnssSendPubx40("RMC", true);
    delay(100);
    gnssSendPubx40("GSA", true);
    delay(100);
    gnssSendPubx40("GSV", true);
    delay(100);

    // Ensure UBX TIM-TP on UART1 (rate=1)
    gnssSendUbxCfgMsg(0x0D, 0x01, /*UART1*/ 1, /*rate*/ 1);

    // Enable UBX-NAV-TIMEUTC (0x01,0x21) on UART1
    gnssSendUbxCfgMsg(0x01, 0x21, /*UART1*/ 1, /*rate*/ 1);

    _gsvReset();

    gnssPollCfgTmode();  // We'll decide what to do in the UBX handler below.

#ifdef DAC_OSC_TUNE
    CONSOLE.printf("Initializing DAC\r\n");
    SPI.begin();
    g_oscTuneDac.begin();
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
    CONSOLE.printf("DAC Initialized\r\n");
#endif // DAC_OSC_TUNE

#ifdef PWM_OSC_TUNE
    g_oscTunePwm = new Teensy_PWM(OSC_TUNE_PWM_PIN, k_clkTunePwmFreq, 50.0);
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
#endif // PWM_OSC_TUNE


    CONSOLE.printf("Beginning Pulse Counting \r\n");
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
                        modeStr = "Disabled";
                    else if (tm.timeMode == 1)
                        modeStr = "Survey-In";
                    else if (tm.timeMode == 2)
                        modeStr = "Fixed";

                    CONSOLE.print("[TMODE] ");
                    CONSOLE.println(modeStr);

                    // --- Decision logic ---
                    if (tm.timeMode == 2)
                    {
                        gnssSendUbxCfgMsg(0x0D, 0x04, /*UART1*/ 1, /*rate*/ 0);  // disable TIM-SVIN
                        g_svinSubscribed  = false;
                        g_svinStartedByUs = false;
                        CONSOLE.println("[SVIN] Unsubscribed TIM-SVIN (Fixed confirmed).");
                    }
                    else
                    {
                        // Not Fixed yet
                        if (k_enableSurveyIn && !g_svinSubscribed)
                        {
                            gnssEnableSurveyIn(k_surveyInMinDur_s, k_surveyInVarLimit);
                            g_svinStartedByUs = true;

                            // Stream Survey-In status (1 Hz) while we run it
                            gnssSendUbxCfgMsg(0x0D, 0x04, /*UART1*/ 1, /*rate*/ 1);  // TIM-SVIN
                            g_svinStartedByUs = true;

                            CONSOLE.printf("[SVIN] Started Survey-In MinDur %i, VarLimit %i",
                                           k_surveyInMinDur_s,
                                           k_surveyInVarLimit);
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
                    CONSOLE.print("[SVIN] t:");
                    CONSOLE.print(sv.durSec);
                    CONSOLE.print("s  obs:");
                    CONSOLE.print(sv.obs);
                    CONSOLE.print("  var:");
                    CONSOLE.print(sv.meanV_mm2);
                    CONSOLE.print("  active:");
                    CONSOLE.print(sv.active ? "Y" : "N");
                    CONSOLE.print("  valid:");
                    CONSOLE.println(sv.valid ? "Y" : "N");

                    // If we started SVIN and it just reported complete → confirm mode switched to Fixed
                    if (g_svinStartedByUs && !sv.active && sv.valid)
                    {
                        CONSOLE.println("[SVIN] COMPLETE → polling TMODE to confirm Fixed...");
                        gnssPollCfgTmode();
                        // We will unsubscribe TIM-SVIN when we actually see CFG-TMODE = Fixed.
                    }
                }
                break;
            }

            case UbxType::TIM_TP:
            {
                if (gnssDecodeTimTp(fr, g_gpsMsgs.timTp))
                {
                    g_gpsMsgs.tLastTimTp_ms = tNow_ms;
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
                // Classified but not handled yet
                break;
            }
        }
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
    // Format: [D:HH:MM:SS.t]  (t = 100 ms ticks)
    CONSOLE.printf("<%lu:%02u:%02u:%02u>",
                   (unsigned long)g_uptime.day,
                   g_uptime.hr,
                   g_uptime.min,
                   g_uptime.sec);
}

/* _handleConsole — Once per second, print a compact, uniform status summary. */
/* _handleConsole — Once per second, print grouped, ~120-char lines. */
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

    // ---------------- Common derived GPS fields ----------------
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

    int   inView     = (g_gsvInView >= 0) ? g_gsvInView : 0;
    float qErr_ns    = haveTimTp ? g_gpsMsgs.timTp.qErrNs : NAN;
    uint32_t tAcc_ns = haveTimeUtc ? g_gpsMsgs.timeUtc.tAccNs : 0;

    char goodPps_c   = g_gpsDoCtrl.goodPps ? 'Y' : 'N';
    char avgReady_c  = _tenMhzAvgReady() ? 'Y' : 'N';

    // =================================================================
    // [TIME] — UTC + qErr + tAcc + PPS / avg status
    // =================================================================
    _printUptimeStamp();
    CONSOLE.print("[TIME] ");

    CONSOLE.print("UTC:");
    if (haveUtc)
    {
        _printUtcFromHhmmss(utcHhmmss);
    }
    else
    {
        CONSOLE.print("--:--:--");
    }

    CONSOLE.print(" | qErr_ns=");
    if (haveTimTp)
    {
        CONSOLE.print(qErr_ns, 1);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | tAcc_ns=");
    if (haveTimeUtc)
    {
        CONSOLE.print(tAcc_ns);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | goodPps=");
    CONSOLE.print(goodPps_c);

    CONSOLE.print(" | avgReady=");
    CONSOLE.print(avgReady_c);

    CONSOLE.print(" | nAvg=");
    CONSOLE.print(g_gpsDoCtrl.avgNSamp);

    CONSOLE.println();

    // =================================================================
    // [GPS] — navigation solution in two shorter lines
    // =================================================================
    // Line 1: fix + sats + DOP
    _printUptimeStamp();
    CONSOLE.print("[GPS] ");

    CONSOLE.print("fixQ=");
    if (haveGga)
    {
        CONSOLE.print(g_gpsMsgs.gga.fixQ);
    }
    else
    {
        CONSOLE.print(0);
    }

    CONSOLE.print(" | fixType=");
    if (haveGsa)
    {
        CONSOLE.print(g_gpsMsgs.gsa.fixType);
    }
    else
    {
        CONSOLE.print(1);
    }

    CONSOLE.print(" | satsUsed=");
    if (satsUsed >= 0)
    {
        CONSOLE.print(satsUsed);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | inView=");
    CONSOLE.print(inView);

    CONSOLE.print(" | pdop=");
    if (!isnan(pdop))
    {
        CONSOLE.print(pdop, 2);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | hdop=");
    if (!isnan(hdop))
    {
        CONSOLE.print(hdop, 2);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | vdop=");
    if (!isnan(vdop))
    {
        CONSOLE.print(vdop, 2);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.println();

    // Line 2: position
    _printUptimeStamp();
    CONSOLE.print("[GPS] ");

    CONSOLE.print("lat=");
    if (!isnan(latDeg))
    {
        CONSOLE.print(latDeg, 7);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | lon=");
    if (!isnan(lonDeg))
    {
        CONSOLE.print(lonDeg, 7);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.print(" | alt_m=");
    if (!isnan(altM))
    {
        CONSOLE.print(altM, 2);
    }
    else
    {
        CONSOLE.print("n/a");
    }

    CONSOLE.println();

    // =================================================================
    // [OSC] — tuning loop in three shorter lines
    // =================================================================

    // Line 1: raw measurement + phase/integrator
    _printUptimeStamp();
    CONSOLE.printf(
        "[OSC] f_mhz=%.6f | cnt=%lu | dCyc=%.6f | phErr_ns=%.3f | integ_s=%.3e\r\n",
        (double)g_gpsDoCtrl.tenMhzCount_hz / 1e6,
        (unsigned long)g_gpsDoCtrl.tenMhzCount_hz,
        g_gpsDoCtrl.dCycles,
        g_gpsDoCtrl.phaseErr_s * 1e9,
        g_gpsDoCtrl.integ);

    // Line 2: PI and FLL terms
    _printUptimeStamp();
    CONSOLE.printf(
        "[OSC] P_hz=%.6f | I_hz=%.6f | dHz=%.6f | dV=%.9f | dvFLL=%.9f\r\n",
        g_gpsDoCtrl.p_hz,
        g_gpsDoCtrl.i_hz,
        g_gpsDoCtrl.delta_hz,
        g_gpsDoCtrl.delta_v,
        g_gpsDoCtrl.dvFll_v_last);

    // Line 3: DAC behavior + long-term stats
    _printUptimeStamp();
    CONSOLE.printf(
        "[OSC] dVtot=%.9f | step_v=%.9f | resid_v=%.9f | Vt=%.6f | lock=%d"
        " | fA=%.6f | err_ppb=%.6f | rms_hz=%.6f\r\n",
        g_gpsDoCtrl.deltaVTotal_last,
        g_gpsDoCtrl.lastStep_v,
        g_gpsDoCtrl.fllResid_v,
        g_gpsDoCtrl.target_v,
        (g_gpsDoCtrl.locked ? 1 : 0),
        g_gpsDoCtrl.avg_hz,
        g_gpsDoCtrl.avgErr_ppb,
        g_gpsDoCtrl.rms_hz);
}


// =====================================================================================================================
/* _handlePps — Handle 1 PPS capture; validate and feed the averaging window. */
// =====================================================================================================================
static void _handlePps(uint32_t tNow_ms)
{
    uint32_t tenMhzCount = 0;

    /* if this is true, we've gotten a PPS and latched the 10MHzCount in hardware */
    if (_poll10MHzCount(&tenMhzCount))
    {
        g_tLastPps_ms = tNow_ms;

        if (tenMhzCount == 0)
        {
            CONSOLE.printf("Error: Clock not detected \r\n", tenMhzCount);
            g_gpsDoCtrl.tenMhzCount_hz = (uint32_t)-1;
            g_oscRxErr                 = true;

        }
        else if (tenMhzCount > 10000100 || tenMhzCount < 9999900)
        {
            CONSOLE.printf("Warning, Clk count significantly out of range: %i\r\n", tenMhzCount);
            g_gpsDoCtrl.tenMhzCount_hz = 0;
            g_oscRxErr                 = true;
        }
        else
        {
            g_gpsDoCtrl.tenMhzCount_hz = tenMhzCount;
            g_tuningCycle              = true;
            g_oscRxErr                 = false;
        }
    }
}

// =====================================================================================================================
/* _handleOscTuning — Per-PPS PI phase loop + slow FLL trim + lock/tunnel gating. */
// =====================================================================================================================
void _handleOscTuning(uint32_t tNow_ms)
{
    if (!g_tuningCycle || g_oscRxErr)
    {
        return;
    }

    // ---- PPS quality (UBX TIM-TP) ----
    const bool  haveQerr = _gnssMsgCurrent(g_gpsMsgs.tLastTimTp_ms);
    const float qErr_ns  = g_gpsMsgs.timTp.qErrNs;

    g_gpsDoCtrl.lastQErr_ns = haveQerr ? qErr_ns : NAN;

    // 1) Signed cycle error (integer cycles over 1 s gate) with fractional correction from qErr
    const int64_t dCycles_raw = (int64_t)g_gpsDoCtrl.tenMhzCount_hz - (int64_t)g_gpsDoCtrl.f0_hz;
    const double  fracCycles  = haveQerr ? (double)qErr_ns * 0.01 /* 1 cycle / 100 ns at 10 MHz */ : 0.0;

    g_gpsDoCtrl.dCycles = (double)dCycles_raw - fracCycles;

    // 1a) Feed the averaging window **only when PPS is good**
    g_gpsDoCtrl.goodPps  = haveQerr && (fabs((double)qErr_ns) <= g_gpsDoCtrl.qErrMax_ns);

    if ( g_gpsDoCtrl.goodPps )
    {
        const double est_hz = g_gpsDoCtrl.f0_hz + g_gpsDoCtrl.dCycles;  // includes fractional cycle via qErr
        _tenMhzAvgPush(est_hz);
    }

    // 2) Phase error in seconds
    g_gpsDoCtrl.phaseErr_s = g_gpsDoCtrl.dCycles / g_gpsDoCtrl.f0_hz;

    // ---- Lock / TUNNEL ----
    g_gpsDoCtrl.haveAvg = _tenMhzAvgReady();

    // 3) PI — update integrator (always, but gains differ in lock vs unlock)
    g_gpsDoCtrl.integ += g_gpsDoCtrl.phaseErr_s;

    // Optional: in lock, apply a slightly stronger leak to keep I from creeping.
    if (g_gpsDoCtrl.locked)
    {
        g_gpsDoCtrl.integ *= 0.9995;
    }

    // clamp integrator (units = seconds of phase)
    const double integMax_s = 200e-6;
    if (g_gpsDoCtrl.integ > integMax_s)
    {
        g_gpsDoCtrl.integ = integMax_s;
    }
    if (g_gpsDoCtrl.integ < -integMax_s)
    {
        g_gpsDoCtrl.integ = -integMax_s;
    }

    // ---- Gains: gentler in lock ----
    double Kp_use = g_gpsDoCtrl.Kp;
    double Ki_use = g_gpsDoCtrl.Ki;
    if (g_gpsDoCtrl.locked)
    {
        Kp_use *= g_gpsDoCtrl.KpLockScale;
        Ki_use *= g_gpsDoCtrl.KiLockScale;
    }

    // Convert phase errors (s) to Hz corrections
    const double effErrP_s = g_gpsDoCtrl.phaseErr_s;  // no longer muted in lock
    g_gpsDoCtrl.p_hz       = -(Kp_use * effErrP_s * g_gpsDoCtrl.f0_hz);
    g_gpsDoCtrl.i_hz       = -(Ki_use * g_gpsDoCtrl.integ * g_gpsDoCtrl.f0_hz);

    g_gpsDoCtrl.delta_hz = g_gpsDoCtrl.p_hz + g_gpsDoCtrl.i_hz;

    // 4) Hz -> V (PI command)
    g_gpsDoCtrl.delta_v = (g_gpsDoCtrl.ocxoHzPerV != 0.0)
                              ? (g_gpsDoCtrl.delta_hz / g_gpsDoCtrl.ocxoHzPerV)
                              : 0.0;

    // ------------- SLOW FLL (outer loop) -------------
    static uint32_t _tLastFll_ms = 0;
    double          dvFll_v      = 0.0;

    if ((tNow_ms - _tLastFll_ms) >= g_gpsDoCtrl.fllUpdate_s * 1000u)
    {
        _tLastFll_ms = tNow_ms;

        if (_tenMhzAvgReady() && (g_gpsDoCtrl.ocxoHzPerV != 0.0))
        {
            const double avgErr_hz = g_gpsDoCtrl.avg_hz - g_gpsDoCtrl.f0_hz;
            const double ppb       = (avgErr_hz / g_gpsDoCtrl.f0_hz) * 1e9;

            if (fabs(ppb) >= g_gpsDoCtrl.fllThresh_ppb)
            {
                // Raw naive step
                const double dvRaw_v = -(avgErr_hz / g_gpsDoCtrl.ocxoHzPerV);

                // Make FLL step gentle
                dvFll_v = dvRaw_v * 0.25;

                // Hard limit FLL step per update to ~0.2 ppb
                const double maxFllStep_v = 0.0002; // at 10 Hz/V -> 0.002 Hz ≈ 0.2 ppb
                if (dvFll_v > maxFllStep_v)  dvFll_v = maxFllStep_v;
                if (dvFll_v < -maxFllStep_v) dvFll_v = -maxFllStep_v;
            }
        }
    }

    g_gpsDoCtrl.dvFll_v_last = dvFll_v;

    // --- Quantization-aware post-processing ---
    double deltaVoltsTotal_v = g_gpsDoCtrl.delta_v + dvFll_v;

    // Slew limit per update
    if (deltaVoltsTotal_v > g_gpsDoCtrl.slewMaxVPerS)
    {
        deltaVoltsTotal_v = g_gpsDoCtrl.slewMaxVPerS;
    }
    if (deltaVoltsTotal_v < -g_gpsDoCtrl.slewMaxVPerS)
    {
        deltaVoltsTotal_v = -g_gpsDoCtrl.slewMaxVPerS;
    }

    g_gpsDoCtrl.deltaVTotal_last = deltaVoltsTotal_v;

    g_gpsDoCtrl.lastStep_v = 0.0;  // default this cycle

    if (g_gpsDoCtrl.goodPps)
    {
        // Accumulate sub-LSBs so they aren't lost (especially for tiny FLL trims)
        g_gpsDoCtrl.fllResid_v += deltaVoltsTotal_v;

        const double vDeadband = 0.5 * k_oscTuneLsb;  // don’t move for < 1/2 LSB
        double       step_v    = 0.0;

        if (fabs(g_gpsDoCtrl.fllResid_v) >= vDeadband)
        {
            // Round to nearest code
            const double codes = round(g_gpsDoCtrl.fllResid_v / k_oscTuneLsb);
            step_v             = codes * k_oscTuneLsb;

            // Keep only the leftover fractional part
            g_gpsDoCtrl.fllResid_v -= step_v;
        }

        g_gpsDoCtrl.lastStep_v = step_v;

        if (step_v != 0.0)
        {
            double new_v = g_gpsDoCtrl.target_v + step_v;

            if (new_v < g_gpsDoCtrl.vMin)
            {
                new_v = g_gpsDoCtrl.vMin;
                g_gpsDoCtrl.integ *= 0.9;
                g_gpsDoCtrl.fllResid_v = 0.0;  // hitting a rail => discard residual
            }
            else if (new_v > g_gpsDoCtrl.vMax)
            {
                new_v = g_gpsDoCtrl.vMax;
                g_gpsDoCtrl.integ *= 0.9;
                g_gpsDoCtrl.fllResid_v = 0.0;
            }

            g_gpsDoCtrl.target_v = new_v;
            _setOscTuneVoltage(g_gpsDoCtrl.target_v);
        }
    }
}



// =====================================================================================================================
/* _handleLockLogic — Simple lock/unlock logic */
// =====================================================================================================================
static void _handleLockLogic(uint32_t tNow_ms)
{
    if (!g_tuningCycle)
    {
        return;
    }

    // ---------- Guards ----------
    if (((tNow_ms - g_tLastPps_ms) > 1100) || g_oscRxErr)
    {
        g_gpsDoCtrl.locked     = false;
        g_gpsDoCtrl.goodCycles = 0;
        return;
    }

    // ---------- Frequency/PPS gates ----------
    const bool avgReady = _tenMhzAvgReady();
    const bool goodPps  = g_gpsDoCtrl.goodPps;

    const double absPpb = fabs(g_gpsDoCtrl.avgErr_ppb);

    // Use ONLY the averaged ppb once avgReady; else, don't allow entry yet.
    const bool freqEnter = avgReady && (absPpb <= g_gpsDoCtrl.lockEnterPpbReq);
    const bool freqHold  = avgReady && (absPpb <= g_gpsDoCtrl.lockExitPpbReq);

    // ---------- Good-cycle accounting ----------
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

    // ---------- Final latch: forbid lock until avgReady ----------
    g_gpsDoCtrl.locked = (avgReady && (g_gpsDoCtrl.goodCycles >= (k_requiredGoodCyclesLock - 1)));
}

// =====================================================================================================================
/* _handleLeds — Blink PPS LED and show lock status on LOCK LED. */
// =====================================================================================================================
static void _handleLeds(uint32_t tNow_ms)
{
    static bool     lockedLedState = false;
    static bool     ppsLedState    = false;
    static uint32_t ppsLedOn_ms    = 0;


    if (g_gpsDoCtrl.locked && !lockedLedState)
    {
        analogWrite(LOCKED_LED_PIN, 5);
        lockedLedState = true;
    }
    else if (!g_gpsDoCtrl.locked && lockedLedState)
    {
        analogWrite(LOCKED_LED_PIN, 0);
        lockedLedState = false;
    }

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
// _handleLCD — update the LCD with UTC time and oscillator status (1 Hz refresh). */
// =====================================================================================================================
// ---------------- helpers ----------------

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
        // higher precision: 6 decimals
        char latbuf[24];
        char lonbuf[24];
        snprintf(latbuf, sizeof(latbuf), "Lat=%+.6f", lat);
        if (isfinite(alt_m))
        {
            // Put Alt on lon line to keep two lines
            int alt_i = (int)lround(alt_m);
            // leave room: " Lon=-122.084000 A:12m"
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
    // pick top-6 SNR
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
        // insert into top-6
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

    // Build "SV xx/yy xx/yy xx/yy"
    char tmp2[32];
    char tmp3[32];
    int  used = 0;

    {
        char* p = tmp2;
        int   rem = (int)sizeof(tmp2);
        int   n = snprintf(p, rem, "SV");
        if (n < 0)
        {
            n = 0;
        }
        if (n >= rem)
        {
            n = rem - 1;
        }
        p   += n;
        rem -= n;

        for (int k = 0; k < 3; ++k)
        {
            if (idx[k] >= 0)
            {
                int prn = g_gsvSats[idx[k]].prn % 100;
                int ss  = snr[k] % 100;
                n = snprintf(p, rem, " %02d/%02d", prn, ss);
                if (n < 0)
                {
                    n = 0;
                }
                if (n >= rem)
                {
                    n = rem - 1;
                }
                p   += n;
                rem -= n;
                used++;
            }
        }
        tmp2[20] = '\0';
    }

    {
        char* p = tmp3;
        int   rem = (int)sizeof(tmp3);
        int   n = snprintf(p, rem, "SV");
        if (n < 0)
        {
            n = 0;
        }
        if (n >= rem)
        {
            n = rem - 1;
        }
        p   += n;
        rem -= n;

        for (int k = 3; k < 6; ++k)
        {
            if (idx[k] >= 0)
            {
                int prn = g_gsvSats[idx[k]].prn % 100;
                int ss  = snr[k] % 100;
                n = snprintf(p, rem, " %02d/%02d", prn, ss);
                if (n < 0)
                {
                    n = 0;
                }
                if (n >= rem)
                {
                    n = rem - 1;
                }
                p   += n;
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

// ---------------- main LCD handler ----------------

/* _handleLCD — 20x4 LCD:
*/
static void _handleLCD(uint32_t tNow_ms)
{
    static unsigned long lastUpdate_ms = 0;
    static uint32_t      lastCycle_ms  = 0;
    static uint8_t       cycleIdx      = 0;  // 0: uptime+sys, 1: lat/lon, 2: sats

    if ((tNow_ms - lastUpdate_ms) < 1000UL)
    {
        return;
    }
    lastUpdate_ms = tNow_ms;

    if ((tNow_ms - lastCycle_ms) >= (k_lcdCyclePeriod_s * 1000UL))
    {
        lastCycle_ms = tNow_ms;
        cycleIdx = (uint8_t)((cycleIdx + 1) % 3);
    }

    // ---- Row 0: UTC + Tuning Voltage (fits 20) ----
    {
        // "UTC 12:34:56  L:YH:N"
        char row0[24];
        String utc = _utcStrCompact();
        snprintf(row0, sizeof(row0), "%s  Vt:%0.5f", utc.c_str(), g_gpsDoCtrl.target_v);
        _lcdPrintRow(0, row0);
    }

    // ---- Row 1: Avg frequency (MHz) ----
    {
        double favg_hz  = (isfinite(g_gpsDoCtrl.avg_hz) ? g_gpsDoCtrl.avg_hz : (double)g_gpsDoCtrl.tenMhzCount_hz);

        char row1[32];
        if (favg_hz >= 10000000)
        {
            snprintf(row1, sizeof(row1), "fA: %.4f", favg_hz);
        }
        else
        {
            snprintf(row1, sizeof(row1), "fA:  %.4f", favg_hz);
        }
        _lcdPrintRow(1, row1);
    }

    // ---- Rows 2 & 3: cycle pages ----
    if (cycleIdx == 0)
    {
        // Page 0: Uptime (row2) + SU/tAcc/V (row3)
        char r2[32];
        char r3[32];

        char upTimeS[32];
        _fmtUpTime(g_uptime, upTimeS, sizeof(upTimeS));

        snprintf(r2, 32, "%s N:%i", upTimeS, g_gpsDoCtrl.avgNSamp);

        {
            char tbuf[8];
            uint32_t tAcc_ns = 0;
            if (_gnssMsgCurrent(g_gpsMsgs.tLastTimeUtc_ms))
            {
                tAcc_ns = g_gpsMsgs.timeUtc.tAccNs;
            }
            
            const uint8_t su = _satsUsed();

            const char lock_c = (g_gpsDoCtrl.locked ? 'Y' : 'N');

            // "SU=12 tA=85ns L:Y H:Y"
            snprintf(r3, sizeof(r3), "SU:%u tA:%i ns L:%c", su, tAcc_ns, lock_c);
        }

        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
    else if (cycleIdx == 1)
    {
        // Page 1: Lat (row2) / Lon(+Alt) (row3)
        char r2[32];
        char r3[32];
        _fmtLatLonLines(r2, sizeof(r2), r3, sizeof(r3));
        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
    else
    {
        // Page 2: Top satellites across both rows (up to 6 PRN/SNR)
        char r2[32];
        char r3[32];
        _fmtTopSatsTwoRows(r2, sizeof(r2), r3, sizeof(r3));
        _lcdPrintRow(2, r2);
        _lcdPrintRow(3, r3);
    }
}

// =====================================================================================================================
/* _handleUptime - Calculate a rollover safe uptime  */
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

    uint32_t delta_ms = tNow_ms - last_ms;   // wrap-safe unsigned diff
    last_ms = tNow_ms;
    acc_ms += (uint64_t)delta_ms;

    uint64_t t = acc_ms;

    g_uptime.day  = (uint32_t)(t / 86400000ULL);
    t             = t % 86400000ULL;

    g_uptime.hr   = (uint8_t)(t / 3600000ULL);
    t             = t % 3600000ULL;

    g_uptime.min  = (uint8_t)(t / 60000ULL);
    t             = t % 60000ULL;

    g_uptime.sec  = (uint8_t)(t / 1000ULL);
    t             = t % 1000ULL;

    g_uptime.tenms= (uint8_t)(t / 10ULL);   // 0..99
}

// =====================================================================================================================
// Main loop
// =====================================================================================================================

/* loop — Service PPS, LEDs, GNSS, console, control, and lock logic. */
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
