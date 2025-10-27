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
static const uint32_t k_requiredPhaseErr_ns    = 2;
static const uint16_t k_tenMhzAvgWindow_s      = 3600;
static const uint16_t k_tenMhzAvgWindowMin     = 600;
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
#define OSC_TUNE_PWM_PIN 5

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
    double ocxo_hz_per_v   = 5.0;                // OCXO EFC sensitivity (Hz/V), signed
    double vMin            = 0.0;                // V
    double vMax            = OSC_TUNE_VREF;      // V
    double slewMax_v_per_s = 0.010;              // V/s (max change per update)
    double f0_hz           = 10000000.0;         // expected frequency

    // ---- Slow FLL + hold/tunnel parameters ----
    uint32_t fllUpdate_s   = 120;   // run slow FLL every N seconds
    double   fllThresh_ppb = 3.0;   // ignore avg errors inside ±3 ppb
    double   fllGain       = 0.30;  // fraction of suggested trim to apply per run
    double   hold_ppb      = 1.0;   // hold band (±ppb)
    double   qerrHold_ns   = 20.0;  // TIM-TP qErr threshold (ns)

    // Gains
    double Kp = 0.1;
    double Ki = 0.0005;

    // Internals
    uint32_t tenMhzCount_hz = 0;    // 1 s gate → Hz
    double   phaseErr_s     = 0.0;  // seconds
    double   dCycles        = 0.0;
    double   integ          = 0.0;  // seconds (phase integrator)
    double   target_v       = 2.0;  // V
    uint8_t  goodCycles     = 0;
    bool     locked         = false;

    // Metrics
    double delta_hz = NAN;  // Hz
    double delta_v  = NAN;  // V
    double p_hz     = NAN;  // Hz (P contribution)
    double i_hz     = NAN;  // Hz (I contribution)
    bool   inHold   = false;
    bool   haveAvg  = false;
    bool   goodPps  = false;

    // 10 MHz frequency averaging (rolling window, 1 sample = 1 s gate)
    TenMhzAvgState avgState;
    double         avg_hz     = NAN;  // Hz
    double         rms_hz     = NAN;  // Hz (sample stddev)
    double         avgErr_ppb = NAN;  // ppb
    uint16_t       avgNSamp   = 0;    // samples in window
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
    if(millis() - tMsg_ms < 1000)
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

    pinMode(PPS_LED_PIN, OUTPUT);
    digitalWrite(PPS_LED_PIN, LOW);

    pinMode(LOCKED_LED_PIN, OUTPUT);
    digitalWrite(LOCKED_LED_PIN, LOW);

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
    CONSOLE.printf("<%lu:%02u:%02u:%02u.%02u>",
                   (unsigned long)g_uptime.day,
                   g_uptime.hr,
                   g_uptime.min,
                   g_uptime.sec,
                   g_uptime.tenms);
}

/* _handleConsole — Once per second, print a compact, uniform status summary. */
static void _handleConsole(uint32_t tNow_ms)
{
    static unsigned long lastPrint_ms = 0;
    if (tNow_ms - lastPrint_ms >= 1000)
    {
        lastPrint_ms = tNow_ms;

        bool haveGga     = _gnssMsgCurrent(g_gpsMsgs.tLastGga_ms);
        bool haveRmc     = _gnssMsgCurrent(g_gpsMsgs.tLastRmc_ms);
        bool haveGsa     = _gnssMsgCurrent(g_gpsMsgs.tLastGsa_ms);
        bool haveTimTp   = _gnssMsgCurrent(g_gpsMsgs.tLastTimTp_ms);
        bool haveTimeUtc = _gnssMsgCurrent(g_gpsMsgs.tLastTimeUtc_ms);

        // =================== GPS ===================
        _printUptimeStamp();
        CONSOLE.print("[GPS] UTC:");
        if (haveGga && (g_gpsMsgs.gga.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_gpsMsgs.gga.utcHhmmss);
        }
        else if (haveRmc && (g_gpsMsgs.rmc.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_gpsMsgs.rmc.utcHhmmss);
        }
        else
        {
            CONSOLE.print("--:--:--");
        }

        // Fix quality / type / sats used
        CONSOLE.print(" | FixQ:");
        if (haveGga)
        {
            CONSOLE.print(g_gpsMsgs.gga.fixQ);
        }
        else
        {
            CONSOLE.print(0);
        }

        CONSOLE.print(" | FixType:");
        if (haveGsa)
        {
            CONSOLE.print(g_gpsMsgs.gsa.fixType);
        }
        else
        {
            CONSOLE.print(1);
        }

        CONSOLE.print(" | SatsUsed:");
        if (haveGga && g_gpsMsgs.gga.sats >= 0)
        {
            CONSOLE.print(g_gpsMsgs.gga.sats);
        }
        else if (haveGsa && g_gpsMsgs.gsa.used > 0)
        {
            CONSOLE.print(g_gpsMsgs.gsa.used);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // DOPs
        CONSOLE.print(" | DOP_P:");
        if (haveGsa && !isnan(g_gpsMsgs.gsa.pdop))
        {
            CONSOLE.print(g_gpsMsgs.gsa.pdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | DOP_H:");
        if (haveGsa && !isnan(g_gpsMsgs.gsa.hdop))
        {
            CONSOLE.print(g_gpsMsgs.gsa.hdop, 2);
        }
        else if (haveGga && !isnan(g_gpsMsgs.gga.hdop))
        {
            CONSOLE.print(g_gpsMsgs.gga.hdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | DOP_V:");
        if (haveGsa && !isnan(g_gpsMsgs.gsa.vdop))
        {
            CONSOLE.print(g_gpsMsgs.gsa.vdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // Position / Altitude
        CONSOLE.print(" | LLA:");
        bool haveLl = false;
        if (haveGga && !isnan(g_gpsMsgs.gga.latDeg) && !isnan(g_gpsMsgs.gga.lonDeg))
        {
            CONSOLE.printf("%.7f,%.7f", g_gpsMsgs.gga.latDeg, g_gpsMsgs.gga.lonDeg);
            haveLl = true;
        }
        else if (haveRmc && !isnan(g_gpsMsgs.rmc.latDeg) && !isnan(g_gpsMsgs.rmc.lonDeg))
        {
            CONSOLE.printf("%.7f,%.7f", g_gpsMsgs.rmc.latDeg, g_gpsMsgs.rmc.lonDeg);
            haveLl = true;
        }
        if (!haveLl)
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | Alt_m:");
        if (haveGga && !isnan(g_gpsMsgs.gga.altM))
        {
            CONSOLE.print(g_gpsMsgs.gga.altM, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // In-view
        CONSOLE.print(" | InView:");
        if (g_gsvInView >= 0)
        {
            CONSOLE.print(g_gsvInView);
        }
        else
        {
            CONSOLE.print(0);
        }

        // TIM-TP qErr and NAV-TIMEUTC tAcc
        CONSOLE.print(" | qErr_ns:");
        if (haveTimTp)
        {
            CONSOLE.print(g_gpsMsgs.timTp.qErrNs);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | tAcc_ns:");
        if (haveTimeUtc)
        {
            CONSOLE.print(g_gpsMsgs.timeUtc.tAccNs);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.println();

        // Brief GSV details once per epoch (top SNRs)
        if (g_gsvComplete && g_gsvCount > 0)
        {
            int shown = 0;
            _printUptimeStamp();
            CONSOLE.print("[GPS] SatsTopSNR:");
            for (int i = 0; i < g_gsvCount && shown < 12; ++i)
            {
                if (g_gsvSats[i].snr >= 0)
                {
                    CONSOLE.printf("PRN%02d/%ddB ", g_gsvSats[i].prn, g_gsvSats[i].snr);
                    ++shown;
                }
            }
            CONSOLE.println();
            g_gsvComplete = false;  // print once per epoch
        }

        // =================== OSC Tuning ===================
        _printUptimeStamp();
        CONSOLE.printf("[OSC] freq_mhz=%.6f | dCycles=%.6f | phaseErr_ns=%.2f | P_hz=%.6f | I_hz=%.6f\r\n",
                       (double)g_gpsDoCtrl.tenMhzCount_hz / 1e6,
                       g_gpsDoCtrl.dCycles,
                       g_gpsDoCtrl.phaseErr_s * 1e9,
                       g_gpsDoCtrl.p_hz,
                       g_gpsDoCtrl.i_hz);

        _printUptimeStamp();
        CONSOLE.printf("[OSC] delta_hz=%.9f | delta_v=%.9f | target_v=%.6f\r\n",
                       g_gpsDoCtrl.delta_hz,
                       g_gpsDoCtrl.delta_v,
                       g_gpsDoCtrl.target_v);

        _printUptimeStamp();
        CONSOLE.printf("[OSC] locked=%d | hold=%d\r\n", (g_gpsDoCtrl.locked ? 1 : 0), (g_gpsDoCtrl.inHold ? 1 : 0));

        _printUptimeStamp();
        CONSOLE.printf("[OSC] avg_hz=%.6f | rms_hz=%.6f | avgErr_ppb=%.6f | avgReady=%c\r\n",
                       g_gpsDoCtrl.avg_hz,
                       g_gpsDoCtrl.rms_hz,
                       g_gpsDoCtrl.avgErr_ppb,
                       (_tenMhzAvgReady() ? 'Y' : 'N'));
    }
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
/* _handleOscTuning — Per-PPS PI phase loop + slow FLL trim + hold/tunnel gating. */
// =====================================================================================================================
void _handleOscTuning(uint32_t tNow_ms)
{
    if (!g_tuningCycle || g_oscRxErr)
    {
        return;

    }
    // Measure elapsed time since last PPS to stabilize slew behavior
    static uint32_t s_lastPps_ms = 0;
    const double    dt_s         = (s_lastPps_ms == 0) ? 1.0 : (double)(tNow_ms - s_lastPps_ms) / 1000.0;
    s_lastPps_ms                 = tNow_ms;

    // ---- PPS quality (UBX TIM-TP) ----
    const bool  haveQerr = _gnssMsgCurrent(g_gpsMsgs.tLastTimTp_ms);
    const float qErr_ns  = g_gpsMsgs.timTp.qErrNs;

    // 1) Signed cycle error (integer cycles over 1 s gate) with fractional correction from qErr
    const int64_t dCycles_raw = (int64_t)g_gpsDoCtrl.tenMhzCount_hz - (int64_t)g_gpsDoCtrl.f0_hz;
    const double  fracCycles  = haveQerr ? (double)qErr_ns * 0.01 /* 1 cycle / 100 ns at 10 MHz */ : 0.0;

    g_gpsDoCtrl.dCycles = (double)dCycles_raw - fracCycles;

    // 1a) Feed the averaging window **only when PPS is good**
    //     This keeps bad PPS epochs from polluting the long-term average.
    const bool ppsGoodForAvg = haveQerr && (fabs((double)qErr_ns) <= g_gpsDoCtrl.qerrHold_ns);
    if (ppsGoodForAvg)
    {
        const double est_hz = g_gpsDoCtrl.f0_hz + g_gpsDoCtrl.dCycles;  // includes fractional cycle via qErr
        _tenMhzAvgPush(est_hz);
    }

    // 2) Phase error in seconds
    g_gpsDoCtrl.phaseErr_s = (g_gpsDoCtrl.dCycles / g_gpsDoCtrl.f0_hz) * dt_s;


    // ---- HOLD / TUNNEL ----
    g_gpsDoCtrl.haveAvg = _tenMhzAvgReady();
    g_gpsDoCtrl.goodPps = ppsGoodForAvg;  // same quality gate as above
    g_gpsDoCtrl.inHold
        = g_gpsDoCtrl.haveAvg && g_gpsDoCtrl.goodPps && (fabs(g_gpsDoCtrl.avgErr_ppb) <= g_gpsDoCtrl.hold_ppb);

    // 3) PI — update integrator
    g_gpsDoCtrl.integ += g_gpsDoCtrl.phaseErr_s;

    // Optional: in hold, apply a slightly stronger leak to keep I from creeping.
    if (g_gpsDoCtrl.inHold)
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

    // Convert phase errors (s) to Hz corrections
    const double effErrP_s = g_gpsDoCtrl.inHold ? 0.0 : g_gpsDoCtrl.phaseErr_s;  // mute P while in hold
    g_gpsDoCtrl.p_hz       = -(g_gpsDoCtrl.Kp * effErrP_s * g_gpsDoCtrl.f0_hz);
    g_gpsDoCtrl.i_hz       = -(g_gpsDoCtrl.Ki * g_gpsDoCtrl.integ * g_gpsDoCtrl.f0_hz);

    g_gpsDoCtrl.delta_hz = g_gpsDoCtrl.p_hz + g_gpsDoCtrl.i_hz;

    // 4) Hz -> V
    g_gpsDoCtrl.delta_v = (g_gpsDoCtrl.ocxo_hz_per_v != 0.0) ? (g_gpsDoCtrl.delta_hz / g_gpsDoCtrl.ocxo_hz_per_v) : 0.0;

    // ------------- SLOW FLL (outer loop) -------------
    static uint32_t _tLastFll_ms = 0;
    double          dvFll_v      = 0.0;

    if ((tNow_ms - _tLastFll_ms) >= g_gpsDoCtrl.fllUpdate_s * 1000u)
    {
        _tLastFll_ms = tNow_ms;

        if (_tenMhzAvgReady() && (g_gpsDoCtrl.ocxo_hz_per_v != 0.0))
        {
            const double avgErr_hz = g_gpsDoCtrl.avg_hz - g_gpsDoCtrl.f0_hz;
            const double ppb       = (avgErr_hz / g_gpsDoCtrl.f0_hz) * 1e9;

            if (fabs(ppb) >= g_gpsDoCtrl.fllThresh_ppb)
            {
                dvFll_v = -(avgErr_hz / g_gpsDoCtrl.ocxo_hz_per_v) * g_gpsDoCtrl.fllGain;

                // Respect slew over the **FLL period** (not per second)
                const double maxStep_v = g_gpsDoCtrl.slewMax_v_per_s * (double)g_gpsDoCtrl.fllUpdate_s;
                if (dvFll_v > maxStep_v)
                {
                    dvFll_v = maxStep_v;
                }
                if (dvFll_v < -maxStep_v)
                {
                    dvFll_v = -maxStep_v;
                }
            }
        }
    }

    // 6) Combine PI step with FLL step, then clamp by **per-update** slew
    double deltaVoltsTotal_v = g_gpsDoCtrl.delta_v + dvFll_v;

    const double perUpdateSlew_v = g_gpsDoCtrl.slewMax_v_per_s * (dt_s > 0.0 ? dt_s : 1.0);
    if (deltaVoltsTotal_v > perUpdateSlew_v)
    {
        deltaVoltsTotal_v = perUpdateSlew_v;
    }
    if (deltaVoltsTotal_v < -perUpdateSlew_v)
    {
        deltaVoltsTotal_v = -perUpdateSlew_v;
    }

    // 7) Update target V, clamp to rails, light anti-windup on rail hit
    double new_v = g_gpsDoCtrl.target_v + deltaVoltsTotal_v;
    if (new_v < g_gpsDoCtrl.vMin)
    {
        new_v = g_gpsDoCtrl.vMin;
        g_gpsDoCtrl.integ *= 0.9;
    }
    else if (new_v > g_gpsDoCtrl.vMax)
    {
        new_v = g_gpsDoCtrl.vMax;
        g_gpsDoCtrl.integ *= 0.9;
    }
    g_gpsDoCtrl.target_v = new_v;

    // 8) Push to DAC
    _setOscTuneVoltage(g_gpsDoCtrl.target_v);
}

// =====================================================================================================================
/* _handleLockLogic — Simple lock/unlock state machine based on phase error magnitude. */
// =====================================================================================================================
static void _handleLockLogic(uint32_t tNow_ms)
{
    // ---------- Tunables ----------
    const double   enterPhase_ns = 5.0;
    const double   exitPhase_ns  = 12.0;
    const double   enterPpb_avg  = 0.5;
    const double   exitPpb_avg   = 1.0;
    const double   alpha         = 0.90;
    const uint16_t minAvgSamples = 60;
    const uint32_t maxPpsGap_ms  = 1100;
    const double   cycle_ns      = 1e9 / g_gpsDoCtrl.f0_hz;

    // State
    static double  lastPhaseErr_ns = NAN;
    static double  ewmaAbs_ns      = 0.0;
    static uint8_t slipStrikes     = 0;

    // ---------- Guards ----------
    if (((tNow_ms - g_tLastPps_ms) > maxPpsGap_ms) || g_oscRxErr)
    {
        g_gpsDoCtrl.locked     = false;
        g_gpsDoCtrl.goodCycles = 0;
        lastPhaseErr_ns        = NAN;
        ewmaAbs_ns             = 0.0;
        slipStrikes            = 0;
        return;
    }

    // ---------- New phase sample? ----------
    const double phaseErr_ns = g_gpsDoCtrl.phaseErr_s * 1e9;
    if (!isfinite(phaseErr_ns) || phaseErr_ns == lastPhaseErr_ns)
    {
        return;
    }

    const double prevPhase_ns = lastPhaseErr_ns;
    lastPhaseErr_ns           = phaseErr_ns;

    // ---------- Wrap + EWMA ----------
    const double wrapped_ns = phaseErr_ns - cycle_ns * round(phaseErr_ns / cycle_ns);
    ewmaAbs_ns              = alpha * ewmaAbs_ns + (1.0 - alpha) * fabs(wrapped_ns);

    // ---------- Cycle-slip (raw) ----------
    const bool prevValid = isfinite(prevPhase_ns);
    const bool nearEdge  = fabs(phaseErr_ns) > 0.45 * cycle_ns;
    const bool sameSide  = prevValid ? ((phaseErr_ns * prevPhase_ns) > 0.0) : false;

    if (nearEdge && sameSide)
    {
        if (slipStrikes < 255)
        {
            slipStrikes++;
        }
    }
    else
    {
        if (slipStrikes > 0)
        {
            slipStrikes--;
        }
    }
    const bool slipPersist = (slipStrikes >= 3);

    // ---------- Frequency/PPS gates ----------
    const bool avgReady = (g_gpsDoCtrl.avgNSamp >= minAvgSamples) && isfinite(g_gpsDoCtrl.avgErr_ppb);
    const bool goodPps  = g_gpsDoCtrl.goodPps;

    // Use ONLY the averaged ppb once avgReady; else, don't allow entry yet.
    const bool freqEnter = avgReady && (fabs(g_gpsDoCtrl.avgErr_ppb) <= enterPpb_avg);
    const bool freqHold  = avgReady && (fabs(g_gpsDoCtrl.avgErr_ppb) <= exitPpb_avg);

    // ---------- Good-cycle accounting ----------
    const bool inBandForEnter   = (ewmaAbs_ns <= enterPhase_ns) && freqEnter && goodPps && !slipPersist;
    const bool outOfBandForExit = (ewmaAbs_ns > exitPhase_ns) || !freqHold || !goodPps || slipPersist;

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
        digitalWrite(LOCKED_LED_PIN, HIGH);
        lockedLedState = true;
    }
    else if (!g_gpsDoCtrl.locked && lockedLedState)
    {
        digitalWrite(LOCKED_LED_PIN, LOW);
        lockedLedState = false;
    }

    if (g_tuningCycle && !ppsLedState)
    {
        digitalWrite(PPS_LED_PIN, HIGH);
        ppsLedState = true;

        ppsLedOn_ms = tNow_ms;
    }
    else if (ppsLedState && (tNow_ms - ppsLedOn_ms >= 100))
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

static inline void _fmtTimeAccCompact(char* out, size_t outsz, long tAcc_ns)
{
    if (tAcc_ns < 0)
    {
        snprintf(out, outsz, "--");
        return;
    }
    if (tAcc_ns < 1000)
    {
        snprintf(out, outsz, "%ldn", tAcc_ns);
    }
    else if (tAcc_ns < 1000000L)
    {
        long us = (tAcc_ns + 500) / 1000;
        if (us > 999)
        {
            us = 999;
        }
        snprintf(out, outsz, "%ldu", us);
    }
    else
    {
        long ms = (tAcc_ns + 500000) / 1000000L;
        if (ms > 999)
        {
            ms = 999;
        }
        snprintf(out, outsz, "%ldm", ms);
    }
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
   Row0: UTC + Lock/Hold (two spaces before L:)
   Row1: Avg frequency (MHz) + avg ppb
   Rows2-3: cycle between (0) Uptime + SU/tAcc/V, (1) Lat / Lon(+Alt), (2) Top satellites (2 rows)
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

    // ---- Row 0: UTC + Lock/Hold (fits 20) ----
    {
        const char lock_c = (g_gpsDoCtrl.locked ? 'Y' : 'N');
        const char hold_c = (g_gpsDoCtrl.inHold ? 'Y' : 'N');

        // "UTC 12:34:56  L:YH:N" -> 12 + 2 + 6 = 20 chars
        char row0[24];
        String utc = _utcStrCompact();
        snprintf(row0, sizeof(row0), "UTC %s L:%c H:%c", utc.c_str(), lock_c, hold_c);
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
            long tAcc_ns = -1;
            if (_gnssMsgCurrent(g_gpsMsgs.tLastTimeUtc_ms))
            {
                tAcc_ns = (long)g_gpsMsgs.timeUtc.tAccNs;
            }
            _fmtTimeAccCompact(tbuf, sizeof(tbuf), tAcc_ns);

            const uint8_t su = _satsUsed();

            char vbuf[8];
            snprintf(vbuf, sizeof(vbuf), "%.5f", g_gpsDoCtrl.target_v);

            // "SU=12 tA=85n V=1.80"
            snprintf(r3, sizeof(r3), "SU:%u tA:%s V:%s", su, tbuf, vbuf);
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
