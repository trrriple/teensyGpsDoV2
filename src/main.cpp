// GPS-DO
// Teensy 4.0 + u-blox LEA-5T (FW 6.02)

#include <Arduino.h>
#include <imxrt.h>
#include "gnss_ubx5.h"
#include "DAC8550.h"

// =====================================================================================================================
// Options / Configuration
// =====================================================================================================================
static const bool     k_enableSurveyIn         = true;
static const uint32_t k_surveyInMinDur_s       = 600;
static const uint32_t k_surveyInVarLimit       = 900000;
static const uint32_t k_requiredGoodCyclesLock = 10;
static const uint32_t k_requiredPhaseErr_ns    = 2;
static const uint16_t k_tenMhzAvgWindow_s      = 600;

// =====================================================================================================================
// Global State Flags
// =====================================================================================================================
static bool     g_svinSubscribed  = false;
static bool     g_svinStartedByUs = false;
static bool     g_tuningCycle     = false;
static uint32_t g_tLastPps_ms     = 0;
static bool     g_clkRxErr        = false;

// =====================================================================================================================
// Latest decoded NMEA/UBX message cache
// =====================================================================================================================
struct GpsMsgs
{
    NmeaGga gga;
    bool    haveGga = false;

    NmeaRmc rmc;
    bool    haveRmc = false;

    NmeaGsa gsa;
    bool    haveGsa = false;

    UbxTimTp timTp;
    bool     haveTimTp = false;

    UbxNavTimeUtc timeUtc;
    bool          haveTimeUtc = false;
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

#define GNSS_SERIAL    Serial5
#define CONSOLE        Serial
#define LOCKED_LED_PIN 15
#define PPS_LED_PIN    14
#define DAC_CS_PIN     10
// DAC CLK is on Pin   13
// DAC DAT is on Pin   11

// =====================================================================================================================
// DAC / Analog config
// =====================================================================================================================
#define DAC_VREF   3.3f
#define DAC_COUNTS 65536U
static const float k_dacZeroVal = DAC_VREF / 2.0f;        // volts
static const float k_dacLsb     = DAC_VREF / DAC_COUNTS;  // volts per count

DAC8550 g_clkDac(DAC_CS_PIN);

// =====================================================================================================================
// Rolling average state for 10 MHz counter (1 Hz samples)
// =====================================================================================================================
struct TenMhzAvgState
{
    uint32_t    buf[k_tenMhzAvgWindow_s];
    uint16_t    n     = 0;     // valid entries (<= window)
    uint16_t    idx   = 0;     // next write index
    uint64_t    sum   = 0;     // sum of counts
    long double sumsq = 0.0L;  // sum of squares
};

// =====================================================================================================================
// GPSDO controller state (config, internals, metrics)
// =====================================================================================================================
struct GpsdoCtrl
{
    // Config
    double ocxo_hz_per_v   = 5.0;         // OCXO EFC sensitivity (Hz/V), signed
    double vMin            = 0.0;         // V
    double vMax            = DAC_VREF;    // V
    double slewMax_v_per_s = 0.010;       // V/s (max change per update)
    double f0_hz           = 10000000.0;  // expected frequency

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
    double   integ          = 0.0;   // seconds (phase integrator)
    double   target_v       = 1.80;  // V
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
    return (g_gpsDoCtrl.avgNSamp >= k_tenMhzAvgWindow_s) && isfinite(g_gpsDoCtrl.avgErr_ppb);
}

// =====================================================================================================================
/* _tenMhzAvgPush — Add a 1-second frequency sample; update mean/RMS/PPB metrics. */
// =====================================================================================================================
static inline void _tenMhzAvgPush(uint32_t count_hz)
{
    auto& s = g_gpsDoCtrl.avgState;

    if (s.n < k_tenMhzAvgWindow_s)
    {
        s.buf[s.idx] = count_hz;
        s.sum += count_hz;
        s.sumsq += (long double)count_hz * (long double)count_hz;
        s.idx = (s.idx + 1) % k_tenMhzAvgWindow_s;
        s.n++;
    }
    else
    {
        uint32_t old = s.buf[s.idx];
        s.buf[s.idx] = count_hz;

        s.sum += (uint64_t)count_hz - (uint64_t)old;
        s.sumsq += (long double)count_hz * (long double)count_hz - (long double)old * (long double)old;

        s.idx = (s.idx + 1) % k_tenMhzAvgWindow_s;
    }

    // ---- Update exported metrics ----
    g_gpsDoCtrl.avgNSamp = s.n;

    if (s.n > 0)
    {
        long double n      = (long double)s.n;
        long double mu     = (long double)s.sum / n;  // mean (Hz)
        g_gpsDoCtrl.avg_hz = (double)mu;

        if (s.n >= 2)
        {
            long double var = (s.sumsq - n * mu * mu) / (n - 1.0L);  // sample variance
            if (var < 0)
                var = 0;
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
// DAC helper
// =====================================================================================================================

/* _getDacVal — Convert desired voltage to DAC code (counts). */
static int _getDacVal(float desired_v)
{
    return (desired_v - k_dacZeroVal) / k_dacLsb;
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

    CONSOLE.printf("Initializing DAC\r\n");
    SPI.begin();
    g_clkDac.begin();
    int targetVoltageRaw = _getDacVal(g_gpsDoCtrl.target_v);
    g_clkDac.setValue(targetVoltageRaw);
    CONSOLE.printf("DAC Initialized\r\n");

    // setup pulse monitoring
    CONSOLE.printf("Beginning Pulse Counting \r\n");
    _counterInit();
}

// =====================================================================================================================
// GNSS / Console / PPS / Control Handlers
// =====================================================================================================================

/* _handleGnss — Read and decode GNSS streams (NMEA/UBX); update message cache. */
static void _handleGnss()
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
                g_gpsMsgs.gga     = gga;
                g_gpsMsgs.haveGga = true;
            }
        }
        else if (t == NmeaType::RMC)
        {
            NmeaRmc rmc;
            if (gnssParseRmc(line, rmc))
            {
                g_gpsMsgs.rmc     = rmc;
                g_gpsMsgs.haveRmc = true;
            }
        }
        else if (t == NmeaType::GSA)
        {
            NmeaGsa gsa;
            if (gnssParseGsa(line, gsa))
            {
                g_gpsMsgs.gsa     = gsa;
                g_gpsMsgs.haveGsa = true;
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
                    CONSOLE.print("[SVIN] t=");
                    CONSOLE.print(sv.durSec);
                    CONSOLE.print("s  obs=");
                    CONSOLE.print(sv.obs);
                    CONSOLE.print("  var=");
                    CONSOLE.print(sv.meanV_mm2);
                    CONSOLE.print("  active=");
                    CONSOLE.print(sv.active ? "Y" : "N");
                    CONSOLE.print("  valid=");
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
                    g_gpsMsgs.haveTimTp = true;
                }
                break;
            }

            case UbxType::NAV_TIMEUTC:
            {
                if (gnssDecodeNavTimeUtc(fr, g_gpsMsgs.timeUtc))
                {
                    g_gpsMsgs.haveTimeUtc = true;
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

/* _handleConsole — Once per second, print a compact status summary line. */
static void _handleConsole()
{
    static unsigned long lastPrint_ms = 0;
    if (millis() - lastPrint_ms >= 1000)
    {
        lastPrint_ms = millis();

        // =================== GPS ================

        // ==== Time ====
        CONSOLE.print("[GPS] UTC ");
        if (g_gpsMsgs.haveGga && (g_gpsMsgs.gga.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_gpsMsgs.gga.utcHhmmss);
        }
        else if (g_gpsMsgs.haveRmc && (g_gpsMsgs.rmc.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_gpsMsgs.rmc.utcHhmmss);
        }
        else
        {
            CONSOLE.print("--:--:--");
        }

        // ==== Fix / sats ====
        CONSOLE.print(" | FixQ ");
        CONSOLE.print(g_gpsMsgs.haveGga ? g_gpsMsgs.gga.fixQ : 0);

        CONSOLE.print(" | FixType ");
        CONSOLE.print(g_gpsMsgs.haveGsa ? g_gpsMsgs.gsa.fixType : 1);

        CONSOLE.print(" | SatsUsed ");
        if (g_gpsMsgs.haveGga && g_gpsMsgs.gga.sats >= 0)
        {
            CONSOLE.print(g_gpsMsgs.gga.sats);
        }
        else if (g_gpsMsgs.haveGsa && g_gpsMsgs.gsa.used > 0)
        {
            CONSOLE.print(g_gpsMsgs.gsa.used);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== DOPs ====
        CONSOLE.print(" | DOP(P/H/V) ");
        if (g_gpsMsgs.haveGsa && !isnan(g_gpsMsgs.gsa.pdop))
        {
            CONSOLE.print(g_gpsMsgs.gsa.pdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }
        CONSOLE.print("/");
        if (g_gpsMsgs.haveGsa && !isnan(g_gpsMsgs.gsa.hdop))
        {
            CONSOLE.print(g_gpsMsgs.gsa.hdop, 2);
        }
        else if (g_gpsMsgs.haveGga && !isnan(g_gpsMsgs.gga.hdop))
        {
            CONSOLE.print(g_gpsMsgs.gga.hdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }
        CONSOLE.print("/");
        if (g_gpsMsgs.haveGsa && !isnan(g_gpsMsgs.gsa.vdop))
        {
            CONSOLE.println(g_gpsMsgs.gsa.vdop, 2);
        }
        else
        {
            CONSOLE.println("n/a");
        }

        // ==== Position / Alt ====
        CONSOLE.print("[GPS] UTC ");

        CONSOLE.print(" | LLA ");
        bool haveLl = false;
        if (g_gpsMsgs.haveGga && !isnan(g_gpsMsgs.gga.latDeg) && !isnan(g_gpsMsgs.gga.lonDeg))
        {
            CONSOLE.printf("%.7f, %.7f", g_gpsMsgs.gga.latDeg, g_gpsMsgs.gga.lonDeg);
            haveLl = true;
        }
        else if (g_gpsMsgs.haveRmc && !isnan(g_gpsMsgs.rmc.latDeg) && !isnan(g_gpsMsgs.rmc.lonDeg))
        {
            CONSOLE.printf("%.7f, %.7f", g_gpsMsgs.rmc.latDeg, g_gpsMsgs.rmc.lonDeg);
            haveLl = true;
        }
        if (!haveLl)
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | Alt ");
        if (g_gpsMsgs.haveGga && !isnan(g_gpsMsgs.gga.altM))
        {
            CONSOLE.printf("%.2f m", g_gpsMsgs.gga.altM);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== InView (from GSV) ====
        CONSOLE.print(" | InView ");
        CONSOLE.print((g_gsvInView >= 0) ? g_gsvInView : 0);

        // ==== TIM-TP qErr ====
        if (g_gpsMsgs.haveTimTp)
        {
            CONSOLE.print(" | qErr(ns) ");
            CONSOLE.print(g_gpsMsgs.timTp.qErrNs);
        }
        else
        {
            CONSOLE.print(" | TIM-TP n/a");
        }

        // ==== NAV_UTCTIME Acc ====
        if (g_gpsMsgs.haveTimeUtc)
        {
            CONSOLE.print(" | tAcc(ns) ");
            CONSOLE.print(g_gpsMsgs.timeUtc.tAccNs);
        }
        else
        {
            CONSOLE.print(" | NAV_UTCTIME n/a");
        }

        CONSOLE.println();

        // Brief GSV details once per epoch (top SNRs)
        if (g_gsvComplete && g_gsvCount > 0)
        {
            int shown = 0;
            CONSOLE.print("[GPS] Sats: ");
            for (int i = 0; i < g_gsvCount && shown < 12; ++i)
            {
                if (g_gsvSats[i].snr >= 0)
                {
                    CONSOLE.printf("PRN%02d/%ddB  ", g_gsvSats[i].prn, g_gsvSats[i].snr);
                    ++shown;
                }
            }
            CONSOLE.println();
            g_gsvComplete = false;  // print once per epoch
        }

        // =================== OSC Tuning ================
        CONSOLE.printf("[OSC] freq=%.6f MHz dCycles=%.6f phaseErr=%.2f ns  P=%.6f Hz  I=%.6f Hz \r\n",
                       (double)g_gpsDoCtrl.tenMhzCount_hz / 1e6,
                       g_gpsDoCtrl.dCycles,
                       g_gpsDoCtrl.phaseErr_s * 1e9,
                       g_gpsDoCtrl.p_hz,
                       g_gpsDoCtrl.i_hz);

        CONSOLE.printf("[OSC] Tuning: deltaHz=%.9f deltaVolts=%.9f targetVoltage=%.6f\r\n",
                       g_gpsDoCtrl.delta_hz,
                       g_gpsDoCtrl.delta_v,
                       g_gpsDoCtrl.target_v);

        CONSOLE.printf("[OSC] Locked=%i, inHold=%i\r\n", g_gpsDoCtrl.locked, g_gpsDoCtrl.inHold);

        CONSOLE.printf("[OSC] avg=%.6f Hz  rms=%.6f Hz  err=%.6f ppb%s\r\n",
                       g_gpsDoCtrl.avg_hz,
                       g_gpsDoCtrl.rms_hz,
                       g_gpsDoCtrl.avgErr_ppb,
                       _tenMhzAvgReady() ? "" : " (warming up)");
    }
}

// =====================================================================================================================
/* _handlePps — Handle 1 PPS capture; validate and feed the averaging window. */
// =====================================================================================================================
static void _handlePps()
{
    uint32_t tenMhzCount = 0;

    /* if this is true, we've gotten a PPS and latched the 10MHzCount in hardware */
    if (_poll10MHzCount(&tenMhzCount))
    {
        g_tLastPps_ms = millis();

        if (tenMhzCount == 0)
        {
            CONSOLE.printf("Error: Clock not detected \r\n", tenMhzCount);
            g_gpsDoCtrl.tenMhzCount_hz = (uint32_t)-1;
        }
        else if (tenMhzCount > 10000100 || tenMhzCount < 9999900)
        {
            CONSOLE.printf("Warning, Clk count significantly out of range: %i\r\n", tenMhzCount);
            g_gpsDoCtrl.tenMhzCount_hz = 0;
            g_clkRxErr                 = true;
        }
        else
        {
            g_gpsDoCtrl.tenMhzCount_hz = tenMhzCount;
            g_tuningCycle              = true;
            g_clkRxErr                 = false;
            _tenMhzAvgPush(tenMhzCount);
        }
    }
}

// =====================================================================================================================
/* _handleOscTuning — Per-PPS PI phase loop + slow FLL trim + hold/tunnel gating. */
// =====================================================================================================================
void _handleOscTuning()
{
    if (!g_tuningCycle)
    {
        return;
    }

    // Inputs from GPS timepulse quality
    const bool  haveQerr = g_gpsMsgs.haveTimTp;
    const float qErr_ns  = g_gpsMsgs.timTp.qErrNs;

    // Reset qErr data to validate next time
    g_gpsMsgs.haveTimTp = false;

    // 1) Signed cycle error (integer cycles over 1 s gate) with fractional correction from qErr
    const int64_t dCycles_raw = (int64_t)g_gpsDoCtrl.tenMhzCount_hz - (int64_t)g_gpsDoCtrl.f0_hz;
    g_gpsDoCtrl.dCycles       = (double)dCycles_raw - (haveQerr ? (double)qErr_ns * 0.01 : 0.0);

    // 2) Phase error in seconds
    g_gpsDoCtrl.phaseErr_s = g_gpsDoCtrl.dCycles / g_gpsDoCtrl.f0_hz;

    // 3) Deadband: apply to P only; I integrates full error (so it can "walk" across boundary)
    //    Exempt the ±1-cycle flip case from deadband entirely so P sees the small ns error.
    const double deadband_s = 2e-9;  // seconds
    const double err_s      = g_gpsDoCtrl.phaseErr_s;

    double errP_s = err_s;  // error seen by the proportional path

    const double aerr = fabs(err_s);
    if (aerr > deadband_s)
    {
        // use only the amount beyond the deadband
        errP_s = copysign(aerr - deadband_s, err_s);
    }
    else
    {
        // inside deadband: mute P, but keep I integrating
        errP_s = 0.0;
        g_gpsDoCtrl.integ *= 0.999;  // gentle bleed
    }

    // ---- HOLD / TUNNEL (mute P when we're "good enough" on averaged freq and PPS quality ok) ----
    g_gpsDoCtrl.haveAvg = _tenMhzAvgReady();
    g_gpsDoCtrl.goodPps = haveQerr && (fabs((double)qErr_ns) <= g_gpsDoCtrl.qerrHold_ns);
    g_gpsDoCtrl.inHold
        = g_gpsDoCtrl.haveAvg && g_gpsDoCtrl.goodPps && (fabs(g_gpsDoCtrl.avgErr_ppb) <= g_gpsDoCtrl.hold_ppb);

    // 4) PI (negative feedback) — update integrator with full error
    g_gpsDoCtrl.integ += err_s;

    // clamp integrator to sane bounds (scaled as seconds of phase)
    const double integMax_s = 200e-6;  // seconds
    if (g_gpsDoCtrl.integ > integMax_s)
        g_gpsDoCtrl.integ = integMax_s;
    if (g_gpsDoCtrl.integ < -integMax_s)
        g_gpsDoCtrl.integ = -integMax_s;

    // Convert phase errors (s) to Hz corrections by multiplying by f0
    const double effErrP_s = g_gpsDoCtrl.inHold ? 0.0 : errP_s;  // hold mutes P
    g_gpsDoCtrl.p_hz       = -(g_gpsDoCtrl.Kp * effErrP_s * g_gpsDoCtrl.f0_hz);
    g_gpsDoCtrl.i_hz       = -(g_gpsDoCtrl.Ki * g_gpsDoCtrl.integ * g_gpsDoCtrl.f0_hz);

    // deltaHz should equal: -(Kp*errP_s*f0 + Ki*integ*f0)
    g_gpsDoCtrl.delta_hz = g_gpsDoCtrl.p_hz + g_gpsDoCtrl.i_hz;

    // 5) Hz -> Volts using OCXO Hz/V
    g_gpsDoCtrl.delta_v = (g_gpsDoCtrl.ocxo_hz_per_v != 0.0) ? (g_gpsDoCtrl.delta_hz / g_gpsDoCtrl.ocxo_hz_per_v) : 0.0;

    // ------------- SLOW FLL (outer loop)  -------------
    // Occasionally apply a gentle trim based on the windowed average (better SNR than 1 s)
    static uint32_t _tLastFll_ms = 0;
    double          dvFll_v      = 0.0;

    const uint32_t now_ms = millis();
    if ((now_ms - _tLastFll_ms) >= g_gpsDoCtrl.fllUpdate_s * 1000u)
    {
        _tLastFll_ms = now_ms;

        if (g_gpsDoCtrl.avgNSamp >= k_tenMhzAvgWindow_s && isfinite(g_gpsDoCtrl.avg_hz)
            && (g_gpsDoCtrl.ocxo_hz_per_v != 0.0))
        {
            const double avgErr_hz = g_gpsDoCtrl.avg_hz - g_gpsDoCtrl.f0_hz;
            const double ppb       = (avgErr_hz / g_gpsDoCtrl.f0_hz) * 1e9;

            if (fabs(ppb) >= g_gpsDoCtrl.fllThresh_ppb)
            {
                // Suggest a voltage trim: ΔV = -Δf / (Hz/V), apply fraction fllGain
                dvFll_v = -(avgErr_hz / g_gpsDoCtrl.ocxo_hz_per_v) * g_gpsDoCtrl.fllGain;

                // Respect your slew limit over the whole FLL period
                const double maxStep_v = g_gpsDoCtrl.slewMax_v_per_s * (double)g_gpsDoCtrl.fllUpdate_s;
                if (dvFll_v > maxStep_v)
                    dvFll_v = maxStep_v;
                if (dvFll_v < -maxStep_v)
                    dvFll_v = -maxStep_v;
            }
        }
    }

    // 6) Combine fast PI step (per-second) with slow FLL step (sporadic) before clamping
    double deltaVoltsTotal_v = g_gpsDoCtrl.delta_v + dvFll_v;

    // Per-second instantaneous clamp (keeps behavior consistent with your existing slew envelope)
    if (deltaVoltsTotal_v > g_gpsDoCtrl.slewMax_v_per_s)
        deltaVoltsTotal_v = g_gpsDoCtrl.slewMax_v_per_s;
    if (deltaVoltsTotal_v < -g_gpsDoCtrl.slewMax_v_per_s)
        deltaVoltsTotal_v = -g_gpsDoCtrl.slewMax_v_per_s;

    // 7) Update target voltage and clamp to range + light anti-windup on rail hit
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
    const int targetVoltageRaw = _getDacVal(g_gpsDoCtrl.target_v);
    g_clkDac.setValue(targetVoltageRaw);
}

// =====================================================================================================================
/* _handleLockLogic — Simple lock/unlock state machine based on phase error magnitude. */
// =====================================================================================================================
static void _handleLockLogic()
{
    const uint32_t exit_ns         = 150;
    static double  lastPhaseErr_ns = 0.0;

    double phaseErr_ns = g_gpsDoCtrl.phaseErr_s * 1e9;

    // verify we're getting regular PPS and a clock reading
    if (((millis() - g_tLastPps_ms) > 1100) && g_clkRxErr)
    {
        // we should have gotten a pps by now, lock fail
        g_gpsDoCtrl.locked     = 0;
        g_gpsDoCtrl.goodCycles = 0;
        return;
    }

    if (phaseErr_ns == lastPhaseErr_ns)
    {
        // we've not calculated a new error so do nothing (no pps yet)
        return;
    }

    lastPhaseErr_ns = phaseErr_ns;

    if (fabs(phaseErr_ns) < k_requiredPhaseErr_ns)
    {
        if (g_gpsDoCtrl.goodCycles < k_requiredGoodCyclesLock)
        {
            g_gpsDoCtrl.goodCycles++;
        }
    }
    else if (fabs(phaseErr_ns) > exit_ns)
    {
        if (g_gpsDoCtrl.goodCycles > 0)
        {
            g_gpsDoCtrl.goodCycles--;
        }
    }

    g_gpsDoCtrl.locked = (g_gpsDoCtrl.goodCycles >= (k_requiredGoodCyclesLock - 1));
}

// =====================================================================================================================
/* _handleLeds — Blink PPS LED and show lock status on LOCK LED. */
// =====================================================================================================================
static void _handleLeds()
{
    static bool     lockedLedState = false;
    static bool     ppsLedState    = false;
    static uint32_t ppsLedOn_ms    = 0;

    uint32_t tNow_ms = millis();

    if (g_gpsDoCtrl.locked && !lockedLedState)
    {
        digitalWrite(LOCKED_LED_PIN, HIGH);
        lockedLedState = true;
    }
    else if (lockedLedState)
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
// Main loop
// =====================================================================================================================

/* loop — Service PPS, LEDs, GNSS, console, control, and lock logic. */
void loop()
{
    _handlePps();
    _handleLeds();
    _handleGnss();
    _handleConsole();
    _handleOscTuning();
    _handleLockLogic();

    g_tuningCycle = false;
}
