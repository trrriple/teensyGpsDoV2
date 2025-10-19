// GPS-DO
// Teensy 4.0 + u-blox LEA-5T (FW 6.02)

#include <Arduino.h>
#include "gnss_ubx5.h"
#include "extIntFreqCount.h"
#include "DAC8550.h"

// ---- Options ----
static const bool     k_enableSurveyIn    = true;
static const uint32_t k_surveyInMinDurSec = 600;
static const uint32_t k_surveyInVarLimit  = 900000;

// ----- State -----
static bool g_svinSubscribed  = false;
static bool g_svinStartedByUs = false;
static bool g_tuningCycle     = false;

// ====== Hold the most recent decoded NMEA/ubx structs ======
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

// ====== GSV epoch aggregator ======
static SatInfo g_gsvSats[64];
static int     g_gsvCount    = 0;
static int     g_gsvInView   = -1;
static int     g_gsvTotal    = 0;
static bool    g_gsvComplete = false;

// ========= Pinout ==============
#define GNSS_SERIAL Serial5
#define CONSOLE Serial
#define PPS_PIN 7
// 10 MHz Clock has to be on pin 9

#define LOCKED_LED_PIN 15
#define PPS_LED_PIN 14

// ======== 10 MHz Tuning DAC =========
#define DAC_VREF 3.3f
#define DAC_COUNTS 65536U
#define DAC_CS_PIN 10
static const float k_dacZeroVal  = DAC_VREF / 2.0f;
static const float k_dacLsb      = DAC_VREF / DAC_COUNTS;

struct GpsdoCtrl
{
    // Config
    double ocxoHzPerVolt = 5.0;  // OCXO EFC sensitivity (Hz/V), signed
    double vMin          = 0.0;
    double vMax          = DAC_VREF;  // clamps for DAC output voltage (defaults 0..vref)
    double slewVoltsMax  = 0.0010;    // max change per second (V)

    // Gains
    double Kp = 0.1;
    double Ki = 0.00005;

    // Internals
    uint32_t nExp          = 10000000u;
    double   f0            = 10000000.0;
    double   integ         = 0.0;             // phase integrator (s)
    double   targetVoltage = 1.80;             // current DAC target (V)
    bool     locked        = false;

    // Metrics
    double  phaseErr_s;
    double  deltaHz;
    double  deltaVolts;
};

GpsdoCtrl g_gpsDoCtrl;

// ============== DAC ====================
DAC8550 g_clkDac(DAC_CS_PIN);

// ============ Helper functions ==============
static void _gsvReset()
{
    g_gsvCount    = 0;
    g_gsvInView   = -1;
    g_gsvTotal    = 0;
    g_gsvComplete = false;
}

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

static int _getDacVal(float desiredV)
{
    return (desiredV - k_dacZeroVal) / k_dacLsb;
}

static double _mm2ToMSigma(uint32_t mm2)
{
    // stddev [m] from variance [mm^2]
    double mm = sqrt((double)mm2);
    return mm / 1000.0;
}

// ====== Pretty helpers ======
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

// ====== Main Setup  ======
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
    int targetVoltageRaw = _getDacVal(2.5);

    CONSOLE.printf("Raw %i\r\n", targetVoltageRaw);
    g_clkDac.setValue(targetVoltageRaw);
    CONSOLE.printf("DAC Initialized\r\n");

    // setup pulse monitoring
    CONSOLE.printf("Beginning Pulse Counting \r\n");
    ExtIntFreqCount.begin(PPS_PIN, FALLING); /* pin to trigger cycle capture */
}

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
                            gnssEnableSurveyIn(k_surveyInMinDurSec, k_surveyInVarLimit);
                            g_svinStartedByUs = true;

                            // Stream Survey-In status (1 Hz) while we run it
                            gnssSendUbxCfgMsg(0x0D, 0x04, /*UART1*/ 1, /*rate*/ 1);  // TIM-SVIN
                            g_svinStartedByUs = true;

                            CONSOLE.printf("[SVIN] Started Survey-In MinDur %i, VarLimit %i",
                                           k_surveyInMinDurSec,
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

static void _handleConsole()
{
    // Once per second, print a compact status line
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 1000)
    {
        lastPrintMs = millis();

        // ==== Time ====
        CONSOLE.print("UTC ");
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
            CONSOLE.print(g_gpsMsgs.gsa.vdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== Position / Alt ====
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
            CONSOLE.print("  GSV: ");
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
    }
}

static uint32_t _handlePps()
{
    if (ExtIntFreqCount.available())
    {
        uint32_t count = ExtIntFreqCount.read();

        if (count > 10000100 || count < 9999900)
        {
            CONSOLE.printf("Warning, Clk count significantly out of range: %i\r\n", count);
            count = 10000000;
        }

        return count;
    }

    return 0;
}

void handleOscTuning(uint32_t count, bool haveQerr, float qErr_ns)
{
    CONSOLE.printf("Tuning: Count %i, have Qerr %i, qErr %f \r\n", count, haveQerr, qErr_ns);

    // 1) Signed cycle error
    int64_t dCycles_raw = (int64_t)count - (int64_t)g_gpsDoCtrl.nExp;

    double dCycles = (double)dCycles_raw - (haveQerr ? (double)qErr_ns * 0.01 : 0.0);
    
    // Detect the 1-cycle toggling case
    bool oneCycleFlip = (llabs(dCycles_raw) == 1) && haveQerr && (fabs(qErr_ns) <= 8.0);
    
    // If it’s the quantization flip, zero the integer part and keep only the ns
    if (oneCycleFlip)
    {
        dCycles = -(double)qErr_ns * 0.01;  // ≈ a few hundredths of a cycle → a few ns
    }

    // 2) Errors
    g_gpsDoCtrl.phaseErr_s = dCycles / g_gpsDoCtrl.f0;

    // 3) Quality gate (optional)
    if (!haveQerr && fabs(g_gpsDoCtrl.phaseErr_s) > 5e-6)
    {
        return;
    }

    // 4) deadband + leak
    const double deadband_s   = 20e-9;
    double       phaseErrUsed = g_gpsDoCtrl.phaseErr_s;
    if (fabs(phaseErrUsed) < deadband_s)
    {
        phaseErrUsed = 0.0;
        g_gpsDoCtrl.integ *= 0.995;  // slow bleed near lock
    }

    // 5) PI (negative feedback)
    g_gpsDoCtrl.integ += phaseErrUsed;
    const double integMax = 200e-6;  // keep tight if using *f0 scaling
    if (g_gpsDoCtrl.integ > integMax)
    {
        g_gpsDoCtrl.integ = integMax;
    }
    if (g_gpsDoCtrl.integ < -integMax)
    {
        g_gpsDoCtrl.integ = -integMax;
    }

    g_gpsDoCtrl.deltaHz
        = -(g_gpsDoCtrl.Kp * phaseErrUsed * g_gpsDoCtrl.f0 + g_gpsDoCtrl.Ki * g_gpsDoCtrl.integ * g_gpsDoCtrl.f0);

    // 6) Hz -> Volts using OCXO Hz/V
    g_gpsDoCtrl.deltaVolts
        = (g_gpsDoCtrl.ocxoHzPerVolt != 0.0) ? (g_gpsDoCtrl.deltaHz / g_gpsDoCtrl.ocxoHzPerVolt) : 0.0;

    CONSOLE.printf("Delta Hz %f, Delta Volt %f\r\n", g_gpsDoCtrl.deltaHz, g_gpsDoCtrl.deltaVolts);

    // 7) Slew limit in volts clamping
    if (g_gpsDoCtrl.deltaVolts > g_gpsDoCtrl.slewVoltsMax)
    {
        g_gpsDoCtrl.deltaVolts = g_gpsDoCtrl.slewVoltsMax;
    }
    if (g_gpsDoCtrl.deltaVolts < -g_gpsDoCtrl.slewVoltsMax)
    {
        g_gpsDoCtrl.deltaVolts = -g_gpsDoCtrl.slewVoltsMax;
    }

    // 8) Update target voltage and clamp to range
    double newV = g_gpsDoCtrl.targetVoltage + g_gpsDoCtrl.deltaVolts;
    if (newV < g_gpsDoCtrl.vMin)
    {
        newV = g_gpsDoCtrl.vMin;
        g_gpsDoCtrl.integ *= 0.9;  // light anti-windup
    }
    else if (newV > g_gpsDoCtrl.vMax)
    {
        newV = g_gpsDoCtrl.vMax;
        g_gpsDoCtrl.integ *= 0.9;
    }
    g_gpsDoCtrl.targetVoltage = newV;

    // 9) Push to DAC
    int targetVoltageRaw = _getDacVal(g_gpsDoCtrl.targetVoltage);
    g_clkDac.setValue(targetVoltageRaw);

    // 10) Lock Metric

    static uint8_t goodCycles     = 0;
    const double   enter_ns       = 50.0;
    const double   exit_ns        = 150.0;
    const int      requiredCycles = 10;

    double phase_ns = (dCycles / g_gpsDoCtrl.f0) * 1e9;

    if (fabs(phase_ns) < enter_ns)
    {
        if (goodCycles < requiredCycles)
        {
            goodCycles++;
        }
    }
    else if (fabs(phase_ns) > exit_ns)
    {
        if (goodCycles > 0)
        {
            goodCycles--;
        }
    }

    // “locked” if mostly good in the window
    g_gpsDoCtrl.locked = (goodCycles >= (requiredCycles - 1));

    CONSOLE.printf("Voltage set %f phaseError_s %.10f locked %i\r\n",
                   g_gpsDoCtrl.targetVoltage,
                   g_gpsDoCtrl.phaseErr_s,
                   g_gpsDoCtrl.locked);

    double pHz = -(g_gpsDoCtrl.Kp * g_gpsDoCtrl.phaseErr_s * g_gpsDoCtrl.f0);
    double iHz = -(g_gpsDoCtrl.Ki * g_gpsDoCtrl.integ * g_gpsDoCtrl.f0);

    CONSOLE.printf("dCycles=%.6f phase=%.1f ns  P=%.3f Hz  I=%.3f Hz  dHz=%.3f Hz\r\n",
                   dCycles,
                   g_gpsDoCtrl.phaseErr_s * 1e9,
                   pHz,
                   iHz,
                   g_gpsDoCtrl.deltaHz);
}

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

// ====== Main Loop  ======
void loop()
{
    uint32_t clkCount = _handlePps();
    
    if(clkCount  != 0)
    {
        g_tuningCycle = true;
    }

    _handleLeds();
    
    _handleGnss();

    if (g_tuningCycle)
    {
        handleOscTuning(clkCount, g_gpsMsgs.haveTimTp, g_gpsMsgs.timTp.qErrNs);
        g_gpsMsgs.haveTimTp = false;
    }

    _handleConsole();

    g_tuningCycle = false;
}
