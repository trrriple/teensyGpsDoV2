// Teensy4_LEA5T_main.ino
// Teensy 4.0 + u-blox LEA-5T (FW 6.02)
// Unified splitter pops raw NMEA / UBX. We call the library’s parsers to get typed structs,
// then print a tidy single-line status once per second (like before), plus a short GSV block.

#include <Arduino.h>
#include "gnss_ubx5.h"

#define GNSS_SERIAL Serial5
#define CONSOLE Serial


// ---- Options ----
static const bool k_enableSurveyIn = true;  // <-- toggle here


// ====== Hold the most recent decoded NMEA structs ======
static NmeaGga g_gga;
static bool    g_haveGga = false;

static NmeaRmc g_rmc;
static bool    g_haveRmc = false;

static NmeaGsa g_gsa;
static bool    g_haveGsa = false;

static bool g_svinSubscribed  = false;
static bool g_svinStartedByUs = false;

// ====== GSV epoch aggregator ======
static SatInfo g_gsvSats[64];
static int     g_gsvCount    = 0;
static int     g_gsvInView   = -1;
static int     g_gsvTotal    = 0;
static bool    g_gsvComplete = false;


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

static double _mm2ToMSigma(uint32_t mm2) {
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

    GNSS_SERIAL.begin(9600);
    gnssInit(&CONSOLE);  // pass null to silence library debug

    // Enable the NMEA sentences we decode
    gnssSendPubx40(GNSS_SERIAL, "GGA", true);
    delay(100);
    gnssSendPubx40(GNSS_SERIAL, "RMC", true);
    delay(100);
    gnssSendPubx40(GNSS_SERIAL, "GSA", true);
    delay(100);
    gnssSendPubx40(GNSS_SERIAL, "GSV", true);
    delay(100);

    // Ensure UBX TIM-TP on UART1 (rate=1)
    gnssSendUbxCfgMsg(GNSS_SERIAL, 0x0D, 0x01, /*UART1*/ 1, /*rate*/ 1);
    gnssSendUbxCfgMsg(GNSS_SERIAL, 0x0D, 0x04, /*UART1*/ 1, /*rate*/ 1);

    _gsvReset();


    gnssPollCfgTmode(GNSS_SERIAL);   // We'll decide what to do in the UBX handler below.


    // if (k_enableSurveyIn)
    // {
    //     gnssEnableSurveyIn(GNSS_SERIAL, 600, 9000000U);
    //     CONSOLE.println("Survey-In requested (min 10m, sigma<=3 m).");
    // }

}

// ====== Main Loop  ======
void loop()
{
    // Feed stream
    while (GNSS_SERIAL.available())
    {
        gnssFeedByte((uint8_t)GNSS_SERIAL.read());
    }

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
                g_gga     = gga;
                g_haveGga = true;
            }
        }
        else if (t == NmeaType::RMC)
        {
            NmeaRmc rmc;
            if (gnssParseRmc(line, rmc))
            {
                g_rmc     = rmc;
                g_haveRmc = true;
            }
        }
        else if (t == NmeaType::GSA)
        {
            NmeaGsa gsa;
            if (gnssParseGsa(line, gsa))
            {
                g_gsa     = gsa;
                g_haveGsa = true;
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
    static int32_t lastQerrNs = 0;
    static bool    haveTimTp  = false;
    UbxFrame       fr;
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
                        // Already Fixed → ensure we are NOT subscribed to TIM-SVIN
                        if (g_svinSubscribed)
                        {
                            gnssSendUbxCfgMsg(GNSS_SERIAL, 0x0D, 0x04, /*UART1*/ 1, /*rate*/ 0);  // disable TIM-SVIN
                            g_svinSubscribed  = false;
                            g_svinStartedByUs = false;
                            CONSOLE.println("[SVIN] Unsubscribed TIM-SVIN (Fixed confirmed).");
                        }
                        // Nothing else to do.
                    }
                    else
                    {
                        // Not Fixed yet
                        if (k_enableSurveyIn && !g_svinSubscribed)
                        {
                            // Start Survey-In and subscribe to TIM-SVIN *now*
                            // Set your thresholds here:
                            const uint32_t minDurSec = 12UL * 3600UL;  // example: 12 hours
                            const uint32_t varLimit  = 100U * 100U;    // example: σ ≤ 100 mm → 10,000 mm^2

                            gnssEnableSurveyIn(GNSS_SERIAL, minDurSec, varLimit);
                            g_svinStartedByUs = true;

                            // Stream Survey-In status (1 Hz) while we run it
                            gnssSendUbxCfgMsg(GNSS_SERIAL, 0x0D, 0x04, /*UART1*/ 1, /*rate*/ 1);  // TIM-SVIN
                            g_svinStartedByUs = true;

                            CONSOLE.println("[SVIN] Started Survey-In and subscribed TIM-SVIN.");
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
                        gnssPollCfgTmode(GNSS_SERIAL);
                        // We will unsubscribe TIM-SVIN when we actually see CFG-TMODE = Fixed.
                    }
                }
                break;
            }

            case UbxType::TIM_TP:
            {
                UbxTimTp tp;
                if (gnssDecodeTimTp(fr, tp))
                {
                    lastQerrNs = tp.qErrNs;
                    haveTimTp  = true;
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

    // Once per second, print a compact status line
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 1000)
    {
        lastPrintMs = millis();

        // ==== Time ====
        CONSOLE.print("UTC ");
        if (g_haveGga && (g_gga.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_gga.utcHhmmss);
        }
        else if (g_haveRmc && (g_rmc.utcHhmmss > 0.0))
        {
            _printUtcFromHhmmss(g_rmc.utcHhmmss);
        }
        else
        {
            CONSOLE.print("--:--:--");
        }

        // ==== Fix / sats ====
        CONSOLE.print(" | FixQ ");
        CONSOLE.print(g_haveGga ? g_gga.fixQ : 0);

        CONSOLE.print(" | FixType ");
        CONSOLE.print(g_haveGsa ? g_gsa.fixType : 1);

        CONSOLE.print(" | SatsUsed ");
        if (g_haveGga && g_gga.sats >= 0)
        {
            CONSOLE.print(g_gga.sats);
        }
        else if (g_haveGsa && g_gsa.used > 0)
        {
            CONSOLE.print(g_gsa.used);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== DOPs ====
        CONSOLE.print(" | DOP(P/H/V) ");
        if (g_haveGsa && !isnan(g_gsa.pdop))
        {
            CONSOLE.print(g_gsa.pdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }
        CONSOLE.print("/");
        if (g_haveGsa && !isnan(g_gsa.hdop))
        {
            CONSOLE.print(g_gsa.hdop, 2);
        }
        else if (g_haveGga && !isnan(g_gga.hdop))
        {
            CONSOLE.print(g_gga.hdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }
        CONSOLE.print("/");
        if (g_haveGsa && !isnan(g_gsa.vdop))
        {
            CONSOLE.print(g_gsa.vdop, 2);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== Position / Alt ====
        CONSOLE.print(" | LLA ");
        bool haveLl = false;
        if (g_haveGga && !isnan(g_gga.latDeg) && !isnan(g_gga.lonDeg))
        {
            CONSOLE.printf("%.7f, %.7f", g_gga.latDeg, g_gga.lonDeg);
            haveLl = true;
        }
        else if (g_haveRmc && !isnan(g_rmc.latDeg) && !isnan(g_rmc.lonDeg))
        {
            CONSOLE.printf("%.7f, %.7f", g_rmc.latDeg, g_rmc.lonDeg);
            haveLl = true;
        }
        if (!haveLl)
        {
            CONSOLE.print("n/a");
        }

        CONSOLE.print(" | Alt ");
        if (g_haveGga && !isnan(g_gga.altM))
        {
            CONSOLE.printf("%.2f m", g_gga.altM);
        }
        else
        {
            CONSOLE.print("n/a");
        }

        // ==== InView (from GSV) ====
        CONSOLE.print(" | InView ");
        CONSOLE.print((g_gsvInView >= 0) ? g_gsvInView : 0);

        // ==== TIM-TP qErr ====
        if (haveTimTp)
        {
            CONSOLE.print(" | qErr(ns) ");
            CONSOLE.print(lastQerrNs);
        }
        else
        {
            CONSOLE.print(" | TIM-TP n/a");
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
