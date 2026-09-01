// gnss_ubx5.cpp
// Unified u-blox 5 stream splitter (NMEA + generic UBX) + NMEA parsers.

#include "gnss_ubx5.h"
#include <string.h>
#include <ctype.h>

// ---------------- Debug ----------------
static Stream* _dbg = nullptr;
static inline void _dbgPrintln(const char* s)
{
    if (_dbg != nullptr)
    {
        _dbg->println(s);
    }
}

// ---------------- NMEA buffering ----------------
static constexpr size_t _NMEA_MAX = 160;
static char             _nmeaBuf[_NMEA_MAX];
static size_t           _nmeaLen = 0;

static char   _lastRawNmea[_NMEA_MAX];
static size_t _lastRawNmeaLen = 0;
static bool   _hasNewNmea     = false;

// ---------------- UBX (generic) state machine ----------------
static const uint8_t _UBX_SYNC1 = 0xB5;
static const uint8_t _UBX_SYNC2 = 0x62;

enum _UbxState
{
    S_WAIT_SYNC1,
    S_WAIT_SYNC2,
    S_CLASS,
    S_ID,
    S_LEN1,
    S_LEN2,
    S_PAYLOAD,
    S_CK_A,
    S_CK_B
};

static Stream* _gnss = nullptr;

static _UbxState _uState = S_WAIT_SYNC1;
static uint8_t   _uClass = 0, _uId = 0;
static uint16_t  _uLen = 0, _uPos = 0;
static uint8_t   _uCkA = 0, _uCkB = 0;

// Generic payload room matches UbxFrame
static uint8_t _uPayload[sizeof(((UbxFrame*)0)->payload)];

// Latch: last UBX frame (consume-on-pop)
static UbxFrame _lastUbx    = {};
static bool     _haveNewUbx = false;

// ---------------- Common helpers ----------------
static uint8_t _hex2Nib(char c)
{
    if (c >= '0' && c <= '9')
    {
        return (uint8_t)(c - '0');
    }
    if (c >= 'A' && c <= 'F')
    {
        return (uint8_t)(c - 'A' + 10);
    }
    if (c >= 'a' && c <= 'f')
    {
        return (uint8_t)(c - 'a' + 10);
    }
    return 0;
}

bool nmeaChecksumOk(const char* line)
{
    if (line == nullptr || line[0] != '$')
    {
        return false;
    }
    const char* star = strrchr(line, '*');
    if (star == nullptr || star < line + 2)
    {
        return false;
    }
    if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]))
    {
        return false;
    }
    uint8_t cs = 0;
    for (const char* p = line + 1; p < star; ++p)
    {
        cs ^= (uint8_t)(*p);
    }
    uint8_t got = (uint8_t)((_hex2Nib(star[1]) << 4) | _hex2Nib(star[2]));
    if (cs == got)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void nmeaComputeChecksum(const char* payload, char outHH[2])
{
    uint8_t cs = 0;
    for (const char* p = payload; *p; ++p)
    {
        cs ^= (uint8_t)*p;
    }
    const char* chksumHex = "0123456789ABCDEF";
    outHH[0]              = chksumHex[(cs >> 4) & 0x0F];
    outHH[1]              = chksumHex[cs & 0x0F];
}

bool nmeaTruncateAtStar(char* line)
{
    char* star = strchr(line, '*');
    if (star == nullptr)
    {
        return false;
    }
    if (!isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2]))
    {
        return false;
    }
    star[3] = '\0';
    return true;
}

void gnssSendPubx40(const char* msg, bool enable)
{
    // $PUBX,40,<msg>,0,<on_off>,0,0,0,0*CS\r\n
    char payload[64];
    snprintf(payload, sizeof(payload), "PUBX,40,%s,0,%d,0,0,0,0", msg, enable ? 1 : 0);
    char cs[2];
    nmeaComputeChecksum(payload, cs);
    char line[80];
    snprintf(line, sizeof(line), "$%s*%c%c\r\n", payload, cs[0], cs[1]);
    _gnss->print(line);
}

// ---- UBX checksum helpers ----
static void _ckReset()
{
    _uCkA = 0;
    _uCkB = 0;
}
static void _ckAcc(uint8_t b)
{
    _uCkA = (uint8_t)(_uCkA + b);
    _uCkB = (uint8_t)(_uCkB + _uCkA);
}

static uint32_t _rdU4(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int32_t _rdI4(const uint8_t* p)
{
    return (int32_t)_rdU4(p);
}
static uint16_t _rdU2(const uint8_t* p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

// ---- CSV split preserving empties ----
static int _csvSplitPreserveEmpties(char* s, char** f, int maxf)
{
    int   count = 0;
    char* p     = s;
    char* start = s;
    while (*p != '\0' && count < maxf)
    {
        if (*p == ',')
        {
            *p         = '\0';
            f[count++] = start;
            start      = p + 1;
        }
        ++p;
    }
    if (count < maxf)
    {
        f[count++] = start;
    }
    return count;
}

// ---- ddmm.mmmm -> decimal degrees ----
static double _dmToDeg(const char* dm)
{
    if (dm == nullptr || *dm == '\0')
    {
        return NAN;
    }
    const char* dot       = strchr(dm, '.');
    int         intDigits = 0;
    if (dot != nullptr)
    {
        intDigits = (int)(dot - dm);
    }
    else
    {
        const char* p = dm;
        while (*p != '\0' && isdigit((unsigned char)*p))
        {
            ++p;
        }
        intDigits = (int)(p - dm);
    }
    if (intDigits < 3)
    {
        return NAN;
    }
    int degDigits = intDigits - 2;
    if (degDigits < 2)
    {
        degDigits = 2;
    }

    int deg = 0;
    for (int i = 0; i < degDigits; ++i)
    {
        if (!isdigit((unsigned char)dm[i]))
        {
            return NAN;
        }
        deg = deg * 10 + (dm[i] - '0');
    }
    const char* minStr  = dm + degDigits;
    double      minutes = atof(minStr);
    return deg + minutes / 60.0;
}

// ---------------- Public: stream split + pop ----------------
void gnssInit(Stream* gnssSerial, Stream* debugSerial)
{
    _gnss = gnssSerial;
    _dbg = debugSerial;

    _nmeaLen        = 0;
    _lastRawNmea[0] = '\0';
    _lastRawNmeaLen = 0;
    _hasNewNmea     = false;

    _uState = S_WAIT_SYNC1;
    _uClass = 0;
    _uId    = 0;
    _uLen   = 0;
    _uPos   = 0;
    _ckReset();
    _haveNewUbx = false;
    _lastUbx    = UbxFrame{};

    if (_dbg != nullptr)
    {
        _dbg->println("[gnss] init");
    }
}

uint32_t gnssReadSerial()
{
    uint32_t bytesRead = 0;

    while (_gnss->available())
    {
        uint8_t c = (uint8_t)_gnss->read();
        bytesRead++;
        // ---- UBX FSM ----
        switch (_uState)
        {
            case S_WAIT_SYNC1:
            {
                if (c == _UBX_SYNC1)
                {
                    _uState = S_WAIT_SYNC2;
                }
                break;
            }
            case S_WAIT_SYNC2:
            {
                if (c == _UBX_SYNC2)
                {
                    _uState = S_CLASS;
                    _ckReset();
                }
                else
                {
                    _uState = S_WAIT_SYNC1;
                }
                break;
            }
            case S_CLASS:
            {
                _uClass = c;
                _ckAcc(c);
                _uState = S_ID;
                break;
            }
            case S_ID:
            {
                _uId = c;
                _ckAcc(c);
                _uState = S_LEN1;
                break;
            }
            case S_LEN1:
            {
                _uLen = c;
                _ckAcc(c);
                _uState = S_LEN2;
                break;
            }
            case S_LEN2:
            {
                _uLen |= ((uint16_t)c << 8);
                _ckAcc(c);
                if (_uLen >= sizeof(_uPayload))
                {
                    if (_dbg != nullptr)
                    {
                        _dbg->println("[gnss] UBX oversize");
                    }
                    _uState = S_WAIT_SYNC1;
                }
                else
                {
                    _uPos   = 0;
                    _uState = S_PAYLOAD;
                }
                break;
            }
            case S_PAYLOAD:
            {
                _uPayload[_uPos++] = c;
                _ckAcc(c);
                if (_uPos >= _uLen)
                {
                    _uState = S_CK_A;
                }
                break;
            }
            case S_CK_A:
            {
                if (c == _uCkA)
                {
                    _uState = S_CK_B;
                }
                else
                {
                    if (_dbg != nullptr)
                    {
                        _dbg->println("[gnss] UBX ckA fail");
                    }
                    _uState = S_WAIT_SYNC1;
                }
                break;
            }
            case S_CK_B:
            {
                if (c == _uCkB)
                {
                    // latch + handlers
                    _lastUbx.cls = _uClass;
                    _lastUbx.id  = _uId;
                    _lastUbx.len = _uLen;
                    if (_lastUbx.len > sizeof(_lastUbx.payload))
                    {
                        _lastUbx.len = sizeof(_lastUbx.payload);
                    }
                    if (_lastUbx.len > 0)
                    {
                        memcpy(_lastUbx.payload, _uPayload, _lastUbx.len);
                    }
                    _haveNewUbx = true;
                }
                else
                {
                    if (_dbg != nullptr)
                    {
                        _dbg->println("[gnss] UBX ckB fail");
                    }
                }
                _uState = S_WAIT_SYNC1;
                break;
            }
        }

        // ---- NMEA line buffer ----
        if (c == '\r')
        {
            return bytesRead;
        }
        if (c == '\n')
        {
            if (_nmeaLen >= 7 && _nmeaBuf[0] == '$')
            {
                _nmeaBuf[_nmeaLen] = '\0';
                size_t n           = _nmeaLen;
                if (n >= _NMEA_MAX)
                {
                    n = _NMEA_MAX - 1;
                }
                memcpy(_lastRawNmea, _nmeaBuf, n);
                _lastRawNmea[n] = '\0';
                _lastRawNmeaLen = n;
                _hasNewNmea     = true;
            }
            _nmeaLen = 0;
            return bytesRead;
        }
        if (_nmeaLen < (_NMEA_MAX - 1))
        {
            _nmeaBuf[_nmeaLen++] = (char)c;
        }
        else
        {
            if (_dbg != nullptr)
            {
                _dbg->println("[gnss] NMEA overflow, drop line");
            }
            _nmeaLen = 0;
        }
    }

    return bytesRead;
}

bool gnssHasNewNmea()
{
    if (_hasNewNmea)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool gnssPopLastRawNmea(char* out, size_t outLen)
{
    if (!_hasNewNmea)
    {
        return false;
    }
    if (out == nullptr || outLen == 0)
    {
        return false;
    }
    size_t n = _lastRawNmeaLen;
    if (n >= outLen)
    {
        n = outLen - 1;
    }
    memcpy(out, _lastRawNmea, n);
    out[n]      = '\0';
    _hasNewNmea = false;
    return true;
}

bool gnssHasNewUbx()
{
    if (_haveNewUbx)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool gnssPopLastUbx(UbxFrame& out)
{
    if (!_haveNewUbx)
    {
        return false;
    }
    out         = _lastUbx;
    _haveNewUbx = false;
    return true;
}

bool gnssClassifyUbx(const UbxFrame& fr, UbxType& outType)
{
    outType = UbxType::Unknown;

    // TIM (0x0D)
    if (fr.cls == 0x0D)
    {
        if (fr.id == 0x01)
        {
            outType = UbxType::TIM_TP;
            return true;
        }
        if (fr.id == 0x03)
        {
            outType = UbxType::TIM_TM2;
            return true;
        }
        if (fr.id == 0x04)
        {
            outType = UbxType::TIM_SVIN;
            return true;
        }
        return false;
    }

    // NAV (0x01)
    if (fr.cls == 0x01)
    {
        if (fr.id == 0x20)
        {
            outType = UbxType::NAV_TIMEGPS;
            return true;
        }
        if (fr.id == 0x21)
        {
            outType = UbxType::NAV_TIMEUTC;
            return true;
        }
        if (fr.id == 0x21)
        {
            outType = UbxType::NAV_TIMEUTC;
            return true;
        }

        return false;
    }

    // MON (0x0A)
    if (fr.cls == 0x0A)
    {
        if (fr.id == 0x04)
        {
            outType = UbxType::MON_VER;
            return true;
        }
        if (fr.id == 0x09)
        {
            outType = UbxType::MON_HW;
            return true;
        }
        return false;
    }

    // CFG (0x06)
    if (fr.cls == 0x06)
    {
        if (fr.id == 0x01)
        {
            outType = UbxType::CFG_MSG;
            return true;
        }
        if (fr.id == 0x1D)
        {
            outType = UbxType::CFG_TMODE;
            return true;
        }
        return false;
    }

    // Unknown class/id
    return false;
}

// ---------------- NMEA helpers (public) ----------------
bool gnssClassifyNmea(const char* line, NmeaType& outType)
{
    if (line == nullptr || line[0] != '$')
    {
        outType = NmeaType::Unknown;
        return false;
    }
    // We only need the talker+type (e.g., "GPGGA" or "GNGSA")
    if (!strncmp(line, "$GPGGA", 6) || !strncmp(line, "$GNGGA", 6))
    {
        outType = NmeaType::GGA;
        return true;
    }
    else if (!strncmp(line, "$GPRMC", 6) || !strncmp(line, "$GNRMC", 6))
    {
        outType = NmeaType::RMC;
        return true;
    }
    else if (!strncmp(line, "$GPGSA", 6) || !strncmp(line, "$GNGSA", 6))
    {
        outType = NmeaType::GSA;
        return true;
    }
    else if (!strncmp(line, "$GPGSV", 6) || !strncmp(line, "$GNGSV", 6))
    {
        outType = NmeaType::GSV;
        return true;
    }
    else
    {
        outType = NmeaType::Unknown;
        return true;
    }
}

// Each parser modifies the input line (to NUL-terminate at '*') and validates checksum.

bool gnssParseGga(char* line, NmeaGga& out)
{
    if (!nmeaTruncateAtStar(line))
    {
        return false;
    }
    if (!nmeaChecksumOk(line))
    {
        return false;
    }
    char* star = strrchr(line, '*');
    if (star != nullptr)
    {
        *star = '\0';
    }

    const int MAXF = 24;
    char*     f[MAXF];
    int       nf = _csvSplitPreserveEmpties(line, f, MAXF);
    if (nf < 7)
    {
        return false;
    }

    out = NmeaGga{};
    if (nf > 1 && f[1][0] != '\0')
    {
        out.utcHhmmss = atof(f[1]);
    }
    if (nf > 2)
    {
        out.latDeg = _dmToDeg(f[2]);
    }
    if (nf > 3 && f[3][0] == 'S')
    {
        if (!isnan(out.latDeg))
        {
            out.latDeg = -out.latDeg;
        }
    }
    if (nf > 4)
    {
        out.lonDeg = _dmToDeg(f[4]);
    }
    if (nf > 5 && f[5][0] == 'W')
    {
        if (!isnan(out.lonDeg))
        {
            out.lonDeg = -out.lonDeg;
        }
    }
    if (nf > 6 && f[6][0] != '\0')
    {
        out.fixQ = atoi(f[6]);
    }
    if (nf > 7 && f[7][0] != '\0')
    {
        out.sats = atoi(f[7]);
    }
    if (nf > 8 && f[8][0] != '\0')
    {
        out.hdop = atof(f[8]);
    }
    if (nf > 9 && f[9][0] != '\0')
    {
        out.altM = atof(f[9]);
    }
    if (nf > 11 && f[11][0] != '\0')
    {
        out.geoidSepM = atof(f[11]);
    }
    if (nf > 13 && f[13][0] != '\0')
    {
        out.dgpsAgeS = atof(f[13]);
    }

    out.valid = (out.fixQ > 0);
    return true;
}

bool gnssParseRmc(char* line, NmeaRmc& out)
{
    if (!nmeaTruncateAtStar(line))
    {
        return false;
    }
    if (!nmeaChecksumOk(line))
    {
        return false;
    }
    char* star = strrchr(line, '*');
    if (star != nullptr)
    {
        *star = '\0';
    }

    const int MAXF = 20;
    char*     f[MAXF];
    int       nf = _csvSplitPreserveEmpties(line, f, MAXF);
    if (nf < 2)
    {
        return false;
    }

    out = NmeaRmc{};
    if (nf > 1 && f[1][0] != '\0')
    {
        out.utcHhmmss = atof(f[1]);
    }
    if (nf > 2 && f[2][0] != '\0')
    {
        out.status = f[2][0];
    }
    if (nf > 3)
    {
        out.latDeg = _dmToDeg(f[3]);
    }
    if (nf > 4 && f[4][0] == 'S')
    {
        if (!isnan(out.latDeg))
        {
            out.latDeg = -out.latDeg;
        }
    }
    if (nf > 5)
    {
        out.lonDeg = _dmToDeg(f[5]);
    }
    if (nf > 6 && f[6][0] == 'W')
    {
        if (!isnan(out.lonDeg))
        {
            out.lonDeg = -out.lonDeg;
        }
    }
    if (nf > 7 && f[7][0] != '\0')
    {
        out.sogKn = atof(f[7]);
    }
    if (nf > 8 && f[8][0] != '\0')
    {
        out.cogDeg = atof(f[8]);
    }
    if (nf > 9 && f[9][0] != '\0')
    {
        out.dateDdMmYy = atoi(f[9]);
    }

    out.valid = (out.status == 'A');
    return true;
}

bool gnssParseGsa(char* line, NmeaGsa& out)
{
    if (!nmeaTruncateAtStar(line))
    {
        return false;
    }
    if (!nmeaChecksumOk(line))
    {
        return false;
    }
    char* star = strrchr(line, '*');
    if (star != nullptr)
    {
        *star = '\0';
    }

    const int MAXF = 40;
    char*     f[MAXF];
    int       nf = _csvSplitPreserveEmpties(line, f, MAXF);
    if (nf < 3)
    {
        return false;
    }

    out = NmeaGsa{};
    if (nf > 1 && f[1][0] != '\0')
    {
        out.mode = f[1][0];
    }
    if (nf > 2 && f[2][0] != '\0')
    {
        out.fixType = atoi(f[2]);
    }

    out.used = 0;
    for (int i = 0; i < 12; ++i)
    {
        int idx = 3 + i;
        if (idx < nf && f[idx][0] != '\0')
        {
            out.prn[i] = atoi(f[idx]);
            ++out.used;
        }
        else
        {
            out.prn[i] = 0;
        }
    }

    if (nf > 15 && f[15][0] != '\0')
    {
        out.pdop = atof(f[15]);
    }
    if (nf > 16 && f[16][0] != '\0')
    {
        out.hdop = atof(f[16]);
    }
    if (nf > 17 && f[17][0] != '\0')
    {
        out.vdop = atof(f[17]);
    }

    out.valid = true;
    return true;
}

bool gnssParseGsv(char* line, NmeaGsv& out)
{
    if (!nmeaTruncateAtStar(line))
    {
        return false;
    }
    if (!nmeaChecksumOk(line))
    {
        return false;
    }
    char* star = strrchr(line, '*');
    if (star != nullptr)
    {
        *star = '\0';
    }

    const int MAXF = 48;
    char*     f[MAXF];
    int       nf = _csvSplitPreserveEmpties(line, f, MAXF);
    if (nf < 4)
    {
        return false;
    }

    out          = NmeaGsv{};
    out.totalMsg = (f[1][0] != '\0') ? atoi(f[1]) : 0;
    out.msgNo    = (f[2][0] != '\0') ? atoi(f[2]) : 0;
    out.inView   = (f[3][0] != '\0') ? atoi(f[3]) : -1;

    out.satsCount = 0;
    for (int i = 4; i + 3 < nf && out.satsCount < 4; i += 4)
    {
        if (f[i][0] == '\0')
        {
            continue;
        }
        SatInfo si;
        si.prn                    = atoi(f[i]);
        si.elev                   = (f[i + 1][0] != '\0') ? atoi(f[i + 1]) : -1;
        si.az                     = (f[i + 2][0] != '\0') ? atoi(f[i + 2]) : -1;
        si.snr                    = (f[i + 3][0] != '\0') ? atoi(f[i + 3]) : -1;
        out.sats[out.satsCount++] = si;
    }

    out.valid = true;
    return true;
}

// ---------------- UBX decoders (public) ----------------
bool gnssDecodeTimTp(const UbxFrame& fr, UbxTimTp& outTp)
{
    // TIM-TP: class 0x0D, id 0x01, payload 16 bytes on u-blox 5
    if (fr.cls != 0x0D || fr.id != 0x01 || fr.len != 16)
    {
        return false;
    }
    const uint8_t* p = fr.payload;
    outTp.towMs      = _rdU4(&p[0]);
    outTp.towSubMs   = _rdU4(&p[4]);
    outTp.qErrNs     = _rdI4(&p[8]) / 1000.0;  // ns on u-blox 5
    outTp.week       = _rdU2(&p[12]);
    outTp.flags      = p[14];
    outTp.refInfo    = p[15];
    outTp.valid      = true;
    return true;
}

bool gnssDecodeTimSvin(const UbxFrame& fr, UbxTimSvin& out)
{
    // TIM-SVIN: class 0x0D, id 0x04, payload 28 bytes (u-blox 5/6)
    if (fr.cls != 0x0D || fr.id != 0x04 || fr.len != 28)
    {
        return false;
    }
    const uint8_t* p = fr.payload;
    out.durSec       = _rdU4(&p[0]);  // dur (s)
    out.meanX_cm     = (int32_t)_rdU4(&p[4]);
    out.meanY_cm     = (int32_t)_rdU4(&p[8]);
    out.meanZ_cm     = (int32_t)_rdU4(&p[12]);
    out.meanV_mm2    = _rdU4(&p[16]);  // variance
    out.obs          = _rdU4(&p[20]);  // observations
    out.valid        = (p[24] != 0);
    out.active       = (p[25] != 0);
    return true;
}

bool gnssDecodeNavTimeUtc(const UbxFrame& fr, UbxNavTimeUtc& out)
{
    if (fr.cls != 0x01 || fr.id != 0x21 || fr.len != 20)
    {
        return false;
    }
    const uint8_t* p = fr.payload;

    auto rdU4 = [&](int off) -> uint32_t
    {
        return (uint32_t)p[off + 0] | ((uint32_t)p[off + 1] << 8) | ((uint32_t)p[off + 2] << 16)
               | ((uint32_t)p[off + 3] << 24);
    };
    auto rdI4 = [&](int off) -> int32_t { return (int32_t)rdU4(off); };
    auto rdU2 = [&](int off) -> uint16_t { return (uint16_t)p[off + 0] | ((uint16_t)p[off + 1] << 8); };

    out.iTowMs = rdU4(0);  // iTOW
    out.tAccNs = rdU4(4);  // tAcc (UTC)
    out.nanoNs = rdI4(8);  // nano (can be negative)
    out.year   = rdU2(12);
    out.month  = p[14];
    out.day    = p[15];
    out.hour   = p[16];
    out.min    = p[17];
    out.sec    = p[18];
    out.valid  = p[19];
    return true;
}

bool gnssDecodeCfgTmode(const UbxFrame& fr, UbxCfgTmode& out)
{
    if (fr.cls != 0x06 || fr.id != 0x1D || fr.len != 28)
    {
        return false;
    }
    const uint8_t* p = fr.payload;

    auto rdU4 = [&](int off) -> uint32_t
    {
        return (uint32_t)p[off + 0] | ((uint32_t)p[off + 1] << 8) | ((uint32_t)p[off + 2] << 16)
               | ((uint32_t)p[off + 3] << 24);
    };
    auto rdI4 = [&](int off) -> int32_t { return (int32_t)rdU4(off); };

    out.timeMode         = rdU4(0);
    out.fixedX_cm        = rdI4(4);
    out.fixedY_cm        = rdI4(8);
    out.fixedZ_cm        = rdI4(12);
    out.fixedVar_mm2     = rdU4(16);
    out.svinMinDur_s     = rdU4(20);
    out.svinVarLimit_mm2 = rdU4(24);
    return true;
}

// ---------------- UBX config (public) ----------------
static void _sendUbxFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    auto    acc = [&](uint8_t b)
    {
        ckA = (uint8_t)(ckA + b);
        ckB = (uint8_t)(ckB + ckA);
    };

    _gnss->write(0xB5);
    _gnss->write(0x62);
    _gnss->write(cls);
    acc(cls);
    _gnss->write(id);
    acc(id);
    _gnss->write((uint8_t)(len & 0xFF));
    acc((uint8_t)(len & 0xFF));
    _gnss->write((uint8_t)(len >> 8));
    acc((uint8_t)(len >> 8));
    for (uint16_t i = 0; i < len; ++i)
    {
        _gnss->write(payload[i]);
        acc(payload[i]);
    }
    _gnss->write(ckA);
    _gnss->write(ckB);
}

static void _sendCfgTmode(uint32_t timeMode,
                          int32_t  x_cm,
                          int32_t  y_cm,
                          int32_t  z_cm,
                          uint32_t fixedPosVar_mm2,
                          uint32_t svinMinDur_s,
                          uint32_t svinVarLimit_mm2)
{
    uint8_t p[28];
    auto    wrU4 = [&](int off, uint32_t v)
    {
        p[off + 0] = (uint8_t)(v);
        p[off + 1] = (uint8_t)(v >> 8);
        p[off + 2] = (uint8_t)(v >> 16);
        p[off + 3] = (uint8_t)(v >> 24);
    };
    auto wrI4 = [&](int off, int32_t v) { wrU4(off, (uint32_t)v); };

    wrU4(0, timeMode);           // 0=Disabled, 1=Survey-In, 2=Fixed
    wrI4(4, x_cm);               // fixedPosX (cm)
    wrI4(8, y_cm);               // fixedPosY (cm)
    wrI4(12, z_cm);              // fixedPosZ (cm)
    wrU4(16, fixedPosVar_mm2);   // fixed position 3D variance (mm^2)
    wrU4(20, svinMinDur_s);      // survey-in minimum duration (s)
    wrU4(24, svinVarLimit_mm2);  // survey-in variance limit (mm^2)

    _sendUbxFrame(0x06, 0x1D, p, sizeof(p));
}

void gnssSetBaudRate(uint8_t portId, uint32_t baudRate)
{
    // UBX-CFG-PRT (Class 0x06, ID 0x00, Length 20)
    uint8_t p[20] = {0};
    p[0] = portId;  // 1 = UART1
    // mode: 8N1 (0x000008D0)
    p[4] = 0xD0;
    p[5] = 0x08;
    p[6] = 0x00;
    p[7] = 0x00;
    // baudRate (4 bytes little-endian)
    p[8]  = (uint8_t)(baudRate & 0xFF);
    p[9]  = (uint8_t)((baudRate >> 8) & 0xFF);
    p[10] = (uint8_t)((baudRate >> 16) & 0xFF);
    p[11] = (uint8_t)((baudRate >> 24) & 0xFF);
    // inProtoMask: 0x0007 (UBX + NMEA + RTCM)
    p[12] = 0x07;
    p[13] = 0x00;
    // outProtoMask: 0x0003 (UBX + NMEA)
    p[14] = 0x03;
    p[15] = 0x00;

    _sendUbxFrame(0x06, 0x00, p, sizeof(p));
}

void gnssSendUbxCfgMsg(uint8_t cls, uint8_t id, uint8_t targetPortId, uint8_t rate)
{
    // u-blox 5: payload = [cls id rateI2C rateUART1 rateUART2 rateUSB rateSPI rateReserved(0)]
    uint8_t p[8] = {cls, id, 0, 0, 0, 0, 0, 0};
    if (targetPortId <= 4)
    {
        p[2 + targetPortId] = rate;
    }
    _sendUbxFrame(0x06, 0x01, p, sizeof(p));  // CFG-MSG
}

void gnssEnableSurveyIn(uint32_t minDurSec, uint32_t varLimit_mm2)
{
    _sendCfgTmode(/*Survey-In*/ 1, 0, 0, 0, 0, minDurSec, varLimit_mm2);
}

void gnssSetFixedPositionECEF(int32_t x_cm, int32_t y_cm, int32_t z_cm, uint32_t posVar_mm2)
{
    _sendCfgTmode(/*Fixed*/ 2, x_cm, y_cm, z_cm, posVar_mm2, 0, 0);
}

void gnssDisableTimeMode()
{
    _sendCfgTmode(/*Disabled*/ 0, 0, 0, 0, 0, 0, 0);
}

void gnssPollCfgTmode()
{
    // Empty payload poll → device replies with CFG-TMODE containing current settings/state
    _sendUbxFrame(0x06, 0x1D, nullptr, 0);
}