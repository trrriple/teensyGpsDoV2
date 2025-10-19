#pragma once
#include <Arduino.h>

// ===== Unified u-blox 5 stream splitter (NMEA + generic UBX) =====
// The library splits the stream AND provides parsing helpers for common messages.
// Style: camelCase; braces on all control statements; static internals prefixed with '_'.

// ---------------- UBX generic frame ----------------
struct UbxFrame
{
    uint8_t  cls = 0;
    uint8_t  id  = 0;
    uint16_t len = 0;
    uint8_t  payload[128];  // enough for common NAV/MON/TIM packets on u-blox 5
};

// ---------------- UBX classifier ----------------
enum class UbxType : uint8_t {
  Unknown = 0,
  // TIM
  TIM_TP,        // 0x0D 0x01
  TIM_TM2,       // 0x0D 0x03
  TIM_SVIN,      // 0x0D 0x04
  // NAV
  NAV_TIMEGPS,   // 0x01 0x20
  NAV_TIMEUTC,   // 0x01 0x21
  // MON
  MON_HW,        // 0x0A 0x09
  MON_VER,       // 0x0A 0x04
  // CFG
  CFG_MSG,        // 0x06 0x01
  CFG_TMODE,      // 0x06 0x1D

};

struct UbxTimTp
{
    uint32_t towMs    = 0;
    uint32_t towSubMs = 0;
    float    qErrNs   = 0;  // u-blox 5: nanoseconds
    uint16_t week     = 0;
    uint8_t  flags    = 0;
    uint8_t  refInfo  = 0;
    bool     valid    = false;
};

struct UbxTimSvin
{
    uint32_t durSec    = 0;  // elapsed survey time (s)
    int32_t  meanX_cm  = 0;  // ECEF X (cm)
    int32_t  meanY_cm  = 0;  // ECEF Y (cm)
    int32_t  meanZ_cm  = 0;  // ECEF Z (cm)
    uint32_t meanV_mm2 = 0;  // variance (mm^2)
    uint32_t obs       = 0;  // observations used
    bool     valid     = false;
    bool     active    = false;
};

// UBX-NAV-TIMEUTC decode target (payload len = 20 on u-blox 5/7)
struct UbxNavTimeUtc
{
    uint32_t iTowMs = 0;  // GPS time of week of nav epoch (ms)
    uint32_t tAccNs = 0;  // time accuracy estimate (ns, UTC)
    int32_t  nanoNs = 0;  // fractional second (can be negative)
    uint16_t year   = 0;  // 1999..2099
    uint8_t  month  = 0;  // 1..12
    uint8_t  day    = 0;  // 1..31
    uint8_t  hour   = 0;  // 0..23
    uint8_t  min    = 0;  // 0..59
    uint8_t  sec    = 0;  // 0..60 (leap second)
    uint8_t  valid  = 0;  // bitfield: bit0=validTOW, bit1=validWKN, bit2=validUTC
};

// CFG-TMODE (u-blox 5) decode target (payload len = 28)
struct UbxCfgTmode
{
    uint32_t timeMode         = 0;  // 0=Disabled, 1=Survey-In, 2=Fixed
    int32_t  fixedX_cm        = 0;  // ECEF X (cm)
    int32_t  fixedY_cm        = 0;  // ECEF Y (cm)
    int32_t  fixedZ_cm        = 0;  // ECEF Z (cm)
    uint32_t fixedVar_mm2     = 0;  // fixed position 3D variance (mm^2)
    uint32_t svinMinDur_s     = 0;  // survey-in min duration (s)
    uint32_t svinVarLimit_mm2 = 0;  // survey-in variance limit (mm^2)
};

// ---------------- NMEA message structs ----------------
struct SatInfo
{
    int prn  = -1;
    int elev = -1;
    int az   = -1;
    int snr  = -1;
};

enum class NmeaType : uint8_t
{
    Unknown = 0,
    GGA,
    RMC,
    GSA,
    GSV
};

struct NmeaGga
{
    bool   valid     = false;
    double utcHhmmss = NAN;  // raw hhmmss.sss (UTC)
    double latDeg    = NAN;
    double lonDeg    = NAN;
    int    fixQ      = 0;  // 0.. for no/fix/DGPS/etc.
    int    sats      = -1;
    double hdop      = NAN;
    double altM      = NAN;  // MSL altitude
    double geoidSepM = NAN;
    double dgpsAgeS  = NAN;
};

struct NmeaRmc
{
    bool   valid      = false;
    double utcHhmmss  = NAN;
    char   status     = 'V';  // A=valid, V=void
    double latDeg     = NAN;
    double lonDeg     = NAN;
    double sogKn      = NAN;
    double cogDeg     = NAN;
    int    dateDdMmYy = -1;  // ddmmyy
};

struct NmeaGsa
{
    bool   valid   = false;
    char   mode    = 'A';  // A=Auto, M=Manual
    int    fixType = 1;    // 1=no, 2=2D, 3=3D
    int    prn[12] = {0};
    int    used    = 0;
    double pdop    = NAN;
    double hdop    = NAN;
    double vdop    = NAN;
};

struct NmeaGsv
{
    bool    valid    = false;
    int     totalMsg = 0;
    int     msgNo    = 0;
    int     inView   = -1;
    SatInfo sats[4];
    int     satsCount = 0;  // 0..4 in this sentence
};

// ---------- Public: stream split + pop ----------
void gnssInit(Stream* gnssSerial, Stream* debugSerial);
uint32_t gnssReadSerial();


// ---------- Public: UBX helpers (parsers live in the lib) ----------
// UBX generic pop (consume-on-read). Returns true if a new frame was copied.
bool gnssPopLastUbx(UbxFrame& out);
bool gnssHasNewUbx();

// Returns true if recognized (outType != Unknown), false otherwise.
bool gnssClassifyUbx(const UbxFrame& fr, UbxType& outType);

// ---------- Public: NMEA helpers (parsers live in the lib) ----------
// NMEA sentence pop (no CR/LF, includes "*HH")
// Returns true if a new sentence was copied (and consumed).
bool gnssPopLastRawNmea(char* out, size_t outLen);
bool gnssHasNewNmea();

bool gnssClassifyNmea(const char* line, NmeaType& outType);  // quick "$GPxxx" classifier

// Each parser expects a modifiable C-string (it will NUL-terminate at '*').
// Returns true if checksum is OK and fields parsed.
bool gnssParseGga(char* line, NmeaGga& out);
bool gnssParseRmc(char* line, NmeaRmc& out);
bool gnssParseGsa(char* line, NmeaGsa& out);
bool gnssParseGsv(char* line, NmeaGsv& out);

// ---------- Public: UBX decoders ----------
bool gnssDecodeTimTp(const UbxFrame& fr, UbxTimTp& outTp);
bool gnssDecodeTimSvin(const UbxFrame& fr, UbxTimSvin& outSvin);
bool gnssDecodeNavTimeUtc(const UbxFrame& fr, UbxNavTimeUtc& out);
bool gnssDecodeCfgTmode(const UbxFrame& fr, UbxCfgTmode& out);


// ---------- Public: message control ----------
void gnssSendPubx40(const char* msg, bool enable);                                    // $PUBX,40,<msg>,...
void gnssSendUbxCfgMsg(uint8_t cls, uint8_t id, uint8_t targetPortId, uint8_t rate);  // UBX-CFG-MSG
void gnssEnableSurveyIn(uint32_t minDurSec, uint32_t varLimit_mm2);

void gnssSetFixedPositionECEF(int32_t x_cm, int32_t y_cm, int32_t z_cm, uint32_t posVar_mm2);
void gnssDisableTimeMode();
void gnssPollCfgTmode();   // ask current TMODE settings


// ---------- Public: checksum helpers ----------
bool nmeaChecksumOk(const char* line);  // "$...*HH"
void nmeaComputeChecksum(const char* payload, char outHH[2]);
bool nmeaTruncateAtStar(char* line);  // keeps "*HH", terminates after it
