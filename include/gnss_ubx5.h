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
  // CFG (classify-only; typically not “decoded”)
  CFG_MSG        // 0x06 0x01
};

struct UbxTimTp
{
    uint32_t towMs    = 0;
    uint32_t towSubMs = 0;
    int32_t  qErrNs   = 0;  // u-blox 5: nanoseconds
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
void gnssInit(Stream* debugSerial = nullptr);
void gnssFeedByte(uint8_t c);


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

// ---------- Public: message control ----------
void gnssSendPubx40(Stream& s, const char* msg, bool enable);                                    // $PUBX,40,<msg>,...
void gnssSendUbxCfgMsg(Stream& s, uint8_t cls, uint8_t id, uint8_t targetPortId, uint8_t rate);  // UBX-CFG-MSG
void gnssEnableSurveyIn(Stream& s, uint32_t minDurSec, uint32_t varLimit_mm2);
void gnssSetFixedPositionECEF(Stream& s, int32_t x_cm, int32_t y_cm, int32_t z_cm, uint32_t posVar_mm2);
void gnssDisableTimeMode(Stream& s);
void gnssPollCfgTmode(Stream& s);   // ask current TMODE settings


// ---------- Public: checksum helpers ----------
bool nmeaChecksumOk(const char* line);  // "$...*HH"
void nmeaComputeChecksum(const char* payload, char outHH[2]);
bool nmeaTruncateAtStar(char* line);  // keeps "*HH", terminates after it
