#pragma once

#include <Arduino.h>

namespace ZeetaAcomms16
{

// ======================================================
// Protocol constants
// ======================================================

static constexpr uint8_t PROTOCOL_VERSION_MAJOR = 1;
static constexpr uint8_t PROTOCOL_VERSION_MINOR = 0;

static constexpr uint8_t FUNCTION_BITS = 4;
static constexpr uint8_t FUSEID_BITS   = 2;
static constexpr uint8_t PAYLOAD_BITS  = 10;

static constexpr uint16_t PAYLOAD_MAX  = 1023;


// ======================================================
// Acoustic Function IDs
// ======================================================

enum class Function : uint8_t
{
    CAL_DEPTH       = 0x0,
    CAL_TEMP        = 0x1,
    MAG_EVENT       = 0x2,
    ACOUSTIC_EVENT  = 0x3,
    STATUS          = 0x4,
    DIAGNOSTIC      = 0x5,
    ACK             = 0x6,
    COMMAND         = 0x7,
    HEARTBEAT       = 0x8,
    CONFIG          = 0x9,
    RESERVED_A      = 0xA,
    RESERVED_B      = 0xB,
    RESERVED_C      = 0xC,
    RESERVED_D      = 0xD,
    RESERVED_E      = 0xE,
    RESERVED_F      = 0xF
};


// ======================================================
// Fuse IDs
// ======================================================

enum class FuseId : uint8_t
{
    FUZE_1 = 0,
    FUZE_2 = 1,
    FUZE_3 = 2,
    FUZE_4 = 3
};


// ======================================================
// Status Modes
// ======================================================

enum class StatusMode : uint8_t
{
    IDLE        = 0,
    INIT        = 1,
    CALIBRATION = 2,
    ACTIVE      = 3,
    RX_WINDOW   = 4,
    RECALL      = 5,
    ERROR_MODE  = 6,
    RESERVED    = 7
};


// ======================================================
// Command IDs
// ======================================================

enum class CommandId : uint8_t
{
    NO_OP               = 0x0,
    ENTER_CAL           = 0x1,
    ENTER_ACTIVE        = 0x2,
    START_RECORD        = 0x3,
    STOP_RECORD         = 0x4,
    RECALL_ON           = 0x5,
    RECALL_OFF          = 0x6,
    REQUEST_STATUS      = 0x7,
    REQUEST_DIAGNOSTIC  = 0x8,
    SET_MAG_THRESHOLD   = 0x9,
    SET_ACOUSTIC_THRESH = 0xA,
    LEDS_ON             = 0xB,
    LEDS_OFF            = 0xC,
    STROBE_ON           = 0xD,
    STROBE_OFF          = 0xE,
    REBOOT              = 0xF
};


// ======================================================
// ACK Codes
// ======================================================

enum class AckCode : uint8_t
{
    OK                = 0x0,
    INVALID_COMMAND   = 0x1,
    INVALID_ARGUMENT  = 0x2,
    BUSY              = 0x3,
    UNSUPPORTED       = 0x4,
    SENSOR_FAILURE    = 0x5,
    STORAGE_FAILURE   = 0x6,
    MODE_REJECTED     = 0x7,
    TIMEOUT           = 0x8,
    RESERVED_9        = 0x9,
    RESERVED_A        = 0xA,
    RESERVED_B        = 0xB,
    RESERVED_C        = 0xC,
    RESERVED_D        = 0xD,
    RESERVED_E        = 0xE,
    RESERVED_F        = 0xF
};


// ======================================================
// Status Flags (7 bits)
// ======================================================

enum StatusFlags : uint8_t
{
    STATUS_LOGGING_ACTIVE = 1u << 0,
    STATUS_LEAK_DETECTED  = 1u << 1,
    STATUS_STROBE_ACTIVE  = 1u << 2,
    STATUS_SD_READY       = 1u << 3,
    STATUS_MODEM_READY    = 1u << 4,
    STATUS_EVENT_LATCHED  = 1u << 5,
    STATUS_FAULT_PRESENT  = 1u << 6
};


// ======================================================
// Diagnostic Flags (10 bits)
// ======================================================

enum DiagnosticBits : uint16_t
{
    DIAG_LEAK_DETECTED  = 1u << 0,
    DIAG_SD_FAILURE     = 1u << 1,
    DIAG_PRESSURE_FAIL  = 1u << 2,
    DIAG_HYDRO_FAIL     = 1u << 3,
    DIAG_MAG_FAIL       = 1u << 4,
    DIAG_MODEM_FAIL     = 1u << 5,
    DIAG_LOW_BATTERY    = 1u << 6,
    DIAG_CAL_INVALID    = 1u << 7,
    DIAG_TIMING_FAULT   = 1u << 8,
    DIAG_RESERVED       = 1u << 9
};


// ======================================================
// Raw packet container
// ======================================================

struct Packet16
{
    uint16_t raw = 0;
};


// ======================================================
// Decoded packet
// ======================================================

struct DecodedPacket
{
    bool valid = false;

    Function function = Function::RESERVED_F;
    FuseId fuseId = FuseId::FUZE_1;
    uint16_t payload10 = 0;

    float depth_m = 0.0f;
    float temp_c = 0.0f;
    float mag_uT = 0.0f;
    float acoustic_dB = 0.0f;

    StatusMode statusMode = StatusMode::IDLE;
    uint8_t statusFlags = 0;
    uint16_t diagnosticBits = 0;

    AckCode ackCode = AckCode::OK;
    uint8_t ackReference = 0;

    CommandId commandId = CommandId::NO_OP;
    uint8_t commandArg = 0;
};


// ======================================================
// Core packet functions
// ======================================================

uint16_t encodePacket(Function fn, FuseId fuseId, uint16_t payload10);
DecodedPacket decodePacket(uint16_t raw);

Function getFunction(uint16_t raw);
FuseId getFuseId(uint16_t raw);
uint16_t getPayload(uint16_t raw);


// ======================================================
// Typed encoding helpers
// ======================================================

uint16_t encodeDepth(FuseId id, float depth_m);
uint16_t encodeTemperatureC(FuseId id, float temp_c);
uint16_t encodeMagEvent(FuseId id, float magnitude_uT);
uint16_t encodeAcousticEvent(FuseId id, float anomaly_dB);

uint16_t encodeStatus(FuseId id, StatusMode mode, uint8_t flags);
uint16_t encodeDiagnostic(FuseId id, uint16_t diagBits);

uint16_t encodeAck(FuseId id, AckCode code, uint8_t reference);
uint16_t encodeCommand(FuseId id, CommandId cmd, uint8_t arg);

uint16_t encodeHeartbeat(FuseId id, StatusMode mode, uint8_t info);
uint16_t encodeConfig(FuseId id, uint8_t key, uint8_t value);


// ======================================================
// Scaling helpers
// ======================================================

uint16_t encodeDepthPayload(float depth_m);
uint16_t encodeTemperaturePayload(float temp_c);
uint16_t encodeMagPayload(float magnitude_uT);
uint16_t encodeAcousticPayload(float anomaly_dB);

float decodeDepthPayload(uint16_t payload);
float decodeTemperaturePayload(uint16_t payload);
float decodeMagPayload(uint16_t payload);
float decodeAcousticPayload(uint16_t payload);


// ======================================================
// Debug helpers
// ======================================================

const char* functionName(Function fn);
const char* commandName(CommandId cmd);
const char* ackName(AckCode code);
const char* statusModeName(StatusMode mode);


} // namespace ZeetaAcomms16
