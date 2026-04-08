#include <Arduino.h>
#include "ZeetaAcomms16.h"

namespace ZeetaRfLink
{

// ======================================================
// RF protocol constants
// ======================================================

static constexpr uint8_t SOF = 0xA5;
static constexpr uint8_t PROTOCOL_VERSION = 0x01;

static constexpr uint8_t HEADER_SIZE = 8;
static constexpr uint8_t CRC_SIZE = 2;

static constexpr size_t MAX_PAYLOAD_SIZE = 64;
static constexpr size_t MAX_FRAME_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;


// ======================================================
// RF Message Types
// ======================================================

enum class MessageType : uint8_t
{
    HEARTBEAT       = 0x01,
    TELEMETRY_DELTA = 0x02,
    EVENT           = 0x03,
    COMMAND         = 0x04,
    ACK             = 0x05,
    DIAGNOSTIC      = 0x06,
    STATUS_SNAPSHOT = 0x07,
    SATCOM_STATUS   = 0x08,
    SATCOM_COMMAND  = 0x09,
    PING            = 0x0A
};


// ======================================================
// Node IDs
// ======================================================

enum class NodeId : uint8_t
{
    UNKNOWN         = 0x00,
    BUOY_1          = 0x10,
    BUOY_2          = 0x11,
    RF_CONTROLLER_1 = 0x20,
    SATCOM_MODULE   = 0x30,
    BROADCAST       = 0xFF
};


// ======================================================
// Target Types
// ======================================================

enum class TargetType : uint8_t
{
    BUOY      = 0x01,
    FUZE      = 0x02,
    SATCOM    = 0x03,
    BROADCAST = 0x04
};


// ======================================================
// RF ACK Codes
// ======================================================

enum class RfAckCode : uint8_t
{
    OK              = 0x00,
    INVALID_COMMAND = 0x01,
    INVALID_ARG     = 0x02,
    BUSY            = 0x03,
    FORWARD_FAILED  = 0x04,
    TIMEOUT         = 0x05,
    UNSUPPORTED     = 0x06
};


// ======================================================
// Core frame structures
// ======================================================

struct Frame
{
    uint8_t data[MAX_FRAME_SIZE];
    size_t length = 0;
};

struct Header
{
    uint8_t sof = SOF;
    uint8_t version = PROTOCOL_VERSION;
    MessageType type = MessageType::HEARTBEAT;
    NodeId src = NodeId::UNKNOWN;
    NodeId dst = NodeId::UNKNOWN;
    uint16_t sequence = 0;
    uint8_t payloadLength = 0;
};

struct DecodedFrame
{
    bool valid = false;
    Header header{};

    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t payloadLength = 0;

    uint16_t crcReceived = 0;
    uint16_t crcComputed = 0;
};


// ======================================================
// RF Payload Structures
// ======================================================

// HEARTBEAT payload
struct HeartbeatPayload
{
    uint8_t buoyFlags = 0;
    uint8_t lastAcousticAgeSec = 0;
    uint8_t gpsFlags = 0;
    uint8_t rfLinkQuality = 0;
    uint8_t activeFuzeBitmap = 0;
};

// EVENT payload
struct EventPayload
{
    uint8_t fuzeId = 0;
    uint8_t acousticFunction = 0;
    uint16_t acousticRaw = 0;
    uint16_t decodedValue = 0;
    uint8_t eventFlags = 0;
    uint8_t age100ms = 0;
};

// TELEMETRY_DELTA logical model
struct TelemetryDeltaState
{
    uint8_t fuzeId = 0;
    uint8_t fieldMask = 0;

    uint16_t depth_cm = 0;
    uint16_t temp_centiC = 0;
    uint16_t mag_nT = 0;
    uint16_t acoustic_centiDb = 0;
    uint8_t status = 0;
    uint16_t diagnostic = 0;
};

// COMMAND payload
struct CommandPayload
{
    TargetType targetType = TargetType::BUOY;
    uint8_t targetId = 0;
    uint8_t commandId = 0;
    uint8_t argument = 0;
    uint8_t commandFlags = 0;
};

// ACK payload
struct AckPayload
{
    uint8_t ackedMessageType = 0;
    uint16_t ackedSequence = 0;
    RfAckCode ackCode = RfAckCode::OK;
    uint8_t detail = 0;
};

// DIAGNOSTIC payload
struct DiagnosticPayload
{
    uint8_t targetType = 0;
    uint8_t targetId = 0;
    uint16_t diagnosticBits = 0;
    uint8_t extraFlags = 0;
};

// STATUS_SNAPSHOT payload
struct StatusSnapshotPayload
{
    uint8_t fuzeId = 0;
    uint8_t status = 0;
    uint16_t depth_cm = 0;
    uint16_t temp_centiC = 0;
    uint16_t magnetic_nT = 0;
    uint16_t acoustic_centiDb = 0;
    uint16_t diagnosticBits = 0;
};


// ======================================================
// Core frame functions
// ======================================================

bool encodeFrame(
    MessageType type,
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const uint8_t* payload,
    uint8_t payloadLength,
    Frame& outFrame
);

bool decodeFrame(const uint8_t* data, size_t length, DecodedFrame& outFrame);


// ======================================================
// CRC helper
// ======================================================

uint16_t computeCrc16(const uint8_t* data, size_t length);


// ======================================================
// Message builders
// ======================================================

bool buildHeartbeat(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const HeartbeatPayload& hb,
    Frame& outFrame
);

bool buildEvent(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const EventPayload& eventPayload,
    Frame& outFrame
);

bool buildTelemetryDelta(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const TelemetryDeltaState& delta,
    Frame& outFrame
);

bool buildCommand(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const CommandPayload& cmd,
    Frame& outFrame
);

bool buildAck(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const AckPayload& ack,
    Frame& outFrame
);

bool buildDiagnostic(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const DiagnosticPayload& diag,
    Frame& outFrame
);

bool buildStatusSnapshot(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const StatusSnapshotPayload& snapshot,
    Frame& outFrame
);


// ======================================================
// Payload parsers
// ======================================================

bool parseHeartbeat(const DecodedFrame& frame, HeartbeatPayload& out);
bool parseEvent(const DecodedFrame& frame, EventPayload& out);
bool parseTelemetryDelta(const DecodedFrame& frame, TelemetryDeltaState& out);
bool parseCommand(const DecodedFrame& frame, CommandPayload& out);
bool parseAck(const DecodedFrame& frame, AckPayload& out);
bool parseDiagnostic(const DecodedFrame& frame, DiagnosticPayload& out);
bool parseStatusSnapshot(const DecodedFrame& frame, StatusSnapshotPayload& out);


// ======================================================
// Sequence helper
// ======================================================

uint16_t nextSequence();


// ======================================================
// Debug helpers
// ======================================================

const char* messageTypeName(MessageType type);
const char* nodeName(NodeId node);
const char* rfAckName(RfAckCode code);


// ======================================================
// Optional stream helpers
// ======================================================

bool writeFrame(Stream& stream, const Frame& frame);
bool hexDumpFrame(Stream& stream, const Frame& frame);

} // namespace ZeetaRfLink
