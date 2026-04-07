#include "ZeetaRfLink.h"

namespace ZeetaRfLink
{

// ======================================================
// Internal helpers
// ======================================================

static uint16_t g_sequenceCounter = 0;

static inline void writeU16BE(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
    dst[1] = static_cast<uint8_t>(value & 0xFF);
}

static inline uint16_t readU16BE(const uint8_t* src)
{
    return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) |
                                 static_cast<uint16_t>(src[1]));
}

static bool appendBytes(Frame& frame, const uint8_t* src, size_t count)
{
    if (frame.length + count > MAX_FRAME_SIZE)
    {
        return false;
    }

    for (size_t i = 0; i < count; ++i)
    {
        frame.data[frame.length++] = src[i];
    }

    return true;
}

static bool appendByte(Frame& frame, uint8_t value)
{
    return appendBytes(frame, &value, 1);
}


// ======================================================
// CRC16 (CCITT-FALSE style)
// ======================================================

uint16_t computeCrc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;

        for (uint8_t b = 0; b < 8; ++b)
        {
            if (crc & 0x8000)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


// ======================================================
// Core frame encode/decode
// ======================================================

bool encodeFrame(
    MessageType type,
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const uint8_t* payload,
    uint8_t payloadLength,
    Frame& outFrame
)
{
    if (payloadLength > MAX_PAYLOAD_SIZE)
    {
        return false;
    }

    outFrame.length = 0;

    if (!appendByte(outFrame, SOF)) return false;
    if (!appendByte(outFrame, PROTOCOL_VERSION)) return false;
    if (!appendByte(outFrame, static_cast<uint8_t>(type))) return false;
    if (!appendByte(outFrame, static_cast<uint8_t>(src))) return false;
    if (!appendByte(outFrame, static_cast<uint8_t>(dst))) return false;

    uint8_t seqBytes[2];
    writeU16BE(seqBytes, sequence);
    if (!appendBytes(outFrame, seqBytes, 2)) return false;

    if (!appendByte(outFrame, payloadLength)) return false;

    if ((payloadLength > 0) && (payload != nullptr))
    {
        if (!appendBytes(outFrame, payload, payloadLength)) return false;
    }

    const uint16_t crc = computeCrc16(outFrame.data, outFrame.length);
    uint8_t crcBytes[2];
    writeU16BE(crcBytes, crc);

    if (!appendBytes(outFrame, crcBytes, 2)) return false;

    return true;
}

bool decodeFrame(const uint8_t* data, size_t length, DecodedFrame& outFrame)
{
    outFrame = DecodedFrame{};

    if (data == nullptr)
    {
        return false;
    }

    if (length < (HEADER_SIZE + CRC_SIZE))
    {
        return false;
    }

    if (data[0] != SOF)
    {
        return false;
    }

    const uint8_t payloadLength = data[7];
    const size_t expectedLength = HEADER_SIZE + payloadLength + CRC_SIZE;

    if (length != expectedLength)
    {
        return false;
    }

    outFrame.header.sof = data[0];
    outFrame.header.version = data[1];
    outFrame.header.type = static_cast<MessageType>(data[2]);
    outFrame.header.src = static_cast<NodeId>(data[3]);
    outFrame.header.dst = static_cast<NodeId>(data[4]);
    outFrame.header.sequence = readU16BE(&data[5]);
    outFrame.header.payloadLength = payloadLength;

    outFrame.payloadLength = payloadLength;

    if (payloadLength > 0)
    {
        for (uint8_t i = 0; i < payloadLength; ++i)
        {
            outFrame.payload[i] = data[8 + i];
        }
    }

    outFrame.crcReceived = readU16BE(&data[length - 2]);
    outFrame.crcComputed = computeCrc16(data, length - 2);

    outFrame.valid = (outFrame.crcReceived == outFrame.crcComputed);
    return outFrame.valid;
}


// ======================================================
// Message builders
// ======================================================

bool buildHeartbeat(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const HeartbeatPayload& hb,
    Frame& outFrame
)
{
    uint8_t payload[5];
    payload[0] = hb.buoyFlags;
    payload[1] = hb.lastAcousticAgeSec;
    payload[2] = hb.gpsFlags;
    payload[3] = hb.rfLinkQuality;
    payload[4] = hb.activeFuzeBitmap;

    return encodeFrame(MessageType::HEARTBEAT, src, dst, sequence, payload, sizeof(payload), outFrame);
}

bool buildEvent(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const EventPayload& eventPayload,
    Frame& outFrame
)
{
    uint8_t payload[8];
    payload[0] = eventPayload.fuzeId;
    payload[1] = eventPayload.acousticFunction;
    writeU16BE(&payload[2], eventPayload.acousticRaw);
    writeU16BE(&payload[4], eventPayload.decodedValue);
    payload[6] = eventPayload.eventFlags;
    payload[7] = eventPayload.age100ms;

    return encodeFrame(MessageType::EVENT, src, dst, sequence, payload, sizeof(payload), outFrame);
}

bool buildTelemetryDelta(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const TelemetryDeltaState& delta,
    Frame& outFrame
)
{
    uint8_t payload[MAX_PAYLOAD_SIZE];
    uint8_t idx = 0;

    payload[idx++] = delta.fuzeId;
    payload[idx++] = delta.fieldMask;

    if (delta.fieldMask & (1u << 0)) // depth
    {
        writeU16BE(&payload[idx], delta.depth_cm);
        idx += 2;
    }

    if (delta.fieldMask & (1u << 1)) // temp
    {
        writeU16BE(&payload[idx], delta.temp_centiC);
        idx += 2;
    }

    if (delta.fieldMask & (1u << 2)) // mag
    {
        writeU16BE(&payload[idx], delta.mag_nT);
        idx += 2;
    }

    if (delta.fieldMask & (1u << 3)) // acoustic
    {
        writeU16BE(&payload[idx], delta.acoustic_centiDb);
        idx += 2;
    }

    if (delta.fieldMask & (1u << 4)) // status
    {
        payload[idx++] = delta.status;
    }

    if (delta.fieldMask & (1u << 5)) // diagnostic
    {
        writeU16BE(&payload[idx], delta.diagnostic);
        idx += 2;
    }

    return encodeFrame(MessageType::TELEMETRY_DELTA, src, dst, sequence, payload, idx, outFrame);
}

bool buildCommand(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const CommandPayload& cmd,
    Frame& outFrame
)
{
    uint8_t payload[5];
    payload[0] = static_cast<uint8_t>(cmd.targetType);
    payload[1] = cmd.targetId;
    payload[2] = cmd.commandId;
    payload[3] = cmd.argument;
    payload[4] = cmd.commandFlags;

    return encodeFrame(MessageType::COMMAND, src, dst, sequence, payload, sizeof(payload), outFrame);
}

bool buildAck(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const AckPayload& ack,
    Frame& outFrame
)
{
    uint8_t payload[5];
    payload[0] = ack.ackedMessageType;
    writeU16BE(&payload[1], ack.ackedSequence);
    payload[3] = static_cast<uint8_t>(ack.ackCode);
    payload[4] = ack.detail;

    return encodeFrame(MessageType::ACK, src, dst, sequence, payload, sizeof(payload), outFrame);
}

bool buildDiagnostic(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const DiagnosticPayload& diag,
    Frame& outFrame
)
{
    uint8_t payload[5];
    payload[0] = diag.targetType;
    payload[1] = diag.targetId;
    writeU16BE(&payload[2], diag.diagnosticBits);
    payload[4] = diag.extraFlags;

    return encodeFrame(MessageType::DIAGNOSTIC, src, dst, sequence, payload, sizeof(payload), outFrame);
}

bool buildStatusSnapshot(
    NodeId src,
    NodeId dst,
    uint16_t sequence,
    const StatusSnapshotPayload& snapshot,
    Frame& outFrame
)
{
    uint8_t payload[12];
    payload[0] = snapshot.fuzeId;
    payload[1] = snapshot.status;
    writeU16BE(&payload[2], snapshot.depth_cm);
    writeU16BE(&payload[4], snapshot.temp_centiC);
    writeU16BE(&payload[6], snapshot.magnetic_nT);
    writeU16BE(&payload[8], snapshot.acoustic_centiDb);
    writeU16BE(&payload[10], snapshot.diagnosticBits);

    return encodeFrame(MessageType::STATUS_SNAPSHOT, src, dst, sequence, payload, sizeof(payload), outFrame);
}


// ======================================================
// Payload parsers
// ======================================================

bool parseHeartbeat(const DecodedFrame& frame, HeartbeatPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::HEARTBEAT || frame.payloadLength != 5)
    {
        return false;
    }

    out.buoyFlags = frame.payload[0];
    out.lastAcousticAgeSec = frame.payload[1];
    out.gpsFlags = frame.payload[2];
    out.rfLinkQuality = frame.payload[3];
    out.activeFuzeBitmap = frame.payload[4];
    return true;
}

bool parseEvent(const DecodedFrame& frame, EventPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::EVENT || frame.payloadLength != 8)
    {
        return false;
    }

    out.fuzeId = frame.payload[0];
    out.acousticFunction = frame.payload[1];
    out.acousticRaw = readU16BE(&frame.payload[2]);
    out.decodedValue = readU16BE(&frame.payload[4]);
    out.eventFlags = frame.payload[6];
    out.age100ms = frame.payload[7];
    return true;
}

bool parseTelemetryDelta(const DecodedFrame& frame, TelemetryDeltaState& out)
{
    if (!frame.valid || frame.header.type != MessageType::TELEMETRY_DELTA || frame.payloadLength < 2)
    {
        return false;
    }

    out = TelemetryDeltaState{};
    out.fuzeId = frame.payload[0];
    out.fieldMask = frame.payload[1];

    uint8_t idx = 2;

    auto requireBytes = [&](uint8_t count) -> bool
    {
        return static_cast<uint16_t>(idx + count) <= frame.payloadLength;
    };

    if (out.fieldMask & (1u << 0))
    {
        if (!requireBytes(2)) return false;
        out.depth_cm = readU16BE(&frame.payload[idx]);
        idx += 2;
    }

    if (out.fieldMask & (1u << 1))
    {
        if (!requireBytes(2)) return false;
        out.temp_centiC = readU16BE(&frame.payload[idx]);
        idx += 2;
    }

    if (out.fieldMask & (1u << 2))
    {
        if (!requireBytes(2)) return false;
        out.mag_nT = readU16BE(&frame.payload[idx]);
        idx += 2;
    }

    if (out.fieldMask & (1u << 3))
    {
        if (!requireBytes(2)) return false;
        out.acoustic_centiDb = readU16BE(&frame.payload[idx]);
        idx += 2;
    }

    if (out.fieldMask & (1u << 4))
    {
        if (!requireBytes(1)) return false;
        out.status = frame.payload[idx++];
    }

    if (out.fieldMask & (1u << 5))
    {
        if (!requireBytes(2)) return false;
        out.diagnostic = readU16BE(&frame.payload[idx]);
        idx += 2;
    }

    return (idx == frame.payloadLength);
}

bool parseCommand(const DecodedFrame& frame, CommandPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::COMMAND || frame.payloadLength != 5)
    {
        return false;
    }

    out.targetType = static_cast<TargetType>(frame.payload[0]);
    out.targetId = frame.payload[1];
    out.commandId = frame.payload[2];
    out.argument = frame.payload[3];
    out.commandFlags = frame.payload[4];
    return true;
}

bool parseAck(const DecodedFrame& frame, AckPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::ACK || frame.payloadLength != 5)
    {
        return false;
    }

    out.ackedMessageType = frame.payload[0];
    out.ackedSequence = readU16BE(&frame.payload[1]);
    out.ackCode = static_cast<RfAckCode>(frame.payload[3]);
    out.detail = frame.payload[4];
    return true;
}

bool parseDiagnostic(const DecodedFrame& frame, DiagnosticPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::DIAGNOSTIC || frame.payloadLength != 5)
    {
        return false;
    }

    out.targetType = frame.payload[0];
    out.targetId = frame.payload[1];
    out.diagnosticBits = readU16BE(&frame.payload[2]);
    out.extraFlags = frame.payload[4];
    return true;
}

bool parseStatusSnapshot(const DecodedFrame& frame, StatusSnapshotPayload& out)
{
    if (!frame.valid || frame.header.type != MessageType::STATUS_SNAPSHOT || frame.payloadLength != 12)
    {
        return false;
    }

    out.fuzeId = frame.payload[0];
    out.status = frame.payload[1];
    out.depth_cm = readU16BE(&frame.payload[2]);
    out.temp_centiC = readU16BE(&frame.payload[4]);
    out.magnetic_nT = readU16BE(&frame.payload[6]);
    out.acoustic_centiDb = readU16BE(&frame.payload[8]);
    out.diagnosticBits = readU16BE(&frame.payload[10]);
    return true;
}


// ======================================================
// Sequence helper
// ======================================================

uint16_t nextSequence()
{
    ++g_sequenceCounter;
    return g_sequenceCounter;
}


// ======================================================
// Debug helpers
// ======================================================

const char* messageTypeName(MessageType type)
{
    switch (type)
    {
        case MessageType::HEARTBEAT:       return "HEARTBEAT";
        case MessageType::TELEMETRY_DELTA: return "TELEMETRY_DELTA";
        case MessageType::EVENT:           return "EVENT";
        case MessageType::COMMAND:         return "COMMAND";
        case MessageType::ACK:             return "ACK";
        case MessageType::DIAGNOSTIC:      return "DIAGNOSTIC";
        case MessageType::STATUS_SNAPSHOT: return "STATUS_SNAPSHOT";
        case MessageType::SATCOM_STATUS:   return "SATCOM_STATUS";
        case MessageType::SATCOM_COMMAND:  return "SATCOM_COMMAND";
        case MessageType::PING:            return "PING";
        default:                           return "UNKNOWN";
    }
}

const char* nodeName(NodeId node)
{
    switch (node)
    {
        case NodeId::UNKNOWN:         return "UNKNOWN";
        case NodeId::BUOY_1:          return "BUOY_1";
        case NodeId::BUOY_2:          return "BUOY_2";
        case NodeId::RF_CONTROLLER_1: return "RF_CONTROLLER_1";
        case NodeId::SATCOM_MODULE:   return "SATCOM_MODULE";
        case NodeId::BROADCAST:       return "BROADCAST";
        default:                      return "UNKNOWN";
    }
}

const char* rfAckName(RfAckCode code)
{
    switch (code)
    {
        case RfAckCode::OK:              return "OK";
        case RfAckCode::INVALID_COMMAND: return "INVALID_COMMAND";
        case RfAckCode::INVALID_ARG:     return "INVALID_ARG";
        case RfAckCode::BUSY:            return "BUSY";
        case RfAckCode::FORWARD_FAILED:  return "FORWARD_FAILED";
        case RfAckCode::TIMEOUT:         return "TIMEOUT";
        case RfAckCode::UNSUPPORTED:     return "UNSUPPORTED";
        default:                         return "UNKNOWN";
    }
}


// ======================================================
// Optional stream helpers
// ======================================================

bool writeFrame(Stream& stream, const Frame& frame)
{
    if (frame.length == 0)
    {
        return false;
    }

    const size_t written = stream.write(frame.data, frame.length);
    return (written == frame.length);
}

bool hexDumpFrame(Stream& stream, const Frame& frame)
{
    if (frame.length == 0)
    {
        return false;
    }

    for (size_t i = 0; i < frame.length; ++i)
    {
        if (frame.data[i] < 0x10)
        {
            stream.print('0');
        }
        stream.print(frame.data[i], HEX);

        if (i + 1 < frame.length)
        {
            stream.print(' ');
        }
    }
    stream.println();
    return true;
}

} // namespace ZeetaRfLink
