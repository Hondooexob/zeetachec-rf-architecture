#include "ZeetaAcomms16.h"

namespace ZeetaAcomms16
{

// ======================================================
// Internal helpers
// ======================================================

static inline uint16_t clampPayload10(uint16_t value)
{
    return (value > PAYLOAD_MAX) ? PAYLOAD_MAX : value;
}

static inline float clampFloat(float value, float minVal, float maxVal)
{
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

static inline uint16_t scaleFloatTo10(float value, float minVal, float maxVal)
{
    value = clampFloat(value, minVal, maxVal);
    const float span = maxVal - minVal;
    const float normalized = (value - minVal) / span;
    const float scaled = normalized * static_cast<float>(PAYLOAD_MAX);
    return clampPayload10(static_cast<uint16_t>(scaled + 0.5f));
}

static inline float scale10ToFloat(uint16_t payload, float minVal, float maxVal)
{
    payload = clampPayload10(payload);
    const float span = maxVal - minVal;
    return minVal + (static_cast<float>(payload) * span / static_cast<float>(PAYLOAD_MAX));
}


// ======================================================
// Core packet functions
// ======================================================

uint16_t encodePacket(Function fn, FuseId fuseId, uint16_t payload10)
{
    const uint16_t fnBits   = (static_cast<uint16_t>(fn) & 0x0F) << 12;
    const uint16_t idBits   = (static_cast<uint16_t>(fuseId) & 0x03) << 10;
    const uint16_t dataBits = clampPayload10(payload10) & 0x03FF;
    return static_cast<uint16_t>(fnBits | idBits | dataBits);
}

Function getFunction(uint16_t raw)
{
    return static_cast<Function>((raw >> 12) & 0x0F);
}

FuseId getFuseId(uint16_t raw)
{
    return static_cast<FuseId>((raw >> 10) & 0x03);
}

uint16_t getPayload(uint16_t raw)
{
    return raw & 0x03FF;
}


// ======================================================
// Scaling helpers
// ======================================================

// Depth: 0 to 100.0 m
uint16_t encodeDepthPayload(float depth_m)
{
    return scaleFloatTo10(depth_m, 0.0f, 100.0f);
}

float decodeDepthPayload(uint16_t payload)
{
    return scale10ToFloat(payload, 0.0f, 100.0f);
}

// Temperature: -2.0 C to 40.0 C
uint16_t encodeTemperaturePayload(float temp_c)
{
    return scaleFloatTo10(temp_c, -2.0f, 40.0f);
}

float decodeTemperaturePayload(uint16_t payload)
{
    return scale10ToFloat(payload, -2.0f, 40.0f);
}

// Magnetic event: 0 to 2.0 uT unsigned magnitude
uint16_t encodeMagPayload(float magnitude_uT)
{
    return scaleFloatTo10(magnitude_uT, 0.0f, 2.0f);
}

float decodeMagPayload(uint16_t payload)
{
    return scale10ToFloat(payload, 0.0f, 2.0f);
}

// Acoustic event: 0 to 48.0 dB above baseline
uint16_t encodeAcousticPayload(float anomaly_dB)
{
    return scaleFloatTo10(anomaly_dB, 0.0f, 48.0f);
}

float decodeAcousticPayload(uint16_t payload)
{
    return scale10ToFloat(payload, 0.0f, 48.0f);
}


// ======================================================
// Typed encoding helpers
// ======================================================

uint16_t encodeDepth(FuseId id, float depth_m)
{
    return encodePacket(Function::CAL_DEPTH, id, encodeDepthPayload(depth_m));
}

uint16_t encodeTemperatureC(FuseId id, float temp_c)
{
    return encodePacket(Function::CAL_TEMP, id, encodeTemperaturePayload(temp_c));
}

uint16_t encodeMagEvent(FuseId id, float magnitude_uT)
{
    return encodePacket(Function::MAG_EVENT, id, encodeMagPayload(magnitude_uT));
}

uint16_t encodeAcousticEvent(FuseId id, float anomaly_dB)
{
    return encodePacket(Function::ACOUSTIC_EVENT, id, encodeAcousticPayload(anomaly_dB));
}

uint16_t encodeStatus(FuseId id, StatusMode mode, uint8_t flags)
{
    const uint16_t payload =
        ((static_cast<uint16_t>(mode) & 0x07) << 7) |
        (static_cast<uint16_t>(flags) & 0x7F);
    return encodePacket(Function::STATUS, id, payload);
}

uint16_t encodeDiagnostic(FuseId id, uint16_t diagBits)
{
    return encodePacket(Function::DIAGNOSTIC, id, diagBits & 0x03FF);
}

uint16_t encodeAck(FuseId id, AckCode code, uint8_t reference)
{
    const uint16_t payload =
        ((static_cast<uint16_t>(code) & 0x0F) << 6) |
        (static_cast<uint16_t>(reference) & 0x3F);
    return encodePacket(Function::ACK, id, payload);
}

uint16_t encodeCommand(FuseId id, CommandId cmd, uint8_t arg)
{
    const uint16_t payload =
        ((static_cast<uint16_t>(cmd) & 0x0F) << 6) |
        (static_cast<uint16_t>(arg) & 0x3F);
    return encodePacket(Function::COMMAND, id, payload);
}

uint16_t encodeHeartbeat(FuseId id, StatusMode mode, uint8_t info)
{
    const uint16_t payload =
        ((static_cast<uint16_t>(mode) & 0x07) << 7) |
        (static_cast<uint16_t>(info) & 0x7F);
    return encodePacket(Function::HEARTBEAT, id, payload);
}

uint16_t encodeConfig(FuseId id, uint8_t key, uint8_t value)
{
    const uint16_t payload =
        ((static_cast<uint16_t>(key) & 0x0F) << 6) |
        (static_cast<uint16_t>(value) & 0x3F);
    return encodePacket(Function::CONFIG, id, payload);
}


// ======================================================
// Decode packet
// ======================================================

DecodedPacket decodePacket(uint16_t raw)
{
    DecodedPacket out{};
    out.valid = true;
    out.function = getFunction(raw);
    out.fuseId = getFuseId(raw);
    out.payload10 = getPayload(raw);

    switch (out.function)
    {
        case Function::CAL_DEPTH:
            out.depth_m = decodeDepthPayload(out.payload10);
            break;

        case Function::CAL_TEMP:
            out.temp_c = decodeTemperaturePayload(out.payload10);
            break;

        case Function::MAG_EVENT:
            out.mag_uT = decodeMagPayload(out.payload10);
            break;

        case Function::ACOUSTIC_EVENT:
            out.acoustic_dB = decodeAcousticPayload(out.payload10);
            break;

        case Function::STATUS:
            out.statusMode  = static_cast<StatusMode>((out.payload10 >> 7) & 0x07);
            out.statusFlags = static_cast<uint8_t>(out.payload10 & 0x7F);
            break;

        case Function::DIAGNOSTIC:
            out.diagnosticBits = out.payload10 & 0x03FF;
            break;

        case Function::ACK:
            out.ackCode = static_cast<AckCode>((out.payload10 >> 6) & 0x0F);
            out.ackReference = static_cast<uint8_t>(out.payload10 & 0x3F);
            break;

        case Function::COMMAND:
            out.commandId = static_cast<CommandId>((out.payload10 >> 6) & 0x0F);
            out.commandArg = static_cast<uint8_t>(out.payload10 & 0x3F);
            break;

        case Function::HEARTBEAT:
            out.statusMode  = static_cast<StatusMode>((out.payload10 >> 7) & 0x07);
            out.statusFlags = static_cast<uint8_t>(out.payload10 & 0x7F);
            break;

        case Function::CONFIG:
            // In v1, config key/value are not broken out in the struct.
            // Payload remains available in payload10 for callers.
            break;

        default:
            // Reserved values still decode structurally.
            break;
    }

    return out;
}


// ======================================================
// Debug helpers
// ======================================================

const char* functionName(Function fn)
{
    switch (fn)
    {
        case Function::CAL_DEPTH:       return "CAL_DEPTH";
        case Function::CAL_TEMP:        return "CAL_TEMP";
        case Function::MAG_EVENT:       return "MAG_EVENT";
        case Function::ACOUSTIC_EVENT:  return "ACOUSTIC_EVENT";
        case Function::STATUS:          return "STATUS";
        case Function::DIAGNOSTIC:      return "DIAGNOSTIC";
        case Function::ACK:             return "ACK";
        case Function::COMMAND:         return "COMMAND";
        case Function::HEARTBEAT:       return "HEARTBEAT";
        case Function::CONFIG:          return "CONFIG";
        case Function::RESERVED_A:      return "RESERVED_A";
        case Function::RESERVED_B:      return "RESERVED_B";
        case Function::RESERVED_C:      return "RESERVED_C";
        case Function::RESERVED_D:      return "RESERVED_D";
        case Function::RESERVED_E:      return "RESERVED_E";
        case Function::RESERVED_F:      return "RESERVED_F";
        default:                        return "UNKNOWN";
    }
}

const char* commandName(CommandId cmd)
{
    switch (cmd)
    {
        case CommandId::NO_OP:               return "NO_OP";
        case CommandId::ENTER_CAL:           return "ENTER_CAL";
        case CommandId::ENTER_ACTIVE:        return "ENTER_ACTIVE";
        case CommandId::START_RECORD:        return "START_RECORD";
        case CommandId::STOP_RECORD:         return "STOP_RECORD";
        case CommandId::RECALL_ON:           return "RECALL_ON";
        case CommandId::RECALL_OFF:          return "RECALL_OFF";
        case CommandId::REQUEST_STATUS:      return "REQUEST_STATUS";
        case CommandId::REQUEST_DIAGNOSTIC:  return "REQUEST_DIAGNOSTIC";
        case CommandId::SET_MAG_THRESHOLD:   return "SET_MAG_THRESHOLD";
        case CommandId::SET_ACOUSTIC_THRESH: return "SET_ACOUSTIC_THRESH";
        case CommandId::LEDS_ON:             return "LEDS_ON";
        case CommandId::LEDS_OFF:            return "LEDS_OFF";
        case CommandId::STROBE_ON:           return "STROBE_ON";
        case CommandId::STROBE_OFF:          return "STROBE_OFF";
        case CommandId::REBOOT:              return "REBOOT";
        default:                             return "UNKNOWN";
    }
}

const char* ackName(AckCode code)
{
    switch (code)
    {
        case AckCode::OK:               return "OK";
        case AckCode::INVALID_COMMAND:  return "INVALID_COMMAND";
        case AckCode::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case AckCode::BUSY:             return "BUSY";
        case AckCode::UNSUPPORTED:      return "UNSUPPORTED";
        case AckCode::SENSOR_FAILURE:   return "SENSOR_FAILURE";
        case AckCode::STORAGE_FAILURE:  return "STORAGE_FAILURE";
        case AckCode::MODE_REJECTED:    return "MODE_REJECTED";
        case AckCode::TIMEOUT:          return "TIMEOUT";
        case AckCode::RESERVED_9:       return "RESERVED_9";
        case AckCode::RESERVED_A:       return "RESERVED_A";
        case AckCode::RESERVED_B:       return "RESERVED_B";
        case AckCode::RESERVED_C:       return "RESERVED_C";
        case AckCode::RESERVED_D:       return "RESERVED_D";
        case AckCode::RESERVED_E:       return "RESERVED_E";
        case AckCode::RESERVED_F:       return "RESERVED_F";
        default:                        return "UNKNOWN";
    }
}

const char* statusModeName(StatusMode mode)
{
    switch (mode)
    {
        case StatusMode::IDLE:        return "IDLE";
        case StatusMode::INIT:        return "INIT";
        case StatusMode::CALIBRATION: return "CALIBRATION";
        case StatusMode::ACTIVE:      return "ACTIVE";
        case StatusMode::RX_WINDOW:   return "RX_WINDOW";
        case StatusMode::RECALL:      return "RECALL";
        case StatusMode::ERROR_MODE:  return "ERROR_MODE";
        case StatusMode::RESERVED:    return "RESERVED";
        default:                      return "UNKNOWN";
    }
}

} // namespace ZeetaAcomms16