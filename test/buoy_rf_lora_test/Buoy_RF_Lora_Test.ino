#include <Arduino.h>
#include <RadioLib.h>
#include "ZeetaAcomms16.h"
#include "ZeetaRfLink.h"

// ======================================================
// Buoy RF LoRa Developer Firmware
//
// Purpose:
// - Developer-facing buoy firmware for RF Controller V2 work
// - Accept mock acoustic packets over USB serial
// - Optionally accept real 16-bit packets on Serial1
// - Forward RF frames over actual SX1262 LoRa
// - Accept COMMAND frames from RF controller over LoRa
// - Convert fuze-targeted commands into 16-bit acoustic COMMAND packets
//
// USB commands:
//   HELP
//   FUZES
//   HEARTBEAT
//   RESETSIM
//   SNAP1 / SNAP2 / SNAP3 / SNAP4
//
//   DEP1 18.6
//   TMP2 14.2
//   MAG3 0.42
//   ACO4 12.5
//   STA1 3 12
//   DIA2 0011
//   ACK1 0 7
//
//   SCENARIO QUIET
//   SCENARIO MIXED
//   SCENARIO MAG_BURST
//   SCENARIO ACO_SWEEP
//   STOP
//
// Notes:
// - Serial1 = acoustic side (real modem or external source)
// - USB Serial = developer console
// - LoRa SX1262 = real RF link to controller
// ======================================================

using namespace ZeetaAcomms16;
using namespace ZeetaRfLink;

// ------------------------------------------------------
// SX1262 pinout
// ------------------------------------------------------

static constexpr int LORA_NSS  = 10;
static constexpr int LORA_DIO1 = 7;
static constexpr int LORA_RST  = 9;
static constexpr int LORA_BUSY = 8;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ------------------------------------------------------
// LoRa settings
// Must match controller exactly
// ------------------------------------------------------

static constexpr float LORA_FREQ_MHZ = 915.0;
static constexpr float LORA_BW_KHZ   = 125.0;
static constexpr uint8_t LORA_SF     = 11;
static constexpr uint8_t LORA_CR     = 7;
static constexpr uint8_t LORA_SYNC   = 0x12;
static constexpr int8_t LORA_POWER   = 22;
static constexpr uint16_t LORA_PREAMBLE = 8;

// ------------------------------------------------------
// Node configuration
// ------------------------------------------------------

static constexpr NodeId THIS_BUOY = NodeId::BUOY_1;
static constexpr NodeId THIS_CONTROLLER = NodeId::RF_CONTROLLER_1;

static constexpr uint32_t SERIAL_DEBUG_BAUD = 115200;
static constexpr uint32_t ACOUSTIC_BAUD = 9600;

static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
static constexpr uint32_t SCENARIO_INTERVAL_MS = 1500;

// ------------------------------------------------------
// Radio RX state
// ------------------------------------------------------

volatile bool g_receivedFlag = false;
volatile bool g_enableInterrupt = true;

void setReceivedFlag(void) {
  if(!g_enableInterrupt) return;
  g_receivedFlag = true;
}

// ------------------------------------------------------
// Scenario types
// ------------------------------------------------------

enum class ScenarioMode : uint8_t
{
  OFF = 0,
  QUIET,
  MIXED,
  MAG_BURST,
  ACO_SWEEP
};

// ------------------------------------------------------
// Per-fuze cache
// ------------------------------------------------------

struct FuzeState
{
  bool active = false;

  uint16_t lastRawAcoustic = 0;
  uint32_t lastAcousticMillis = 0;

  float depth_m = 0.0f;
  float temp_c = 0.0f;
  float mag_uT = 0.0f;
  float acoustic_dB = 0.0f;

  uint8_t statusByte = 0;
  uint16_t diagnosticBits = 0;

  bool depthDirty = false;
  bool tempDirty = false;
  bool magDirty = false;
  bool acousticDirty = false;
  bool statusDirty = false;
  bool diagnosticDirty = false;
};

FuzeState g_fuze[4];

uint32_t g_lastHeartbeatMs = 0;
uint32_t g_lastScenarioMs = 0;
ScenarioMode g_scenarioMode = ScenarioMode::OFF;
uint32_t g_scenarioStep = 0;

// ------------------------------------------------------
// Helpers
// ------------------------------------------------------

static uint8_t fuseIndexFromId(FuseId id)
{
  return static_cast<uint8_t>(id);
}

static uint8_t fuseNumberFromId(FuseId id)
{
  return static_cast<uint8_t>(id) + 1;
}

static FuseId fuseIdFromNumber(uint8_t n)
{
  if(n < 1) n = 1;
  if(n > 4) n = 4;
  return static_cast<FuseId>(n - 1);
}

static uint8_t buildActiveFuzeBitmap()
{
  uint8_t bitmap = 0;
  for(uint8_t i = 0; i < 4; ++i)
  {
    if(g_fuze[i].active)
    {
      bitmap |= (1u << i);
    }
  }
  return bitmap;
}

static uint8_t packStatusByte(StatusMode mode, uint8_t flags)
{
  return static_cast<uint8_t>(((static_cast<uint8_t>(mode) & 0x07) << 5) | (flags & 0x1F));
}

static uint16_t depthToCm(float depth_m)
{
  if(depth_m < 0.0f) depth_m = 0.0f;
  return static_cast<uint16_t>(depth_m * 100.0f + 0.5f);
}

static uint16_t tempToCentiC(float temp_c)
{
  return static_cast<uint16_t>((temp_c + 100.0f) * 100.0f + 0.5f);
}

static uint16_t magToNanoTesla(float mag_uT)
{
  if(mag_uT < 0.0f) mag_uT = 0.0f;
  return static_cast<uint16_t>(mag_uT * 1000.0f + 0.5f);
}

static uint16_t acousticToCentiDb(float acoustic_dB)
{
  if(acoustic_dB < 0.0f) acoustic_dB = 0.0f;
  return static_cast<uint16_t>(acoustic_dB * 100.0f + 0.5f);
}

static void printDivider()
{
  Serial.println("----------------------------------------");
}

// ------------------------------------------------------
// Debug printing
// ------------------------------------------------------

static void printDecodedAcoustic(const DecodedPacket& pkt, uint16_t raw)
{
  Serial.print("[ACOUSTIC] raw=0x");
  Serial.print(raw, HEX);
  Serial.print(" fn=");
  Serial.print(functionName(pkt.function));
  Serial.print(" fuze=");
  Serial.print(fuseNumberFromId(pkt.fuseId));

  switch(pkt.function)
  {
    case Function::CAL_DEPTH:
      Serial.print(" depth_m=");
      Serial.println(pkt.depth_m, 3);
      break;

    case Function::CAL_TEMP:
      Serial.print(" temp_c=");
      Serial.println(pkt.temp_c, 3);
      break;

    case Function::MAG_EVENT:
      Serial.print(" mag_uT=");
      Serial.println(pkt.mag_uT, 6);
      break;

    case Function::ACOUSTIC_EVENT:
      Serial.print(" acoustic_dB=");
      Serial.println(pkt.acoustic_dB, 3);
      break;

    case Function::STATUS:
      Serial.print(" mode=");
      Serial.print(statusModeName(pkt.statusMode));
      Serial.print(" flags=0x");
      Serial.println(pkt.statusFlags, HEX);
      break;

    case Function::DIAGNOSTIC:
      Serial.print(" diag=0x");
      Serial.println(pkt.diagnosticBits, HEX);
      break;

    case Function::ACK:
      Serial.print(" ack=");
      Serial.print(ackName(pkt.ackCode));
      Serial.print(" ref=");
      Serial.println(pkt.ackReference);
      break;

    case Function::COMMAND:
      Serial.print(" cmd=");
      Serial.print(commandName(pkt.commandId));
      Serial.print(" arg=");
      Serial.println(pkt.commandArg);
      break;

    default:
      Serial.println();
      break;
  }
}

static void printFuzeStates()
{
  printDivider();
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print("Fuze ");
    Serial.print(i + 1);
    Serial.print(": active=");
    Serial.print(g_fuze[i].active ? "Y" : "N");
    Serial.print(" depth=");
    Serial.print(g_fuze[i].depth_m, 2);
    Serial.print("m temp=");
    Serial.print(g_fuze[i].temp_c, 2);
    Serial.print("C mag=");
    Serial.print(g_fuze[i].mag_uT, 4);
    Serial.print("uT acoustic=");
    Serial.print(g_fuze[i].acoustic_dB, 2);
    Serial.print("dB diag=0x");
    Serial.print(g_fuze[i].diagnosticBits, HEX);
    Serial.print(" status=0x");
    Serial.println(g_fuze[i].statusByte, HEX);
  }
}

// ------------------------------------------------------
// LoRa TX helpers
// ------------------------------------------------------

static bool transmitFrame(const Frame& frame, const char* label)
{
  Serial.print("[RF TX ");
  Serial.print(label);
  Serial.print("] ");
  hexDumpFrame(Serial, frame);

  g_enableInterrupt = false;
  g_receivedFlag = false;

  int state = radio.transmit(frame.data, frame.length);

  g_enableInterrupt = true;

  if(state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[RF TX] transmit failed, code=");
    Serial.println(state);
    radio.startReceive();
    return false;
  }

  state = radio.startReceive();
  if(state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[RF RX] startReceive failed, code=");
    Serial.println(state);
    return false;
  }

  return true;
}

static bool sendHeartbeat()
{
  uint32_t newestMs = 0;
  for(uint8_t i = 0; i < 4; ++i)
  {
    if(g_fuze[i].lastAcousticMillis > newestMs)
    {
      newestMs = g_fuze[i].lastAcousticMillis;
    }
  }

  uint8_t ageSec = 0xFF;
  if(newestMs > 0)
  {
    uint32_t age = (millis() - newestMs) / 1000UL;
    if(age > 255) age = 255;
    ageSec = static_cast<uint8_t>(age);
  }

  HeartbeatPayload hb{};
  hb.buoyFlags = 0x03;
  hb.lastAcousticAgeSec = ageSec;
  hb.gpsFlags = 0x00;
  hb.rfLinkQuality = 0x64;
  hb.activeFuzeBitmap = buildActiveFuzeBitmap();

  Frame frame{};
  if(!buildHeartbeat(THIS_BUOY, THIS_CONTROLLER, nextSequence(), hb, frame))
  {
    return false;
  }

  return transmitFrame(frame, "HEARTBEAT");
}

static bool sendEventFrame(FuseId fuseId, Function fn, uint16_t rawWord, uint16_t decodedValue)
{
  EventPayload ep{};
  ep.fuzeId = fuseNumberFromId(fuseId);
  ep.acousticFunction = static_cast<uint8_t>(fn);
  ep.acousticRaw = rawWord;
  ep.decodedValue = decodedValue;
  ep.eventFlags = 0x01;
  ep.age100ms = 0;

  Frame frame{};
  if(!buildEvent(THIS_BUOY, THIS_CONTROLLER, nextSequence(), ep, frame))
  {
    return false;
  }

  return transmitFrame(frame, "EVENT");
}

static bool sendTelemetryDeltaFrame(FuseId fuseId)
{
  FuzeState& fs = g_fuze[fuseIndexFromId(fuseId)];

  TelemetryDeltaState delta{};
  delta.fuzeId = fuseNumberFromId(fuseId);

  if(fs.depthDirty)
  {
    delta.fieldMask |= (1u << 0);
    delta.depth_cm = depthToCm(fs.depth_m);
  }

  if(fs.tempDirty)
  {
    delta.fieldMask |= (1u << 1);
    delta.temp_centiC = tempToCentiC(fs.temp_c);
  }

  if(fs.magDirty)
  {
    delta.fieldMask |= (1u << 2);
    delta.mag_nT = magToNanoTesla(fs.mag_uT);
  }

  if(fs.acousticDirty)
  {
    delta.fieldMask |= (1u << 3);
    delta.acoustic_centiDb = acousticToCentiDb(fs.acoustic_dB);
  }

  if(fs.statusDirty)
  {
    delta.fieldMask |= (1u << 4);
    delta.status = fs.statusByte;
  }

  if(fs.diagnosticDirty)
  {
    delta.fieldMask |= (1u << 5);
    delta.diagnostic = fs.diagnosticBits;
  }

  if(delta.fieldMask == 0)
  {
    return true;
  }

  Frame frame{};
  if(!buildTelemetryDelta(THIS_BUOY, THIS_CONTROLLER, nextSequence(), delta, frame))
  {
    return false;
  }

  fs.depthDirty = false;
  fs.tempDirty = false;
  fs.magDirty = false;
  fs.acousticDirty = false;
  fs.statusDirty = false;
  fs.diagnosticDirty = false;

  return transmitFrame(frame, "DELTA");
}

static bool sendStatusSnapshotFrame(uint8_t fuzeNumber)
{
  if(fuzeNumber < 1 || fuzeNumber > 4) return false;

  FuseId fuseId = fuseIdFromNumber(fuzeNumber);
  FuzeState& fs = g_fuze[fuseIndexFromId(fuseId)];

  StatusSnapshotPayload ss{};
  ss.fuzeId = fuzeNumber;
  ss.status = fs.statusByte;
  ss.depth_cm = depthToCm(fs.depth_m);
  ss.temp_centiC = tempToCentiC(fs.temp_c);
  ss.magnetic_nT = magToNanoTesla(fs.mag_uT);
  ss.acoustic_centiDb = acousticToCentiDb(fs.acoustic_dB);
  ss.diagnosticBits = fs.diagnosticBits;

  Frame frame{};
  if(!buildStatusSnapshot(THIS_BUOY, THIS_CONTROLLER, nextSequence(), ss, frame))
  {
    return false;
  }

  return transmitFrame(frame, "SNAPSHOT");
}

static bool sendAckForCommand(uint16_t ackedSequence, RfAckCode ackCode, uint8_t detail)
{
  AckPayload ack{};
  ack.ackedMessageType = static_cast<uint8_t>(MessageType::COMMAND);
  ack.ackedSequence = ackedSequence;
  ack.ackCode = ackCode;
  ack.detail = detail;

  Frame frame{};
  if(!buildAck(THIS_BUOY, THIS_CONTROLLER, nextSequence(), ack, frame))
  {
    return false;
  }

  return transmitFrame(frame, "ACK");
}

// ------------------------------------------------------
// Acoustic handling
// ------------------------------------------------------

static void handleAcousticPacket(uint16_t rawWord)
{
  DecodedPacket pkt = decodePacket(rawWord);
  printDecodedAcoustic(pkt, rawWord);

  const uint8_t idx = fuseIndexFromId(pkt.fuseId);
  FuzeState& fs = g_fuze[idx];

  fs.active = true;
  fs.lastRawAcoustic = rawWord;
  fs.lastAcousticMillis = millis();

  switch(pkt.function)
  {
    case Function::CAL_DEPTH:
      fs.depth_m = pkt.depth_m;
      fs.depthDirty = true;
      sendTelemetryDeltaFrame(pkt.fuseId);
      break;

    case Function::CAL_TEMP:
      fs.temp_c = pkt.temp_c;
      fs.tempDirty = true;
      sendTelemetryDeltaFrame(pkt.fuseId);
      break;

    case Function::MAG_EVENT:
      fs.mag_uT = pkt.mag_uT;
      fs.magDirty = true;
      sendEventFrame(pkt.fuseId, pkt.function, rawWord, magToNanoTesla(pkt.mag_uT));
      break;

    case Function::ACOUSTIC_EVENT:
      fs.acoustic_dB = pkt.acoustic_dB;
      fs.acousticDirty = true;
      sendEventFrame(pkt.fuseId, pkt.function, rawWord, acousticToCentiDb(pkt.acoustic_dB));
      break;

    case Function::STATUS:
      fs.statusByte = packStatusByte(pkt.statusMode, pkt.statusFlags);
      fs.statusDirty = true;
      sendTelemetryDeltaFrame(pkt.fuseId);
      break;

    case Function::DIAGNOSTIC:
      fs.diagnosticBits = pkt.diagnosticBits;
      fs.diagnosticDirty = true;
      sendTelemetryDeltaFrame(pkt.fuseId);
      break;

    default:
      break;
  }
}

static void injectRawAcousticPacket(uint16_t rawWord)
{
  handleAcousticPacket(rawWord);
}

static void pollAcousticSerial()
{
  while(Serial1.available() >= 2)
  {
    const uint8_t msb = static_cast<uint8_t>(Serial1.read());
    const uint8_t lsb = static_cast<uint8_t>(Serial1.read());
    const uint16_t rawWord = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
    handleAcousticPacket(rawWord);
  }
}

// ------------------------------------------------------
// RF command handling
// ------------------------------------------------------

static bool forwardCommandToFuze(uint8_t targetId, uint8_t commandId, uint8_t argument)
{
  if(targetId < 1 || targetId > 4)
  {
    return false;
  }

  const FuseId fuseId = fuseIdFromNumber(targetId);
  const CommandId cmd = static_cast<CommandId>(commandId & 0x0F);

  const uint16_t rawCmd = encodeCommand(fuseId, cmd, static_cast<uint8_t>(argument & 0x3F));

  const uint8_t bytes[2] = {
    static_cast<uint8_t>((rawCmd >> 8) & 0xFF),
    static_cast<uint8_t>(rawCmd & 0xFF)
  };

  Serial.print("[ACOUSTIC CMD TX] raw=0x");
  Serial.println(rawCmd, HEX);

  return (Serial1.write(bytes, 2) == 2);
}

static void handleRfCommand(const DecodedFrame& frame)
{
  CommandPayload cmd{};
  if(!parseCommand(frame, cmd))
  {
    return;
  }

  Serial.print("[RF COMMAND] targetType=");
  Serial.print(static_cast<uint8_t>(cmd.targetType));
  Serial.print(" targetId=");
  Serial.print(cmd.targetId);
  Serial.print(" cmd=");
  Serial.print(cmd.commandId);
  Serial.print(" arg=");
  Serial.println(cmd.argument);

  bool ok = false;

  if(cmd.targetType == TargetType::FUZE)
  {
    ok = forwardCommandToFuze(cmd.targetId, cmd.commandId, cmd.argument);
    sendAckForCommand(frame.header.sequence,
                      ok ? RfAckCode::OK : RfAckCode::FORWARD_FAILED,
                      cmd.targetId);
  }
  else if(cmd.targetType == TargetType::BUOY)
  {
    ok = true;
    sendAckForCommand(frame.header.sequence, RfAckCode::OK, cmd.targetId);
  }
  else
  {
    sendAckForCommand(frame.header.sequence, RfAckCode::UNSUPPORTED, cmd.targetId);
  }
}

static void processReceivedRfFrame(const uint8_t* data, size_t len)
{
  DecodedFrame frame{};
  if(!decodeFrame(data, len, frame))
  {
    Serial.println("[RF RX] invalid frame / CRC fail");
    return;
  }

  Serial.print("[RF RX] type=");
  Serial.print(messageTypeName(frame.header.type));
  Serial.print(" src=");
  Serial.print(nodeName(frame.header.src));
  Serial.print(" seq=");
  Serial.println(frame.header.sequence);

  if(frame.header.type == MessageType::COMMAND)
  {
    handleRfCommand(frame);
  }
}

static void pollRadio()
{
  if(!g_receivedFlag)
  {
    return;
  }

  g_enableInterrupt = false;
  g_receivedFlag = false;

  size_t packetLen = radio.getPacketLength();
  if(packetLen == 0 || packetLen > MAX_FRAME_SIZE)
  {
    Serial.println("[RF RX] invalid packet length");
    g_enableInterrupt = true;
    radio.startReceive();
    return;
  }

  uint8_t buffer[MAX_FRAME_SIZE];
  int state = radio.readData(buffer, packetLen);

  g_enableInterrupt = true;

  if(state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[RF RX] readData failed, code=");
    Serial.println(state);
    radio.startReceive();
    return;
  }

  Serial.print("[RF RX RAW] ");
  for(size_t i = 0; i < packetLen; ++i)
  {
    if(buffer[i] < 0x10) Serial.print('0');
    Serial.print(buffer[i], HEX);
    if(i + 1 < packetLen) Serial.print(' ');
  }
  Serial.println();

  processReceivedRfFrame(buffer, packetLen);

  state = radio.startReceive();
  if(state != RADIOLIB_ERR_NONE)
  {
    Serial.print("[RF RX] restart receive failed, code=");
    Serial.println(state);
  }
}

// ------------------------------------------------------
// Developer console
// ------------------------------------------------------

static void printHelp()
{
  printDivider();
  Serial.println("Buoy RF LoRa Developer Console");
  Serial.println("HELP");
  Serial.println("FUZES");
  Serial.println("HEARTBEAT");
  Serial.println("RESETSIM");
  Serial.println("SNAP1 / SNAP2 / SNAP3 / SNAP4");
  Serial.println();
  Serial.println("Inject packets:");
  Serial.println("DEP1 18.6");
  Serial.println("TMP2 14.2");
  Serial.println("MAG3 0.42");
  Serial.println("ACO4 12.5");
  Serial.println("STA1 3 12");
  Serial.println("DIA2 0011");
  Serial.println("ACK1 0 7");
  Serial.println();
  Serial.println("Scenarios:");
  Serial.println("SCENARIO QUIET");
  Serial.println("SCENARIO MIXED");
  Serial.println("SCENARIO MAG_BURST");
  Serial.println("SCENARIO ACO_SWEEP");
  Serial.println("STOP");
}

static void resetSimulation()
{
  for(uint8_t i = 0; i < 4; ++i)
  {
    g_fuze[i] = FuzeState{};
  }
  g_scenarioMode = ScenarioMode::OFF;
  g_scenarioStep = 0;
  Serial.println("[SIM] reset");
}

static bool parseFuzeSuffix(const String& token, const char* prefix, uint8_t& fuzeNumber)
{
  if(!token.startsWith(prefix)) return false;
  if(token.length() != strlen(prefix) + 1) return false;

  char c = token.charAt(token.length() - 1);
  if(c < '1' || c > '4') return false;

  fuzeNumber = static_cast<uint8_t>(c - '0');
  return true;
}

static void injectDepth(uint8_t fuzeNumber, float value)
{
  injectRawAcousticPacket(encodeDepth(fuseIdFromNumber(fuzeNumber), value));
}

static void injectTemp(uint8_t fuzeNumber, float value)
{
  injectRawAcousticPacket(encodeTemperatureC(fuseIdFromNumber(fuzeNumber), value));
}

static void injectMag(uint8_t fuzeNumber, float value)
{
  injectRawAcousticPacket(encodeMagEvent(fuseIdFromNumber(fuzeNumber), value));
}

static void injectAcoustic(uint8_t fuzeNumber, float value)
{
  injectRawAcousticPacket(encodeAcousticEvent(fuseIdFromNumber(fuzeNumber), value));
}

static void injectStatus(uint8_t fuzeNumber, uint8_t mode, uint8_t flags)
{
  injectRawAcousticPacket(
    encodeStatus(fuseIdFromNumber(fuzeNumber),
                 static_cast<StatusMode>(mode & 0x07),
                 flags & 0x7F)
  );
}

static void injectDiagnostic(uint8_t fuzeNumber, uint16_t diagBits)
{
  injectRawAcousticPacket(
    encodeDiagnostic(fuseIdFromNumber(fuzeNumber), diagBits & 0x03FF)
  );
}

static void injectAcousticAck(uint8_t fuzeNumber, uint8_t ackCode, uint8_t reference)
{
  injectRawAcousticPacket(
    encodeAck(fuseIdFromNumber(fuzeNumber),
              static_cast<AckCode>(ackCode & 0x0F),
              reference & 0x3F)
  );
}

static void handleScenarioCommand(const String& arg)
{
  if(arg == "QUIET")
  {
    g_scenarioMode = ScenarioMode::QUIET;
  }
  else if(arg == "MIXED")
  {
    g_scenarioMode = ScenarioMode::MIXED;
  }
  else if(arg == "MAG_BURST")
  {
    g_scenarioMode = ScenarioMode::MAG_BURST;
  }
  else if(arg == "ACO_SWEEP")
  {
    g_scenarioMode = ScenarioMode::ACO_SWEEP;
  }
  else
  {
    Serial.println("[SIM] unknown scenario");
    return;
  }

  g_scenarioStep = 0;
  g_lastScenarioMs = millis();
  Serial.print("[SIM] scenario set to ");
  Serial.println(arg);
}

static void handleUsbCommand(String line)
{
  line.trim();
  line.toUpperCase();

  if(line.length() == 0) return;

  if(line == "HELP")
  {
    printHelp();
    return;
  }

  if(line == "FUZES")
  {
    printFuzeStates();
    return;
  }

  if(line == "HEARTBEAT")
  {
    sendHeartbeat();
    return;
  }

  if(line == "RESETSIM")
  {
    resetSimulation();
    return;
  }

  if(line == "STOP")
  {
    g_scenarioMode = ScenarioMode::OFF;
    Serial.println("[SIM] scenario stopped");
    return;
  }

  if(line == "SNAP1" || line == "SNAP2" || line == "SNAP3" || line == "SNAP4")
  {
    uint8_t fuzeNumber = static_cast<uint8_t>(line.charAt(4) - '0');
    sendStatusSnapshotFrame(fuzeNumber);
    return;
  }

  if(line.startsWith("SCENARIO "))
  {
    String arg = line.substring(9);
    arg.trim();
    handleScenarioCommand(arg);
    return;
  }

  int space1 = line.indexOf(' ');
  String token1 = line;
  String token2 = "";
  String token3 = "";

  if(space1 >= 0)
  {
    token1 = line.substring(0, space1);
    String rest = line.substring(space1 + 1);
    rest.trim();

    int space2 = rest.indexOf(' ');
    if(space2 >= 0)
    {
      token2 = rest.substring(0, space2);
      token3 = rest.substring(space2 + 1);
      token3.trim();
    }
    else
    {
      token2 = rest;
    }
  }

  uint8_t fuzeNumber = 0;

  if(parseFuzeSuffix(token1, "DEP", fuzeNumber) && token2.length() > 0)
  {
    injectDepth(fuzeNumber, token2.toFloat());
    return;
  }

  if(parseFuzeSuffix(token1, "TMP", fuzeNumber) && token2.length() > 0)
  {
    injectTemp(fuzeNumber, token2.toFloat());
    return;
  }

  if(parseFuzeSuffix(token1, "MAG", fuzeNumber) && token2.length() > 0)
  {
    injectMag(fuzeNumber, token2.toFloat());
    return;
  }

  if(parseFuzeSuffix(token1, "ACO", fuzeNumber) && token2.length() > 0)
  {
    injectAcoustic(fuzeNumber, token2.toFloat());
    return;
  }

  if(parseFuzeSuffix(token1, "STA", fuzeNumber) && token2.length() > 0 && token3.length() > 0)
  {
    uint8_t mode = static_cast<uint8_t>(token2.toInt());
    uint8_t flags = static_cast<uint8_t>(strtoul(token3.c_str(), nullptr, 16));
    injectStatus(fuzeNumber, mode, flags);
    return;
  }

  if(parseFuzeSuffix(token1, "DIA", fuzeNumber) && token2.length() > 0)
  {
    uint16_t diag = static_cast<uint16_t>(strtoul(token2.c_str(), nullptr, 16));
    injectDiagnostic(fuzeNumber, diag);
    return;
  }

  if(parseFuzeSuffix(token1, "ACK", fuzeNumber) && token2.length() > 0 && token3.length() > 0)
  {
    uint8_t ackCode = static_cast<uint8_t>(token2.toInt());
    uint8_t ref = static_cast<uint8_t>(token3.toInt());
    injectAcousticAck(fuzeNumber, ackCode, ref);
    return;
  }

  Serial.println("[USB] unknown command, type HELP");
}

static void pollUsbSerial()
{
  static String line;

  while(Serial.available() > 0)
  {
    char c = static_cast<char>(Serial.read());

    if(c == '\n' || c == '\r')
    {
      if(line.length() > 0)
      {
        handleUsbCommand(line);
        line = "";
      }
    }
    else
    {
      line += c;
    }
  }
}

// ------------------------------------------------------
// Scenario engine
// ------------------------------------------------------

static void runScenarioStep()
{
  switch(g_scenarioMode)
  {
    case ScenarioMode::OFF:
      return;

    case ScenarioMode::QUIET:
    {
      uint8_t f = static_cast<uint8_t>((g_scenarioStep % 4) + 1);
      injectDepth(f, 12.0f + static_cast<float>(f));
      injectTemp(f, 13.5f + 0.2f * static_cast<float>(f));
      break;
    }

    case ScenarioMode::MIXED:
    {
      switch(g_scenarioStep % 6)
      {
        case 0: injectDepth(1, 18.6f); break;
        case 1: injectTemp(2, 14.2f); break;
        case 2: injectMag(3, 0.42f); break;
        case 3: injectAcoustic(4, 12.5f); break;
        case 4: injectStatus(1, 3, 0x12); break;
        case 5: injectDiagnostic(2, 0x0011); break;
      }
      break;
    }

    case ScenarioMode::MAG_BURST:
    {
      uint8_t f = static_cast<uint8_t>((g_scenarioStep % 4) + 1);
      float mag = 0.15f + 0.10f * static_cast<float>(g_scenarioStep % 6);
      injectMag(f, mag);
      break;
    }

    case ScenarioMode::ACO_SWEEP:
    {
      uint8_t f = static_cast<uint8_t>((g_scenarioStep % 4) + 1);
      float aco = 4.0f + 2.5f * static_cast<float>(g_scenarioStep % 8);
      injectAcoustic(f, aco);
      break;
    }
  }

  ++g_scenarioStep;
}

static void pollScenarioEngine()
{
  if(g_scenarioMode == ScenarioMode::OFF) return;

  uint32_t now = millis();
  if(now - g_lastScenarioMs >= SCENARIO_INTERVAL_MS)
  {
    g_lastScenarioMs = now;
    runScenarioStep();
  }
}

// ------------------------------------------------------
// Setup / loop
// ------------------------------------------------------

void setup()
{
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial1.begin(ACOUSTIC_BAUD);

  delay(500);
  printDivider();
  Serial.println("Buoy_RF_LoRa_Test starting");

int state = radio.begin(915.0, 500.0, 10, 7);

if(state != RADIOLIB_ERR_NONE)
{
  Serial.print("Radio init failed, code=");
  Serial.println(state);
  while(true) { delay(1000); }
}

radio.setDio1Action(setReceivedFlag);

state = radio.startReceive();
if(state != RADIOLIB_ERR_NONE)
{
  Serial.print("startReceive failed, code=");
  Serial.println(state);
  while(true) { delay(1000); }
}

int txPower = 22;
if (radio.setOutputPower(txPower) == RADIOLIB_ERR_NONE) {
  Serial.println("Power set to +22 dBm (max)");
} else {
  Serial.println("Failed to set power!");
}

  printHelp();
}

void loop()
{
  pollUsbSerial();
  pollAcousticSerial();
  pollRadio();
  pollScenarioEngine();

  const uint32_t now = millis();
  if(now - g_lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)
  {
    g_lastHeartbeatMs = now;
    sendHeartbeat();
  }
}
