#include <Arduino.h>
#include <RadioLib.h>
#include "ZeetaRfLink.h"

// ======================================================
// Real LoRa RF Controller Test Sketch
//
// Purpose:
// - Receive ZeetaRfLink frames over SX1262 LoRa
// - Decode and print HEARTBEAT / EVENT / DELTA / ACK
// - Maintain a simple per-fuze state table
// - Send COMMAND frames back to the buoy from USB serial
//
// USB Serial Commands:
//   HELP
//   PING
//   CAL1
//   ACT2
//   REC3
//   STO4
//   RON1
//   ROF2
//   STA3
//   DIA4
//
// Expected ZeetaRfLink frame format:
//   SOF | Version | Type | Src | Dst | Seq | Len | Payload | CRC16
// ======================================================

using namespace ZeetaRfLink;

// ------------------------------------------------------
// SX1262 pinout
// ------------------------------------------------------

static constexpr int LORA_NSS  = 10;
static constexpr int LORA_DIO1 = 7;
static constexpr int LORA_RST  = 9;
static constexpr int LORA_BUSY = 8;

// RadioLib SX1262 object
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ------------------------------------------------------
// LoRa settings
// Adjust to match buoy side exactly
// ------------------------------------------------------

static constexpr float LORA_FREQ_MHZ = 915.0;
static constexpr float LORA_BW_KHZ   = 125.0;
static constexpr uint8_t LORA_SF     = 11;
static constexpr uint8_t LORA_CR     = 7;     // 4/7 in RadioLib style
static constexpr uint8_t LORA_SYNC   = 0x12;
static constexpr int8_t LORA_POWER   = 22;
static constexpr uint16_t LORA_PREAMBLE = 8;

// ------------------------------------------------------
// RF node identity
// ------------------------------------------------------

static constexpr NodeId THIS_NODE = NodeId::RF_CONTROLLER_1;
static constexpr NodeId BUOY_NODE = NodeId::BUOY_1;

// ------------------------------------------------------
// Serial settings
// ------------------------------------------------------

static constexpr uint32_t SERIAL_BAUD = 115200;

// ------------------------------------------------------
// RX state
// ------------------------------------------------------

volatile bool g_receivedFlag = false;
volatile bool g_enableInterrupt = true;

// ------------------------------------------------------
// Simple controller-side fuze state
// ------------------------------------------------------

struct FuzeUiState {
  bool seen = false;
  uint32_t lastUpdateMs = 0;

  uint16_t depth_cm = 0;
  uint16_t temp_centiC = 0;
  uint16_t mag_nT = 0;
  uint16_t acoustic_centiDb = 0;
  uint8_t status = 0;
  uint16_t diagnostic = 0;
};

FuzeUiState g_fuze[4];

// ------------------------------------------------------
// IRQ callback
// ------------------------------------------------------

void setReceivedFlag(void) {
  if(!g_enableInterrupt) {
    return;
  }
  g_receivedFlag = true;
}

// ------------------------------------------------------
// Helpers
// ------------------------------------------------------

static void printDivider() {
  Serial.println("----------------------------------------");
}

static bool validFuzeId(uint8_t fuzeId) {
  return fuzeId >= 1 && fuzeId <= 4;
}

static FuzeUiState* getFuzeState(uint8_t fuzeId) {
  if(!validFuzeId(fuzeId)) return nullptr;
  return &g_fuze[fuzeId - 1];
}

static void markSeen(uint8_t fuzeId) {
  FuzeUiState* s = getFuzeState(fuzeId);
  if(!s) return;
  s->seen = true;
  s->lastUpdateMs = millis();
}

static float depthMetersFromCm(uint16_t cm) {
  return static_cast<float>(cm) / 100.0f;
}

static float tempCFromCentiC(uint16_t centiC) {
  return (static_cast<float>(centiC) / 100.0f) - 100.0f;
}

static float acousticDbFromCentiDb(uint16_t centiDb) {
  return static_cast<float>(centiDb) / 100.0f;
}

static void printFuzeTable() {
  printDivider();
  Serial.println("Fuze State Table");
  for(uint8_t i = 0; i < 4; ++i) {
    const FuzeUiState& s = g_fuze[i];
    Serial.print("Fuze ");
    Serial.print(i + 1);
    Serial.print(": seen=");
    Serial.print(s.seen ? "Y" : "N");
    Serial.print(" depth=");
    Serial.print(depthMetersFromCm(s.depth_cm), 2);
    Serial.print("m temp=");
    Serial.print(tempCFromCentiC(s.temp_centiC), 2);
    Serial.print("C mag=");
    Serial.print(s.mag_nT);
    Serial.print("nT acoustic=");
    Serial.print(acousticDbFromCentiDb(s.acoustic_centiDb), 2);
    Serial.print("dB status=0x");
    Serial.print(s.status, HEX);
    Serial.print(" diag=0x");
    Serial.println(s.diagnostic, HEX);
  }
}

static void printHelp() {
  printDivider();
  Serial.println("RF Controller LoRa Test Commands:");
  Serial.println("  HELP");
  Serial.println("  PING");
  Serial.println("  CAL1  ACT2  REC3  STO4");
  Serial.println("  RON1  ROF2");
  Serial.println("  STA3  DIA4");
  Serial.println("  FUZES");
}

// ------------------------------------------------------
// TX helpers
// ------------------------------------------------------

static bool transmitFrame(const Frame& frame, const char* label) {
  Serial.print("[RF TX ");
  Serial.print(label);
  Serial.print("] ");
  hexDumpFrame(Serial, frame);

  g_enableInterrupt = false;
  g_receivedFlag = false;

  int state = radio.transmit(frame.data, frame.length);

  g_enableInterrupt = true;

  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("[RF TX] transmit failed, code=");
    Serial.println(state);
    radio.startReceive();
    return false;
  }

  state = radio.startReceive();
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("[RF RX] startReceive failed, code=");
    Serial.println(state);
    return false;
  }

  return true;
}

static bool sendPing() {
  Frame frame{};
  bool ok = encodeFrame(
    MessageType::PING,
    THIS_NODE,
    BUOY_NODE,
    nextSequence(),
    nullptr,
    0,
    frame
  );

  if(!ok) return false;
  return transmitFrame(frame, "PING");
}

static bool sendFuzeCommand(uint8_t fuzeId, uint8_t commandId, uint8_t arg) {
  if(!validFuzeId(fuzeId)) {
    Serial.println("[CMD] invalid fuze id");
    return false;
  }

  CommandPayload cmd{};
  cmd.targetType = TargetType::FUZE;
  cmd.targetId = fuzeId;
  cmd.commandId = commandId;
  cmd.argument = arg;
  cmd.commandFlags = 0x01; // ACK required

  Frame frame{};
  bool ok = buildCommand(
    THIS_NODE,
    BUOY_NODE,
    nextSequence(),
    cmd,
    frame
  );

  if(!ok) return false;
  return transmitFrame(frame, "COMMAND");
}

// ------------------------------------------------------
// RX decode / print
// ------------------------------------------------------

static void handleHeartbeat(const HeartbeatPayload& hb) {
  printDivider();
  Serial.println("[HEARTBEAT]");
  Serial.print("Buoy Flags: 0x");
  Serial.println(hb.buoyFlags, HEX);
  Serial.print("Last Acoustic Age (s): ");
  Serial.println(hb.lastAcousticAgeSec);
  Serial.print("GPS Flags: 0x");
  Serial.println(hb.gpsFlags, HEX);
  Serial.print("RF Link Quality: ");
  Serial.println(hb.rfLinkQuality);
  Serial.print("Active Fuze Bitmap: 0x");
  Serial.println(hb.activeFuzeBitmap, HEX);
}

static void handleEvent(const EventPayload& ev) {
  printDivider();
  Serial.println("[EVENT]");
  Serial.print("Fuze ID: ");
  Serial.println(ev.fuzeId);
  Serial.print("Acoustic Function: 0x");
  Serial.println(ev.acousticFunction, HEX);
  Serial.print("Acoustic Raw: 0x");
  Serial.println(ev.acousticRaw, HEX);
  Serial.print("Decoded Value: ");
  Serial.println(ev.decodedValue);
  Serial.print("Event Flags: 0x");
  Serial.println(ev.eventFlags, HEX);
  Serial.print("Age (100 ms): ");
  Serial.println(ev.age100ms);

  FuzeUiState* s = getFuzeState(ev.fuzeId);
  if(s) {
    markSeen(ev.fuzeId);

    // 0x02 = MAG_EVENT, 0x03 = ACOUSTIC_EVENT
    if(ev.acousticFunction == 0x02) {
      s->mag_nT = ev.decodedValue;
    } else if(ev.acousticFunction == 0x03) {
      s->acoustic_centiDb = ev.decodedValue;
    }
  }
}

static void handleTelemetryDelta(const TelemetryDeltaState& td) {
  printDivider();
  Serial.println("[TELEMETRY_DELTA]");
  Serial.print("Fuze ID: ");
  Serial.println(td.fuzeId);
  Serial.print("Field Mask: 0x");
  Serial.println(td.fieldMask, HEX);

  FuzeUiState* s = getFuzeState(td.fuzeId);
  if(s) {
    markSeen(td.fuzeId);

    if(td.fieldMask & (1u << 0)) {
      s->depth_cm = td.depth_cm;
      Serial.print("Depth: ");
      Serial.print(depthMetersFromCm(td.depth_cm), 2);
      Serial.println(" m");
    }

    if(td.fieldMask & (1u << 1)) {
      s->temp_centiC = td.temp_centiC;
      Serial.print("Temp: ");
      Serial.print(tempCFromCentiC(td.temp_centiC), 2);
      Serial.println(" C");
    }

    if(td.fieldMask & (1u << 2)) {
      s->mag_nT = td.mag_nT;
      Serial.print("Mag: ");
      Serial.print(td.mag_nT);
      Serial.println(" nT");
    }

    if(td.fieldMask & (1u << 3)) {
      s->acoustic_centiDb = td.acoustic_centiDb;
      Serial.print("Acoustic: ");
      Serial.print(acousticDbFromCentiDb(td.acoustic_centiDb), 2);
      Serial.println(" dB");
    }

    if(td.fieldMask & (1u << 4)) {
      s->status = td.status;
      Serial.print("Status: 0x");
      Serial.println(td.status, HEX);
    }

    if(td.fieldMask & (1u << 5)) {
      s->diagnostic = td.diagnostic;
      Serial.print("Diagnostic: 0x");
      Serial.println(td.diagnostic, HEX);
    }
  }
}

static void handleAck(const AckPayload& ack) {
  printDivider();
  Serial.println("[ACK]");
  Serial.print("Acked Message Type: 0x");
  Serial.println(ack.ackedMessageType, HEX);
  Serial.print("Acked Sequence: ");
  Serial.println(ack.ackedSequence);
  Serial.print("ACK Code: ");
  Serial.println(rfAckName(ack.ackCode));
  Serial.print("Detail: ");
  Serial.println(ack.detail);
}

static void handleDiagnostic(const DiagnosticPayload& diag) {
  printDivider();
  Serial.println("[DIAGNOSTIC]");
  Serial.print("Target Type: ");
  Serial.println(diag.targetType);
  Serial.print("Target ID: ");
  Serial.println(diag.targetId);
  Serial.print("Diagnostic Bits: 0x");
  Serial.println(diag.diagnosticBits, HEX);
  Serial.print("Extra Flags: 0x");
  Serial.println(diag.extraFlags, HEX);
}

static void handleStatusSnapshot(const StatusSnapshotPayload& ss) {
  printDivider();
  Serial.println("[STATUS_SNAPSHOT]");
  Serial.print("Fuze ID: ");
  Serial.println(ss.fuzeId);
  Serial.print("Status: 0x");
  Serial.println(ss.status, HEX);
  Serial.print("Depth: ");
  Serial.print(depthMetersFromCm(ss.depth_cm), 2);
  Serial.println(" m");
  Serial.print("Temp: ");
  Serial.print(tempCFromCentiC(ss.temp_centiC), 2);
  Serial.println(" C");
  Serial.print("Mag: ");
  Serial.print(ss.magnetic_nT);
  Serial.println(" nT");
  Serial.print("Acoustic: ");
  Serial.print(acousticDbFromCentiDb(ss.acoustic_centiDb), 2);
  Serial.println(" dB");
  Serial.print("Diagnostic Bits: 0x");
  Serial.println(ss.diagnosticBits, HEX);

  FuzeUiState* s = getFuzeState(ss.fuzeId);
  if(s) {
    markSeen(ss.fuzeId);
    s->status = ss.status;
    s->depth_cm = ss.depth_cm;
    s->temp_centiC = ss.temp_centiC;
    s->mag_nT = ss.magnetic_nT;
    s->acoustic_centiDb = ss.acoustic_centiDb;
    s->diagnostic = ss.diagnosticBits;
  }
}

static void processReceivedFrame(const uint8_t* data, size_t len) {
  DecodedFrame frame{};
  if(!decodeFrame(data, len, frame)) {
    Serial.println("[RF RX] invalid frame / CRC fail");
    return;
  }

  Serial.print("[RF RX] type=");
  Serial.print(messageTypeName(frame.header.type));
  Serial.print(" src=");
  Serial.print(nodeName(frame.header.src));
  Serial.print(" seq=");
  Serial.println(frame.header.sequence);

  switch(frame.header.type) {
    case MessageType::HEARTBEAT: {
      HeartbeatPayload hb{};
      if(parseHeartbeat(frame, hb)) handleHeartbeat(hb);
      break;
    }

    case MessageType::EVENT: {
      EventPayload ev{};
      if(parseEvent(frame, ev)) handleEvent(ev);
      break;
    }

    case MessageType::TELEMETRY_DELTA: {
      TelemetryDeltaState td{};
      if(parseTelemetryDelta(frame, td)) handleTelemetryDelta(td);
      break;
    }

    case MessageType::ACK: {
      AckPayload ack{};
      if(parseAck(frame, ack)) handleAck(ack);
      break;
    }

    case MessageType::DIAGNOSTIC: {
      DiagnosticPayload diag{};
      if(parseDiagnostic(frame, diag)) handleDiagnostic(diag);
      break;
    }

    case MessageType::STATUS_SNAPSHOT: {
      StatusSnapshotPayload ss{};
      if(parseStatusSnapshot(frame, ss)) handleStatusSnapshot(ss);
      break;
    }

    default:
      printDivider();
      Serial.println("[UNHANDLED FRAME TYPE]");
      break;
  }
}

// ------------------------------------------------------
// LoRa receive loop
// ------------------------------------------------------

static void pollRadio() {
  if(!g_receivedFlag) {
    return;
  }

  g_enableInterrupt = false;
  g_receivedFlag = false;

  size_t packetLen = radio.getPacketLength();
  if(packetLen == 0 || packetLen > MAX_FRAME_SIZE) {
    Serial.println("[RF RX] invalid packet length");
    g_enableInterrupt = true;
    radio.startReceive();
    return;
  }

  uint8_t buffer[MAX_FRAME_SIZE];
  int state = radio.readData(buffer, packetLen);

  g_enableInterrupt = true;

  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("[RF RX] readData failed, code=");
    Serial.println(state);
    radio.startReceive();
    return;
  }

  Serial.print("[RF RX RAW] ");
  for(size_t i = 0; i < packetLen; ++i) {
    if(buffer[i] < 0x10) Serial.print('0');
    Serial.print(buffer[i], HEX);
    if(i + 1 < packetLen) Serial.print(' ');
  }
  Serial.println();

  processReceivedFrame(buffer, packetLen);

  state = radio.startReceive();
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("[RF RX] restart receive failed, code=");
    Serial.println(state);
  }
}

// ------------------------------------------------------
// USB serial command parser
// ------------------------------------------------------

static void handleUserCommand(String input) {
  input.trim();
  input.toUpperCase();

  if(input.length() == 0) return;

  if(input == "HELP") {
    printHelp();
    return;
  }

  if(input == "FUZES") {
    printFuzeTable();
    return;
  }

  if(input == "PING") {
    sendPing();
    return;
  }

  if(input.length() != 4) {
    Serial.println("[CMD] unknown format, type HELP");
    return;
  }

  String op = input.substring(0, 3);
  char fuzeChar = input.charAt(3);

  if(fuzeChar < '1' || fuzeChar > '4') {
    Serial.println("[CMD] invalid fuze id");
    return;
  }

  uint8_t fuzeId = static_cast<uint8_t>(fuzeChar - '0');

  if(op == "CAL") {
    sendFuzeCommand(fuzeId, 0x01, 0x00);
  } else if(op == "ACT") {
    sendFuzeCommand(fuzeId, 0x02, 0x00);
  } else if(op == "REC") {
    sendFuzeCommand(fuzeId, 0x03, 0x00);
  } else if(op == "STO") {
    sendFuzeCommand(fuzeId, 0x04, 0x00);
  } else if(op == "RON") {
    sendFuzeCommand(fuzeId, 0x05, 0x00);
  } else if(op == "ROF") {
    sendFuzeCommand(fuzeId, 0x06, 0x00);
  } else if(op == "STA") {
    sendFuzeCommand(fuzeId, 0x07, 0x00);
  } else if(op == "DIA") {
    sendFuzeCommand(fuzeId, 0x08, 0x00);
  } else {
    Serial.println("[CMD] unknown command, type HELP");
  }
}

static void pollUsbSerial() {
  static String line;

  while(Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if(c == '\n' || c == '\r') {
      if(line.length() > 0) {
        handleUserCommand(line);
        line = "";
      }
    } else {
      line += c;
    }
  }
}

// ------------------------------------------------------
// Setup / loop
// ------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  printDivider();
  Serial.println("RF_Controller_LoRa_Test starting");

int state = radio.begin(915.0, 500.0, 10, 7);

  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("Radio init failed, code=");
    Serial.println(state);
    while(true) { delay(1000); }
  }

  radio.setDio1Action(setReceivedFlag);

  int txPower = 22;
  if (radio.setOutputPower(txPower) == RADIOLIB_ERR_NONE) {
    Serial.println("Power set to +22 dBm (max)");
  } else {
    Serial.println("Failed to set power!");
  }

  state = radio.startReceive();
  if(state != RADIOLIB_ERR_NONE) {
    Serial.print("startReceive failed, code=");
    Serial.println(state);
    while(true) { delay(1000); }
  }

  printHelp();
}

void loop() {
  pollRadio();
  pollUsbSerial();
}
