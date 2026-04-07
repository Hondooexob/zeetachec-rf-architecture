# Zeetachec RF Protocol Specification

Version: 1.0  
Status: LOCKED (see PROTOCOL_LOCK.md)

---

# 1. Overview

This document defines the RF communication protocol between:

- Buoy (gateway node)
- RF Controller (operator interface)

The RF protocol is the **primary system contract**.  
All acoustic (ACOMMS) data is mapped into this RF format.

---

# 2. Design Goals

- Low bandwidth efficiency (LoRa optimized)
- Deterministic binary format
- Multi-fuze support (up to 4 per buoy)
- Reliable command/event delivery
- Expandable for SATCOM and future systems

---

# 3. Frame Structure

All RF messages use the following binary frame:


SOF | Version | Type | Src | Dst | Seq | Len | Payload | CRC16


### Field Definitions

| Field    | Size | Description |
|----------|------|-------------|
| SOF      | 1B   | Start of Frame (0xA5) |
| Version  | 1B   | Protocol version (0x01) |
| Type     | 1B   | Message type |
| Src      | 1B   | Source node ID |
| Dst      | 1B   | Destination node ID |
| Seq      | 2B   | Sequence number (big endian) |
| Len      | 1B   | Payload length (bytes) |
| Payload  | N    | Message-specific payload |
| CRC16    | 2B   | CCITT-FALSE CRC |

---

# 4. Node IDs

| Node              | Value |
|------------------|------|
| BUOY_1           | 0x10 |
| BUOY_2           | 0x11 |
| RF_CONTROLLER_1  | 0x20 |
| SATCOM_MODULE    | 0x30 |
| BROADCAST        | 0xFF |

---

# 5. Message Types

| Type | Name              | Reliable |
|------|-------------------|----------|
| 0x01 | HEARTBEAT         | No       |
| 0x02 | TELEMETRY_DELTA   | No       |
| 0x03 | EVENT             | YES      |
| 0x04 | COMMAND           | YES      |
| 0x05 | ACK               | YES      |
| 0x06 | DIAGNOSTIC        | No       |
| 0x07 | STATUS_SNAPSHOT   | On request |
| 0x08 | SATCOM_STATUS     | Future   |
| 0x09 | SATCOM_COMMAND    | Future   |
| 0x0A | PING              | Optional |

---

# 6. Reliability Rules

## Guaranteed Delivery

The following must be tracked and retried:

- COMMAND
- EVENT
- ACK

### Behavior

- Sender assigns sequence number
- Receiver sends ACK
- Sender retries if timeout
- Receiver must ignore duplicates

---

## Best Effort

No retries required:

- HEARTBEAT
- TELEMETRY_DELTA
- DIAGNOSTIC

---

# 7. Payload Definitions

---

## 7.1 HEARTBEAT


[flags][age][gps][rf_quality][fuze_bitmap]


| Field | Description |
|------|-------------|
| flags | Buoy system flags |
| age | Seconds since last acoustic packet |
| gps | GPS status |
| rf_quality | Link quality indicator |
| fuze_bitmap | Active fuzes |

---

## 7.2 EVENT


[fuzeId][function][raw][decoded][flags][age]


| Field | Description |
|------|-------------|
| fuzeId | 1–4 |
| function | Acoustic event type |
| raw | Raw 16-bit acoustic packet |
| decoded | Converted value |
| flags | Event flags |
| age | Age in 100ms units |

### Notes
- Critical events must be delivered reliably
- Used for threshold crossings (mag/acoustic)

---

## 7.3 TELEMETRY_DELTA


[fuzeId][fieldMask][fields...]


Field mask bits:

| Bit | Field |
|-----|------|
| 0 | Depth (cm) |
| 1 | Temperature (centi-C) |
| 2 | Magnetic (nT) |
| 3 | Acoustic (centi-dB) |
| 4 | Status |
| 5 | Diagnostic |

---

## 7.4 COMMAND


[targetType][targetId][commandId][argument][flags]


| Field | Description |
|------|-------------|
| targetType | BUOY / FUZE / SATCOM |
| targetId | Target index |
| commandId | Command enum |
| argument | Optional parameter |
| flags | Control flags |

---

## 7.5 ACK


[msgType][sequence][ackCode][detail]


| Field | Description |
|------|-------------|
| msgType | Message being acknowledged |
| sequence | Sequence number |
| ackCode | Result |
| detail | Optional info |

---

## 7.6 DIAGNOSTIC


[targetType][targetId][diagBits][flags]


---

## 7.7 STATUS_SNAPSHOT


[fuzeId][status][depth][temp][mag][acoustic][diag]


### Usage

- Requested by controller
- Used to populate UI immediately

---

# 8. Snapshot Behavior

Controller must request snapshot:

- On UI connect (WiFi/phone)
- On reconnect
- On operator request

---

# 9. Controller Internal Model

Controller must maintain:

Per-fuze:
- depth
- temperature
- magnetic
- acoustic
- status
- diagnostics
- last update time

System:
- RF link health
- heartbeat age
- active fuzes

---

# 10. Future Expansion

Reserved message types:

- SATCOM_STATUS
- SATCOM_COMMAND

---

# 11. Notes

- Binary protocol only (no JSON)
- Big-endian encoding
- Max payload: 64 bytes
- CRC16 required on all frames

---

# END OF SPEC
