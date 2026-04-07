# RF Architecture Decisions

## 1. RF is the master protocol
Acoustic messages map into RF messages.

## 2. Controller uses a richer internal model
RF packets are not directly exposed to UI.

## 3. Reliable messages
The following require delivery tracking:
- COMMAND
- EVENT
- ACK

## 4. Best-effort messages
- HEARTBEAT
- TELEMETRY_DELTA
- DIAGNOSTIC

## 5. Snapshot behavior
Controller requests STATUS_SNAPSHOT:
- on phone/web client connect
- on reconnect
- on manual refresh

## 6. Protocol stability
Packet format is locked unless explicitly changed.

## 7. Reliable Message Behavior

The following message types are reliable/tracked:

- COMMAND
- EVENT
- ACK

### Required behavior

- Sender assigns sequence number
- Receiver returns ACK
- Sender retries on timeout
- Receiver suppresses duplicates
- Controller tracks pending reliable transactions

### Initial implementation guidance

- Keep retry count small
- Keep timeout deterministic
- Do not allow duplicate events to create multiple UI alerts unless explicitly intended
- Preserve event ordering as much as possible within LoRa constraints

### Snapshot interaction

If controller reconnects or a phone/web client connects, controller should request `STATUS_SNAPSHOT` to rebuild current state cleanly.
