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
