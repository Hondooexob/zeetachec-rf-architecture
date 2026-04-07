# Contributing

## Purpose

This repository defines the RF communication architecture for the Zeetachec Maritime Sensor System.

It contains:
- RF protocol specifications
- acoustic protocol support library
- RF protocol library
- buoy-side reference test firmware
- controller-side reference test firmware

## Contribution Rules

### 1. Do not change locked protocol structure
Do not modify any of the following without explicit approval:

- RF frame structure
- message type IDs
- payload layout
- reliability rules
- snapshot behavior

See:
- `docs/PROTOCOL_LOCK.md`
- `docs/rf_architecture_decisions.md`
- `docs/rf_protocol.md`

### 2. Keep layers separated
Maintain separation between:

- protocol libraries (`/lib`)
- reference test firmware (`/test`)
- future production firmware (`/firmware`)
- documentation (`/docs`)

### 3. RF is the master contract
Acoustic messages map into RF messages.

Do not redesign the system so that RF depends on acoustic structure.

### 4. Prefer small changes
Contributions should be narrow and clear:
- bug fix
- refactor
- documentation improvement
- test improvement
- new scoped feature

Avoid broad rewrites.

### 5. Preserve binary efficiency
This protocol is LoRa-oriented.

Do not introduce:
- JSON payloads
- verbose text protocols
- dynamic message formats
unless explicitly requested.

### 6. Reliable message handling
The following message types are reliable/tracked:

- COMMAND
- EVENT
- ACK

All other message types are best-effort unless explicitly requested.

### 7. Snapshot behavior
Controller requests `STATUS_SNAPSHOT`:
- on UI connect
- on reconnect
- on manual refresh

Do not change this behavior without approval.

## Recommended Workflow

1. Read `README.md`
2. Read `docs/rf_architecture_decisions.md`
3. Read `docs/rf_protocol.md`
4. Make a small scoped change
5. Keep protocol compatibility intact

## Good Contribution Examples

- add retry manager for reliable RF messages
- improve controller state tracking
- improve buoy simulation console
- improve docs
- scaffold ESP32 controller structure

## Bad Contribution Examples

- redesign packet format
- replace binary RF with JSON
- remove CRC
- change message IDs
- merge protocol and UI layers together
