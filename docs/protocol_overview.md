# Protocol Overview

## System Purpose

The Zeetachec Maritime Sensor System uses two communication layers:

1. Acoustic communications between fuze and buoy
2. RF communications between buoy and RF controller

This repository focuses primarily on the **RF architecture**, which is the main system contract for operator-facing control, telemetry, event delivery, and future WiFi/web integration.

---

## System Flow

```text
Fuze
  ↓ acoustic packet
Buoy
  ↓ RF packet over LoRa
RF Controller
  ↓ richer internal model
WiFi / Web App / Phone
