# CLAUDE.md - Coding Agent Reference Guide for Meshmerize

## Project Overview
Autonomous line-follower and maze-solving robot designed for **Meshmerize (Techfest 2026-27, IIT Bombay)**.
- **Controller:** ESP32 DevKit V1 (Tensilica Xtensa Dual-Core 240MHz)
- **Framework:** Arduino via PlatformIO
- **Sensors:** 8-channel analog IR reflectance sensor array
- **Driver:** TB6612FNG Dual H-Bridge motor driver with active braking
- **Power:** Onboard LiPo (max potential difference <= 24V)

## Essential Build & Test Commands
```bash
# Add PlatformIO to PATH (if in fresh Windows terminal session)
$env:PATH = "C:\Users\Dev\AppData\Local\Python\pythoncore-3.14-64\Scripts;" + $env:PATH

# Compile ESP32 firmware
pio run

# Upload to connected ESP32
pio run --target upload

# Run serial monitor
pio device monitor -b 115200

# Run host-side algorithm test suite
python test/maze_sim.py
```

## Architectural Rules & Coding Standards
1. **Never use blind delays for turning:** Always use closed-loop sensor feedback (`MotionControl::turnLeft90`, etc.) where the center sensor re-acquires the perpendicular line, backed by a safety timeout (`TURN_TIMEOUT_MS`).
2. **ADC Constraints:** Classic ESP32 ADC1 comprises GPIOs 32-39. GPIOs 25 and 26 are ADC2 channels and cannot be read when Wi-Fi/Bluetooth is active. Wireless features must remain disabled.
3. **Strapping Pins:** Be aware of GPIO 0, 2, 5, 12, and 15 bootstrap behavior.
4. **PID Protection:** Always clamp time intervals (`dt <= 0.05s`) after blocking maneuvers and suppress first-run derivative kick.
5. **Junction Clearance:** Always clear the transverse intersection line (`CLEAR_JUNCTION_TIME_MS`) or check anti-thrashing locks to prevent duplicate junction registrations.
6. **No Fake Claims:** Do not claim code runs on physical hardware unless verified via serial logs.
