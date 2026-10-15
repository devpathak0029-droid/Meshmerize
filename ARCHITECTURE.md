# Techfest Meshmerize 2026-27 Robot Architecture

## Competition Overview
- **Event:** Techfest 2026-27, IIT Bombay - Meshmerize
- **Arena Dimensions:** 230 cm x 230 cm (Angle between lines: 90°, Stripe width: 30 mm, White on black background)
- **End Zone:** Solid 400 mm x 400 mm white box (must light a red LED upon sensing)
- **Checkpoints:** 25 points each (Reference points only; no physical markers)
- **Runs:**
  1. **Dry Run (Max 3 min):** Traverses the maze, maps junctions, learns shortest path, illuminates Red LED at end zone.
  2. **Actual Run (Max 2 min 30 sec):** Retraces the optimal shortest path at high speed.
- **Bot Size Limit:** 220 mm x 220 mm x 220 mm; Max voltage <= 24V.

## Hardware Stack Configuration (ESP32)
- **Microcontroller:** ESP32 (30-pin or 38-pin DevKit V1)
- **Sensor Array:** 8-channel IR reflectance sensor array (e.g. QTR-8A / QTR-8RC / analog IR bar) + optional outer wing sensors
- **Motor Driver:** TB6612FNG Dual H-Bridge (PWM on ESP32 LEDC channels)
- **Motors:** N20 Micro Metal Gearmotors (6V/12V, 500-1000 RPM)
- **Power:** 2S / 3S LiPo (7.4V / 11.1V) with LM2596 / Buck converter for 5V/3.3V logic
- **Indicator:** Red LED on End Zone detection (mandatory rule)
- **Buttons / Switches:** Calibration button, Mode switch (Dry Run / Actual Run)

## Modular Software Directory Structure
```
d:\TechFest\
│── include\
│   ├── Config.h             # Pin definitions, sensor thresholds, PID constants, speeds
│   ├── PIDController.h      # High-speed PID steering algorithm
│   ├── SensorArray.h        # Calibration, normalization, line position (0-7000), junction recognition
│   ├── MotionControl.h      # Dual motor driver (TB6612FNG via ESP32 LEDC), closed-loop turns, inching
│   ├── MazeSolver.h         # Left-Hand-Rule exploration, LSRB path simplification & loop reduction
│── src\
│   ├── PIDController.cpp
│   ├── SensorArray.cpp
│   ├── MotionControl.cpp
│   ├── MazeSolver.cpp
│   ├── main.cpp             # State machine (Calibration -> Dry Run -> Path Optimization -> Actual Run)
│── test\
│   ├── maze_sim.py          # Python visualizer and unit tests verifying the exact Techfest 2026-27 arena
│── README.md
```
