# Meshmerize - Autonomous Maze Solving Robot

Autonomous line-following and maze-solving robot firmware engineered for **Meshmerize (Techfest 2026-27, IIT Bombay)** using an ESP32 microcontroller.

---

## 🏆 Competition Overview
* **Arena Dimensions:** $230\text{ cm} \times 230\text{ cm}$
* **Path Details:** $30\text{ mm}$ white vinyl strips on a black surface, $90^\circ$ intersections.
* **End Zone:** Solid $400\text{ mm} \times 400\text{ mm}$ white box.
* **LED Rule:** The bot must illuminate a Red LED upon reaching the End Zone ($L = 5\text{ pts}$).
* **Two-Pass Challenge:**
  1. **Dry Run (Max 3 min):** Fully autonomous exploration, mapping junctions, recording turns, and locating the end zone.
  2. **Actual Run (Max 2 min 30 sec):** Fast-paced traversal from start to end zone following the mathematically optimized shortest path.
* **Scoring Formula:**
  $$\text{Total} = (A + B + C + S + L + T) - P$$
  * $A = 25 \times \text{Checkpoints covered}$
  * $B = 30\text{ pts (Dry Run completion)}$
  * $C = 180 - \text{DryRunTime (seconds)}$
  * $S = 30\text{ pts (Shortest path bonus)}$
  * $T = 150 - \text{ActualRunTime (seconds)}$
  * $L = 5\text{ pts (End zone LED)}$
  * $P = \text{Penalties}$

---

## ⚙️ Hardware Stack & Architecture
* **Microcontroller:** ESP32 (30-pin / 38-pin DevKit V1)
* **Line Sensor Array:** 8-channel IR reflectance sensor array (ADC1 pins: GPIO 36, 39, 34, 35, 32, 33, 25, 26)
* **Motor Driver:** TB6612FNG Dual H-Bridge Driver
* **Motors:** N20 Micro Metal Gearmotors (6V / 12V)
* **PWM Frequency:** ESP32 hardware LEDC channels at $20\text{ kHz}$ (silent motor operation, no coil whining)
* **Power Supply:** 2S / 3S LiPo battery with buck converter for $5\text{V} / 3.3\text{V}$ logic
* **Indicators & Controls:**
  * GPIO 2: Mandatory Red End-Zone LED
  * GPIO 15: Push-button for automated on-track sensor calibration sweep
  * GPIO 13: Mode toggle switch (LOW = Dry Run exploration, HIGH = Actual Run race)

---

## 🚀 Key Technical Enhancements over Generic Repositories

| Generic Competitor Repos | Our Implementation |
| :--- | :--- |
| **Blind Turn Delays:** Uses hardcoded `delay(350)` for 90° turns, leading to overshooting/undershooting when battery drains. | **Closed-Loop Sensor-Assisted Turning:** Bot pivots actively until its center IR sensors re-lock onto the line with debounce timeouts. |
| **False End-Zone Triggers:** A 4-way cross ($30\text{ mm}$) trips all sensors white and is mistaken for the End Zone. | **Forward Inching Verification:** Inches forward $40\text{ mm}$ upon reading all-white. Only solid $400\text{ mm}$ boxes register as End Zone; 4-way crosses are handled smoothly. |
| **Simple LHR Dead-End Traps:** Loops and redundant detours degrade performance. | **Recursive LSRB Simplification:** Trims triplets (`LBL` $\rightarrow$ `S`, `LBS` $\rightarrow$ `R`, `LBR` $\rightarrow$ `B`, `SBL` $\rightarrow$ `R`, `SBS` $\rightarrow$ `B`, `RBL` $\rightarrow$ `B`) both on the fly and post-exploration. |
| **Static Single Speed:** Runs slow during the race or too fast during exploration. | **Dual Speed Profiles:** Controlled speed (`PWM 140`) during mapping to ensure 100% path capture, maximum speed (`PWM 200-255`) on the Actual Run. |

---

## 📁 Repository Directory Structure

```
Meshmerize/
├── include/
│   ├── Config.h             # Pin mappings, speeds, PID parameters, timings
│   ├── PIDController.h      # Anti-windup clamped PID steering engine
│   ├── SensorArray.h        # 8-channel calibration, normalization & junction detection
│   ├── MotionControl.h      # TB6612FNG motor driver & closed-loop turn routines
│   └── MazeSolver.h         # Turn recording & recursive path simplification engine
├── src/
│   ├── PIDController.cpp
│   ├── SensorArray.cpp
│   ├── MotionControl.cpp
│   ├── MazeSolver.cpp
│   └── main.cpp             # Finite State Machine (Calibration -> Dry Run -> Actual Run)
├── test/
│   └── maze_sim.py          # Python test harness verifying turn reduction rules
├── ARCHITECTURE.md          # Technical specifications and competition rulebook analysis
└── README.md
```

---

## 🛠️ Usage Instructions

### 1. Build and Flash
* Open this project in **VS Code with PlatformIO** or in the **Arduino IDE** (select board: `ESP32 Dev Module`).
* Connect the ESP32 via USB and flash the firmware.

### 2. Sensor Calibration (1-minute window)
* Place the robot with the IR sensor array resting across a white line on the black arena surface.
* Press the **Calibration Button (GPIO 15)**.
* The robot will execute a 3-second auto-sweep to read min/max reflectance values across all 8 channels.

### 3. Dry Run (Exploration)
* Place the bot at the **START** position.
* Set the **Mode Switch (GPIO 13)** to `LOW` (Dry Run).
* The bot traverses using Left-Hand Rule, maps all turns, stops at the $400\text{ mm}$ End Zone, and glows the **Red LED**.

### 4. Actual Run (Shortest Path Race)
* Return the bot to the **START** position within your 3 restarts limit.
* Toggle the **Mode Switch (GPIO 13)** to `HIGH` (Actual Run).
* The robot executes the learned, dead-end-free shortest path at high speed to the finish box.

### 5. Running the Simulation Suite
Verify the turn-reduction engine locally using Python:
```bash
python test/maze_sim.py
```

Run the end-to-end command-line telemetry simulation:
```bash
python test/demo_cli.py
```

### 6. Interactive 2D Virtual Arena Simulator
The repository includes a browser-based real-time 2D visual simulator (`simulator.html`) that simulates:
- The full 230 cm x 230 cm Techfest 2026-27 arena.
- 8-channel IR sensor bar raycasting and noise.
- PID line following and closed-loop junction turns.
- Real-time path recording and LSRB turn reduction.
- Checkpoints, live telemetry, and official scoring calculation.

To run: Open `simulator.html` in any web browser (Chrome, Edge, Firefox).
Expected output:
```
--------------------------------------------------
Running Maze Turn Simplification Test Suite
--------------------------------------------------
Raw: LBL          -> Optimized: S      | Expected: S      [PASS]
Raw: LBS          -> Optimized: R      | Expected: R      [PASS]
Raw: LBR          -> Optimized: B      | Expected: B      [PASS]
Raw: RBL          -> Optimized: B      | Expected: B      [PASS]
Raw: SBL          -> Optimized: R      | Expected: R      [PASS]
Raw: SBS          -> Optimized: B      | Expected: B      [PASS]
Raw: LBLBS        -> Optimized: B      | Expected: B      [PASS]
Raw: LBLBLL       -> Optimized: RL     | Expected: RL     [PASS]
Raw: LBLLBR       -> Optimized: SB     | Expected: SB     [PASS]
--------------------------------------------------
>> ALL TEST CASES PASSED SUCCESSFULLY! <<
--------------------------------------------------
```
