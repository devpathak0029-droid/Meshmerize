#include <Arduino.h>
#include "Config.h"
#include "PIDController.h"
#include "SensorArray.h"
#include "MotionControl.h"
#include "MazeSolver.h"

// Explicit Operating States
enum BotState {
    STATE_BOOT,
    STATE_CALIBRATING,
    STATE_READY_FOR_DRY_RUN,
    STATE_DRY_RUN_EXPLORING,
    STATE_DRY_RUN_COMPLETED,
    STATE_READY_FOR_ACTUAL_RUN,
    STATE_ACTUAL_RUN_REPLAY,
    STATE_COMPLETED,
    STATE_FAULT
};

// Global Objects
SensorArray    sensorArray;
PIDController  pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, SENSOR_SETPOINT, PID_MAX_CORRECTION);
MotionControl  motion(&sensorArray);
MazeSolver     solver;

BotState currentState = STATE_BOOT;
uint32_t lastLoopTime = 0;
uint8_t  actualRunTurnIndex = 0;
bool     junctionLocked = false; // Prevents multi-logging at the same physical junction

void runCalibrationSweep() {
    Serial.println("Starting Sensor Calibration Sweep...");
    currentState = STATE_CALIBRATING;
    sensorArray.resetCalibration();

    // Pivot back and forth across line to capture min/max on all 8 channels
    uint32_t start = millis();
    while (millis() - start < 3000) {
        motion.setMotorSpeeds(-110, 110);
        for (int i = 0; i < 15; i++) {
            sensorArray.calibrateMinMax();
            delay(5);
        }
        motion.setMotorSpeeds(110, -110);
        for (int i = 0; i < 15; i++) {
            sensorArray.calibrateMinMax();
            delay(5);
        }
    }
    motion.brake();
    sensorArray.printDiagnostics();
    Serial.println("Calibration complete. Bot is ready.");
    currentState = STATE_READY_FOR_DRY_RUN;
}

void setup() {
    Serial.begin(115200);
    delay(400);

    Serial.println("\n============================================");
    Serial.println("  TECHFEST 2026-27: MESHMERIZE ROBOT FIRMWARE");
    Serial.println("============================================");

    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_CALIB_BUTTON, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);
    digitalWrite(PIN_RED_LED, LOW);

    sensorArray.begin();
    motion.begin();

    Serial.println("Boot complete. Place on line and press Calibrate (GPIO 15).");
    currentState = STATE_READY_FOR_DRY_RUN;
}

// ----------------------------------------------------
// DRY RUN EXPLORATION
// ----------------------------------------------------
void handleDryRun() {
    float pos = SENSOR_SETPOINT;
    bool onLine = sensorArray.readLinePosition(pos);

    // 1. Check for End Zone (Solid 400x400 mm white box)
    if (sensorArray.isAllWhite()) {
        motion.inchForward(INCH_TIME_MS);
        sensorArray.readSensors(nullptr);

        if (sensorArray.isAllWhite()) {
            // CONFIRMED END ZONE!
            motion.brake();
            digitalWrite(PIN_RED_LED, HIGH); // Mandatory Red LED rule
            solver.recordTurn('E');
            solver.optimizeAll();
            Serial.println("\n>>> [DRY RUN] END ZONE CONFIRMED! <<<");
            solver.printPath();
            currentState = STATE_DRY_RUN_COMPLETED;
            return;
        } else {
            // It was a 4-way cross intersection!
            // Left-Hand Rule: take Left
            motion.turnLeft90();
            motion.inchForward(CLEAR_JUNCTION_TIME_MS); // clear intersection
            solver.recordTurn('L');
            junctionLocked = true;
            lastLoopTime = micros();
            return;
        }
    }

    // 2. Dead End Check (All black)
    if (sensorArray.isAllBlack()) {
        motion.inchForward(50); // verify dead end
        sensorArray.readSensors(nullptr);
        if (sensorArray.isAllBlack()) {
            motion.turnAround180();
            solver.recordTurn('B');
            junctionLocked = true;
            lastLoopTime = micros();
            return;
        }
    }

    // 3. Junction Detection with anti-thrashing lock
    bool leftDetected = sensorArray.isLeftDetected();
    bool rightDetected = sensorArray.isRightDetected();

    if (leftDetected || rightDetected) {
        if (!junctionLocked) {
            // New junction encountered: advance wheel axis over junction center
            motion.inchForward(INCH_TIME_MS);
            sensorArray.readSensors(nullptr);

            bool straightAvailable = sensorArray.isCenterDetected();

            if (leftDetected) {
                // Left-Hand Rule Priority: Turn Left
                motion.turnLeft90();
                motion.inchForward(CLEAR_JUNCTION_TIME_MS); // Clear intersection line
                solver.recordTurn('L');
            } else if (straightAvailable) {
                // Straight path available
                solver.recordTurn('S');
                motion.forward(DRY_RUN_BASE_SPEED); // Keep moving forward, do NOT remain stopped!
            } else if (rightDetected) {
                // Only Right available
                motion.turnRight90();
                motion.inchForward(CLEAR_JUNCTION_TIME_MS);
                solver.recordTurn('R');
            }

            junctionLocked = true;
            lastLoopTime = micros(); // Re-sync loop time to prevent PID dt explosion
            return;
        }
    } else {
        // Once both wings are clear of transverse lines, unlock junction detector
        junctionLocked = false;
    }

    // 4. Smooth PID Line Following
    uint32_t now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;

    if (dt > PID_MAX_DT_SEC) dt = 0.01f; // Clamp dt if blocked

    float correction = pid.compute(pos, dt);
    int16_t leftPwm  = (int16_t)(DRY_RUN_BASE_SPEED + correction);
    int16_t rightPwm = (int16_t)(DRY_RUN_BASE_SPEED - correction);

    motion.setMotorSpeeds(leftPwm, rightPwm);
}

// ----------------------------------------------------
// ACTUAL RUN (OPTIMIZED REPLAY AT HIGH SPEED)
// ----------------------------------------------------
void handleActualRun() {
    float pos = SENSOR_SETPOINT;
    sensorArray.readLinePosition(pos);

    // 1. Check for End Zone
    if (sensorArray.isAllWhite()) {
        motion.inchForward(INCH_TIME_MS);
        sensorArray.readSensors(nullptr);
        if (sensorArray.isAllWhite()) {
            motion.brake();
            digitalWrite(PIN_RED_LED, HIGH);
            Serial.println("\n>>> [ACTUAL RUN] COMPLETED AT SHORTEST PATH! <<<");
            currentState = STATE_COMPLETED;
            return;
        }
    }

    // 2. Junction Reached: Execute planned turn from optimized array
    bool leftDetected = sensorArray.isLeftDetected();
    bool rightDetected = sensorArray.isRightDetected();

    if (leftDetected || rightDetected) {
        if (!junctionLocked) {
            motion.inchForward(INCH_TIME_MS);

            if (actualRunTurnIndex < solver.getPathLength()) {
                char nextTurn = solver.getTurnAt(actualRunTurnIndex++);
                Serial.print("[ACTUAL RUN] Executing Turn: ");
                Serial.println(nextTurn);

                if (nextTurn == 'L') {
                    motion.turnLeft90();
                    motion.inchForward(CLEAR_JUNCTION_TIME_MS);
                } else if (nextTurn == 'R') {
                    motion.turnRight90();
                    motion.inchForward(CLEAR_JUNCTION_TIME_MS);
                } else if (nextTurn == 'S') {
                    motion.forward(ACTUAL_RUN_BASE_SPEED); // Resume forward immediately!
                } else if (nextTurn == 'E') {
                    motion.brake();
                    digitalWrite(PIN_RED_LED, HIGH);
                    currentState = STATE_COMPLETED;
                    return;
                }
            }
            junctionLocked = true;
            lastLoopTime = micros();
            return;
        }
    } else {
        junctionLocked = false;
    }

    // 3. High-Speed PID Tracking between junctions
    uint32_t now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;

    if (dt > PID_MAX_DT_SEC) dt = 0.01f;

    float correction = pid.compute(pos, dt);
    int16_t leftPwm  = (int16_t)(ACTUAL_RUN_BASE_SPEED + correction);
    int16_t rightPwm = (int16_t)(ACTUAL_RUN_BASE_SPEED - correction);

    motion.setMotorSpeeds(leftPwm, rightPwm);
}

void loop() {
    // Calibration trigger
    if (digitalRead(PIN_CALIB_BUTTON) == LOW) {
        delay(200); // debounce
        if (currentState == STATE_READY_FOR_DRY_RUN || currentState == STATE_BOOT) {
            runCalibrationSweep();
        }
    }

    // State transitions
    if (currentState == STATE_READY_FOR_DRY_RUN) {
        // Start Dry Run when mode switch is LOW and Calibration button is pressed to confirm
        if (digitalRead(PIN_MODE_SWITCH) == LOW) {
            Serial.println("Armed: Press Calib Button to START DRY RUN...");
            if (digitalRead(PIN_CALIB_BUTTON) == LOW) {
                delay(500);
                Serial.println("Starting DRY RUN Exploration now!");
                currentState = STATE_DRY_RUN_EXPLORING;
                lastLoopTime = micros();
                pid.reset();
            }
        }
    }

    // Transition from Dry Run to Actual Run
    if (currentState == STATE_DRY_RUN_COMPLETED) {
        if (digitalRead(PIN_MODE_SWITCH) == HIGH) {
            Serial.println("Armed: Place bot at START and press Button for ACTUAL RUN...");
            digitalWrite(PIN_RED_LED, LOW);
            actualRunTurnIndex = 0;
            currentState = STATE_READY_FOR_ACTUAL_RUN;
        }
    }

    if (currentState == STATE_READY_FOR_ACTUAL_RUN) {
        if (digitalRead(PIN_CALIB_BUTTON) == LOW) {
            delay(500);
            Serial.println("Starting ACTUAL RUN (Shortest Path Replay) now!");
            currentState = STATE_ACTUAL_RUN_REPLAY;
            lastLoopTime = micros();
            pid.reset();
        }
    }

    // Execution
    if (currentState == STATE_DRY_RUN_EXPLORING) {
        handleDryRun();
    } else if (currentState == STATE_ACTUAL_RUN_REPLAY) {
        handleActualRun();
    }
}
