#include <Arduino.h>
#include "Config.h"
#include "PIDController.h"
#include "SensorArray.h"
#include "MotionControl.h"
#include "MazeSolver.h"

// State Machine States
enum BotState {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_DRY_RUN,
    STATE_ACTUAL_RUN,
    STATE_FINISHED
};

// Global Objects
SensorArray    sensorArray;
PIDController  pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, SENSOR_SETPOINT);
MotionControl  motion(&sensorArray);
MazeSolver     solver;

BotState currentState = STATE_IDLE;
uint32_t lastLoopTime = 0;
uint8_t actualRunTurnIndex = 0;

void runCalibrationRoutine() {
    Serial.println("Starting Sensor Calibration...");
    currentState = STATE_CALIBRATING;
    sensorArray.resetCalibration();

    // Pivot bot back and forth across line to expose all sensors to min/max
    uint32_t calibStart = millis();
    while (millis() - calibStart < 3000) {
        motion.setMotorSpeeds(-110, 110);
        for (int i = 0; i < 20; i++) {
            sensorArray.calibrateMinMax();
            delay(5);
        }
        motion.setMotorSpeeds(110, -110);
        for (int i = 0; i < 20; i++) {
            sensorArray.calibrateMinMax();
            delay(5);
        }
    }
    motion.stop();
    Serial.println("Calibration Complete!");
    currentState = STATE_IDLE;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("======================================");
    Serial.println(" Techfest 2026-27: Meshmerize Bot Init");
    Serial.println("======================================");

    pinMode(PIN_RED_LED, OUTPUT);
    pinMode(PIN_CALIB_BUTTON, INPUT_PULLUP);
    pinMode(PIN_MODE_SWITCH, INPUT_PULLUP);
    digitalWrite(PIN_RED_LED, LOW);

    sensorArray.begin();
    motion.begin();

    Serial.println("Ready. Press calibration button to calibrate, or mode switch to run.");
}

// ----------------------------------------------------
// DRY RUN EXPLORATION LOGIC (Left-Hand Rule + Mapping)
// ----------------------------------------------------
void handleDryRun() {
    float pos = SENSOR_SETPOINT;
    bool onLine = sensorArray.readLinePosition(pos);

    // 1. Check for potential End Zone (solid 400x400 mm box) or Cross
    if (sensorArray.isAllWhite()) {
        // Verification: Inching forward to confirm solid 400mm box vs 30mm stripe
        motion.inchForward(INCH_TIME_MS);
        sensorArray.readSensors(nullptr);

        if (sensorArray.isAllWhite()) {
            // CONFIRMED END ZONE!
            motion.stop();
            digitalWrite(PIN_RED_LED, HIGH); // Mandatory red LED glow rule
            solver.recordTurn('E');
            solver.optimizeAll();
            Serial.println(">>> END ZONE REACHED IN DRY RUN! <<<");
            solver.printPath();
            currentState = STATE_FINISHED;
            return;
        } else {
            // It was a 4-way cross intersection!
            // Left Hand Rule: Take Left
            motion.turnLeft90();
            solver.recordTurn('L');
            return;
        }
    }

    // 2. Dead End Check (All black)
    if (sensorArray.isAllBlack()) {
        motion.inchForward(60); // Verify dead end
        sensorArray.readSensors(nullptr);
        if (sensorArray.isAllBlack()) {
            motion.turnAround180();
            solver.recordTurn('B');
            return;
        }
    }

    // 3. Junction Detection
    bool leftDetected = sensorArray.isLeftDetected();
    bool rightDetected = sensorArray.isRightDetected();

    if (leftDetected || rightDetected) {
        // Inch forward slightly to align bot center with junction intersection
        motion.inchForward(INCH_TIME_MS);
        sensorArray.readSensors(nullptr);

        // Check if there is still a straight path ahead
        bool straightAvailable = sensorArray.isCenterDetected();

        if (leftDetected) {
            // Left-Hand Rule: ALWAYS take Left if available
            motion.turnLeft90();
            solver.recordTurn('L');
        } else if (straightAvailable) {
            // Straight is available, no left turn
            solver.recordTurn('S');
            // Continue driving straight
        } else if (rightDetected) {
            // Only Right is available
            motion.turnRight90();
            solver.recordTurn('R');
        }
        return;
    }

    // 4. Normal PID Line Following
    uint32_t now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;

    float correction = pid.compute(pos, dt);
    int16_t leftPwm  = (int16_t)(DRY_RUN_BASE_SPEED + correction);
    int16_t rightPwm = (int16_t)(DRY_RUN_BASE_SPEED - correction);

    motion.setMotorSpeeds(leftPwm, rightPwm);
}

// ----------------------------------------------------
// ACTUAL RUN (OPTIMIZED HIGH SPEED TRAVERSAL)
// ----------------------------------------------------
void handleActualRun() {
    float pos = SENSOR_SETPOINT;
    sensorArray.readLinePosition(pos);

    // 1. Check for End Zone
    if (sensorArray.isAllWhite()) {
        motion.inchForward(INCH_TIME_MS);
        sensorArray.readSensors(nullptr);
        if (sensorArray.isAllWhite()) {
            motion.stop();
            digitalWrite(PIN_RED_LED, HIGH);
            Serial.println(">>> ACTUAL RUN COMPLETED THROUGH SHORTEST PATH! <<<");
            currentState = STATE_FINISHED;
            return;
        }
    }

    // 2. Junction Reached: Execute planned turn from optimized array!
    bool leftDetected = sensorArray.isLeftDetected();
    bool rightDetected = sensorArray.isRightDetected();

    if (leftDetected || rightDetected) {
        motion.inchForward(INCH_TIME_MS);

        if (actualRunTurnIndex < solver.getPathLength()) {
            char nextTurn = solver.getTurnAt(actualRunTurnIndex++);
            Serial.print("Executing Planned Turn: ");
            Serial.println(nextTurn);

            if (nextTurn == 'L') {
                motion.turnLeft90();
            } else if (nextTurn == 'R') {
                motion.turnRight90();
            } else if (nextTurn == 'S') {
                // Keep moving straight across junction
            } else if (nextTurn == 'E') {
                motion.stop();
                digitalWrite(PIN_RED_LED, HIGH);
                currentState = STATE_FINISHED;
                return;
            }
        }
        return;
    }

    // 3. High-Speed PID Tracking between junctions
    uint32_t now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;
    lastLoopTime = now;

    float correction = pid.compute(pos, dt);
    int16_t leftPwm  = (int16_t)(ACTUAL_RUN_BASE_SPEED + correction);
    int16_t rightPwm = (int16_t)(ACTUAL_RUN_BASE_SPEED - correction);

    motion.setMotorSpeeds(leftPwm, rightPwm);
}

void loop() {
    // Check calibration button
    if (digitalRead(PIN_CALIB_BUTTON) == LOW && currentState == STATE_IDLE) {
        delay(200); // Debounce
        runCalibrationRoutine();
    }

    // Mode Switch check to start runs
    if (currentState == STATE_IDLE) {
        if (digitalRead(PIN_MODE_SWITCH) == LOW) {
            Serial.println("Mode: Starting DRY RUN Exploration...");
            delay(1000); // 1 sec countdown
            currentState = STATE_DRY_RUN;
            lastLoopTime = micros();
        }
    }

    // Finished Dry run transition to Actual Run
    if (currentState == STATE_FINISHED && digitalRead(PIN_MODE_SWITCH) == HIGH) {
        // Toggle switch switched to Actual Run
        Serial.println("Mode: Armed for ACTUAL RUN (Shortest Path)...");
        digitalWrite(PIN_RED_LED, LOW);
        actualRunTurnIndex = 0;
        delay(2000);
        currentState = STATE_ACTUAL_RUN;
        lastLoopTime = micros();
    }

    // State execution
    if (currentState == STATE_DRY_RUN) {
        handleDryRun();
    } else if (currentState == STATE_ACTUAL_RUN) {
        handleActualRun();
    }
}
