#include "MotionControl.h"

MotionControl::MotionControl(SensorArray* sensors) : _sensors(sensors) {}

void MotionControl::begin() {
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_BIN1, OUTPUT);
    pinMode(PIN_MOTOR_BIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Enable motor driver standby
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Configure ESP32 LEDC PWM timer and pins
    ledcSetup(PWM_CHAN_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHAN_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_PWMA, PWM_CHAN_A);
    ledcAttachPin(PIN_MOTOR_PWMB, PWM_CHAN_B);

    stop();
}

void MotionControl::stop() {
    setMotorSpeeds(0, 0);
}

void MotionControl::driveLeft(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        ledcWrite(PWM_CHAN_A, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, HIGH);
        ledcWrite(PWM_CHAN_A, -speed);
    } else {
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
        ledcWrite(PWM_CHAN_A, 0);
    }
}

void MotionControl::driveRight(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(PIN_MOTOR_BIN1, HIGH);
        digitalWrite(PIN_MOTOR_BIN2, LOW);
        ledcWrite(PWM_CHAN_B, speed);
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_BIN1, LOW);
        digitalWrite(PIN_MOTOR_BIN2, HIGH);
        ledcWrite(PWM_CHAN_B, -speed);
    } else {
        digitalWrite(PIN_MOTOR_BIN1, LOW);
        digitalWrite(PIN_MOTOR_BIN2, LOW);
        ledcWrite(PWM_CHAN_B, 0);
    }
}

void MotionControl::setMotorSpeeds(int16_t leftPwm, int16_t rightPwm) {
    driveLeft(leftPwm);
    driveRight(rightPwm);
}

void MotionControl::forward(int16_t speed) {
    setMotorSpeeds(speed, speed);
}

void MotionControl::backward(int16_t speed) {
    setMotorSpeeds(-speed, -speed);
}

void MotionControl::inchForward(uint16_t durationMs) {
    forward(INCH_SPEED);
    delay(durationMs);
    stop();
}

// Closed-loop Left 90 Turn:
// 1. Pivot until line is cleared (debounce)
// 2. Pivot until center sensor re-acquires the white line
bool MotionControl::turnLeft90() {
    uint32_t start = millis();
    
    // Step 1: Spin off the current line
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(150); // Small initial blind kick to clear original line

    // Step 2: Spin until center sensor detects new perpendicular line
    while ((millis() - start) < TURN_TIMEOUT_MS) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            stop();
            return true;
        }
        delay(5);
    }
    stop();
    return false; // Timeout safety
}

// Closed-loop Right 90 Turn
bool MotionControl::turnRight90() {
    uint32_t start = millis();

    // Step 1: Spin off the current line
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(150);

    // Step 2: Spin until center sensor detects new perpendicular line
    while ((millis() - start) < TURN_TIMEOUT_MS) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            stop();
            return true;
        }
        delay(5);
    }
    stop();
    return false;
}

// Closed-loop 180 Turn (U-Turn at dead end)
bool MotionControl::turnAround180() {
    uint32_t start = millis();

    // Pivot around until line re-acquired
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(300); // Larger kick to clear reverse dead end

    while ((millis() - start) < (TURN_TIMEOUT_MS * 1.5)) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            stop();
            return true;
        }
        delay(5);
    }
    stop();
    return false;
}
