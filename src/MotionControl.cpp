#include "MotionControl.h"

// Compatibility layer for ESP32 Arduino Core 2.x vs Core 3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    #define USE_LEDC_V3 1
#else
    #define USE_LEDC_V3 0
#endif

MotionControl::MotionControl(SensorArray* sensors) : _sensors(sensors) {}

void MotionControl::begin() {
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_AIN2, OUTPUT);
    pinMode(PIN_MOTOR_BIN1, OUTPUT);
    pinMode(PIN_MOTOR_BIN2, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Initial safe state: motors standby active
    digitalWrite(PIN_MOTOR_STBY, HIGH);

#if USE_LEDC_V3
    ledcAttach(PIN_MOTOR_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_MOTOR_PWMB, PWM_FREQ, PWM_RESOLUTION);
#else
    ledcSetup(PWM_CHAN_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHAN_B, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_MOTOR_PWMA, PWM_CHAN_A);
    ledcAttachPin(PIN_MOTOR_PWMB, PWM_CHAN_B);
#endif

    brake();
}

void MotionControl::stop() {
    // High impedance coast
    digitalWrite(PIN_MOTOR_AIN1, LOW);
    digitalWrite(PIN_MOTOR_AIN2, LOW);
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_BIN2, LOW);
#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_PWMA, 0);
    ledcWrite(PIN_MOTOR_PWMB, 0);
#else
    ledcWrite(PWM_CHAN_A, 0);
    ledcWrite(PWM_CHAN_B, 0);
#endif
}

void MotionControl::brake() {
    // TB6612FNG Active Brake: IN1=HIGH, IN2=HIGH, PWM=255
    digitalWrite(PIN_MOTOR_AIN1, HIGH);
    digitalWrite(PIN_MOTOR_AIN2, HIGH);
    digitalWrite(PIN_MOTOR_BIN1, HIGH);
    digitalWrite(PIN_MOTOR_BIN2, HIGH);
#if USE_LEDC_V3
    ledcWrite(PIN_MOTOR_PWMA, 255);
    ledcWrite(PIN_MOTOR_PWMB, 255);
#else
    ledcWrite(PWM_CHAN_A, 255);
    ledcWrite(PWM_CHAN_B, 255);
#endif
}

void MotionControl::driveLeft(int16_t speed) {
    if (INVERT_LEFT_MOTOR) speed = -speed;
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, LOW);
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMA, speed);
#else
        ledcWrite(PWM_CHAN_A, speed);
#endif
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_AIN1, LOW);
        digitalWrite(PIN_MOTOR_AIN2, HIGH);
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMA, -speed);
#else
        ledcWrite(PWM_CHAN_A, -speed);
#endif
    } else {
        digitalWrite(PIN_MOTOR_AIN1, HIGH);
        digitalWrite(PIN_MOTOR_AIN2, HIGH); // Brake
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMA, 255);
#else
        ledcWrite(PWM_CHAN_A, 255);
#endif
    }
}

void MotionControl::driveRight(int16_t speed) {
    if (INVERT_RIGHT_MOTOR) speed = -speed;
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        digitalWrite(PIN_MOTOR_BIN1, HIGH);
        digitalWrite(PIN_MOTOR_BIN2, LOW);
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMB, speed);
#else
        ledcWrite(PWM_CHAN_B, speed);
#endif
    } else if (speed < 0) {
        digitalWrite(PIN_MOTOR_BIN1, LOW);
        digitalWrite(PIN_MOTOR_BIN2, HIGH);
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMB, -speed);
#else
        ledcWrite(PWM_CHAN_B, -speed);
#endif
    } else {
        digitalWrite(PIN_MOTOR_BIN1, HIGH);
        digitalWrite(PIN_MOTOR_BIN2, HIGH); // Brake
#if USE_LEDC_V3
        ledcWrite(PIN_MOTOR_PWMB, 255);
#else
        ledcWrite(PWM_CHAN_B, 255);
#endif
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
    brake();
    delay(40); // brief settle
}

bool MotionControl::turnLeft90() {
    uint32_t start = millis();
    // Step 1: Initial pivot kick to clear current line
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(130);

    // Step 2: Spin until center sensor locks onto new perpendicular line
    while ((millis() - start) < TURN_TIMEOUT_MS) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            brake();
            delay(30);
            return true;
        }
        delay(4);
    }
    brake();
    return false;
}

bool MotionControl::turnRight90() {
    uint32_t start = millis();
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(130);

    while ((millis() - start) < TURN_TIMEOUT_MS) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            brake();
            delay(30);
            return true;
        }
        delay(4);
    }
    brake();
    return false;
}

bool MotionControl::turnAround180() {
    uint32_t start = millis();
    setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(260); // kick to clear dead end

    while ((millis() - start) < (TURN_TIMEOUT_MS * 1.5)) {
        _sensors->readSensors(nullptr);
        if (_sensors->isCenterDetected()) {
            brake();
            delay(30);
            return true;
        }
        delay(4);
    }
    brake();
    return false;
}
