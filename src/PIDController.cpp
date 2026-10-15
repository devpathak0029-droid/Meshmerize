#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float setpoint)
    : _kp(kp), _ki(ki), _kd(kd), _setpoint(setpoint),
      _lastError(0.0f), _integral(0.0f), _integralLimit(500.0f) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

void PIDController::reset() {
    _lastError = 0.0f;
    _integral = 0.0f;
}

float PIDController::compute(float currentPosition, float dtSeconds) {
    float error = _setpoint - currentPosition;

    // Proportional term
    float pTerm = _kp * error;

    // Integral term with anti-windup clamping
    _integral += error * dtSeconds;
    if (_integral > _integralLimit) _integral = _integralLimit;
    else if (_integral < -_integralLimit) _integral = -_integralLimit;
    float iTerm = _ki * _integral;

    // Derivative term (band-limited difference)
    float dTerm = 0.0f;
    if (dtSeconds > 0.0001f) {
        dTerm = _kd * ((error - _lastError) / dtSeconds);
    }
    _lastError = error;

    return pTerm + iTerm + dTerm;
}
