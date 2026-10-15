#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float setpoint, float maxOutput)
    : _kp(kp), _ki(ki), _kd(kd), _setpoint(setpoint), _maxOutput(maxOutput),
      _lastError(0.0f), _integral(0.0f), _integralLimit(500.0f), _firstRun(true) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setSetpoint(float setpoint) {
    _setpoint = setpoint;
}

void PIDController::setOutputLimits(float maxOutput) {
    _maxOutput = (maxOutput > 0.0f) ? maxOutput : 200.0f;
}

void PIDController::reset() {
    _lastError = 0.0f;
    _integral = 0.0f;
    _firstRun = true;
}

float PIDController::compute(float currentPosition, float dtSeconds) {
    float error = _setpoint - currentPosition;

    // Safety guard against unreasonable or zero time intervals
    if (dtSeconds <= 0.0001f || dtSeconds > 0.1f) {
        dtSeconds = 0.01f; // fallback to 10ms nominal cycle
    }

    // Proportional term
    float pTerm = _kp * error;

    // Integral term with anti-windup clamping
    _integral += error * dtSeconds;
    if (_integral > _integralLimit) _integral = _integralLimit;
    else if (_integral < -_integralLimit) _integral = -_integralLimit;
    float iTerm = _ki * _integral;

    // Derivative term with first-run kick suppression
    float dTerm = 0.0f;
    if (_firstRun) {
        _lastError = error;
        _firstRun = false;
    } else {
        dTerm = _kd * ((error - _lastError) / dtSeconds);
        _lastError = error;
    }

    float totalOutput = pTerm + iTerm + dTerm;

    // Bound output to prevent motor saturation overflow
    if (totalOutput > _maxOutput) totalOutput = _maxOutput;
    else if (totalOutput < -_maxOutput) totalOutput = -_maxOutput;

    return totalOutput;
}
