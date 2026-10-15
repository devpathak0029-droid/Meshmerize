#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float setpoint, float maxOutput = 200.0f);

    void setTunings(float kp, float ki, float kd);
    void setSetpoint(float setpoint);
    void setOutputLimits(float maxOutput);
    void reset();

    // Computes motor differential speed correction with dt clamping & kick protection
    float compute(float currentPosition, float dtSeconds);

private:
    float _kp;
    float _ki;
    float _kd;
    float _setpoint;
    float _maxOutput;

    float _lastError;
    float _integral;
    float _integralLimit;
    bool  _firstRun;
};

#endif // PIDCONTROLLER_H
