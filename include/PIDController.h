#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float setpoint);

    void setTunings(float kp, float ki, float kd);
    void setSetpoint(float setpoint);
    void reset();

    // Computes motor differential speed correction
    float compute(float currentPosition, float dtSeconds);

private:
    float _kp;
    float _ki;
    float _kd;
    float _setpoint;

    float _lastError;
    float _integral;
    float _integralLimit;
};

#endif // PIDCONTROLLER_H
