#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include <Arduino.h>
#include "Config.h"
#include "SensorArray.h"

class MotionControl {
public:
    MotionControl(SensorArray* sensors);

    void begin();
    void stop();
    void brake();
    void setMotorSpeeds(int16_t leftPwm, int16_t rightPwm);

    void forward(int16_t speed);
    void backward(int16_t speed);

    // Inching forward with optional brake
    void inchForward(uint16_t durationMs = INCH_TIME_MS);

    // Closed-loop sensor assisted turns with active braking
    bool turnLeft90();
    bool turnRight90();
    bool turnAround180();

private:
    SensorArray* _sensors;
    void driveLeft(int16_t speed);
    void driveRight(int16_t speed);
};

#endif // MOTIONCONTROL_H
