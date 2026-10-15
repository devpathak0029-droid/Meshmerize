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
    void setMotorSpeeds(int16_t leftPwm, int16_t rightPwm);

    // Drive forward with speed and optional duration
    void forward(int16_t speed);
    void backward(int16_t speed);

    // Safe Inching (Forward verification: helps accurately distinguish between 30mm stripe vs 400mm end zone)
    void inchForward(uint16_t durationMs = INCH_TIME_MS);

    // Closed-loop sensor assisted 90 degree and 180 degree turns
    // Uses center sensor line acquisition instead of blind delay!
    bool turnLeft90();
    bool turnRight90();
    bool turnAround180();

private:
    SensorArray* _sensors;
    void driveLeft(int16_t speed);
    void driveRight(int16_t speed);
};

#endif // MOTIONCONTROL_H
