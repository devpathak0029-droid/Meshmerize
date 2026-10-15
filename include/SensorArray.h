#ifndef SENSORARRAY_H
#define SENSORARRAY_H

#include <Arduino.h>
#include "Config.h"

enum JunctionType {
    JUNCTION_NONE,
    JUNCTION_LINE,            // Normal straight line segment
    JUNCTION_LEFT_ONLY,       // 90 deg Left turn (no straight path)
    JUNCTION_RIGHT_ONLY,      // 90 deg Right turn (no straight path)
    JUNCTION_T_LEFT,          // T-intersection with left branch + straight
    JUNCTION_T_RIGHT,         // T-intersection with right branch + straight
    JUNCTION_T_SPLIT,         // T-intersection: Left and Right branches, no straight
    JUNCTION_CROSS_4WAY,      // 4-way cross: Left, Straight, Right
    JUNCTION_DEAD_END,        // No line detected under any sensor
    JUNCTION_END_ZONE         // 400mm x 400mm solid white box
};

class SensorArray {
public:
    SensorArray();

    void begin();
    void calibrateMinMax(); // Call repeatedly during manual or auto-spin calibration
    void resetCalibration();

    // Read normalized sensor values (0 = pure black surface, 1000 = pure white line)
    void readSensors(uint16_t* outValues);

    // Calculate line center position (0 to 7000, 3500 is centered)
    // Returns true if line is present, false if line lost
    bool readLinePosition(float& position);

    // Raw junction detection based on sensor bitmasks
    JunctionType detectJunctionImmediate();

    // Verification helpers
    bool isAllWhite();
    bool isAllBlack();
    bool isLeftDetected();
    bool isRightDetected();
    bool isCenterDetected();

private:
    uint16_t _calibMin[NUM_SENSORS];
    uint16_t _calibMax[NUM_SENSORS];
    uint16_t _lastValues[NUM_SENSORS];
    float _lastPosition;
};

#endif // SENSORARRAY_H
