#include "SensorArray.h"

SensorArray::SensorArray() : _lastPosition(SENSOR_SETPOINT) {
    resetCalibration();
}

void SensorArray::begin() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }
}

void SensorArray::resetCalibration() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        _calibMin[i] = 4095; // Initial high for ADC
        _calibMax[i] = 0;    // Initial low for ADC
        _lastValues[i] = 0;
    }
}

void SensorArray::calibrateMinMax() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t val = analogRead(SENSOR_PINS[i]);
        if (val < _calibMin[i]) _calibMin[i] = val;
        if (val > _calibMax[i]) _calibMax[i] = val;
    }
}

void SensorArray::readSensors(uint16_t* outValues) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t raw = analogRead(SENSOR_PINS[i]);
        
        // Safety guard against divide by zero if uncalibrated
        uint16_t span = (_calibMax[i] > _calibMin[i] + 50) ? (_calibMax[i] - _calibMin[i]) : 1;
        int32_t norm = ((int32_t)(raw - _calibMin[i]) * 1000) / span;

        if (norm < 0) norm = 0;
        if (norm > 1000) norm = 1000;

        _lastValues[i] = (uint16_t)norm;
        if (outValues != nullptr) {
            outValues[i] = _lastValues[i];
        }
    }
}

bool SensorArray::readLinePosition(float& position) {
    readSensors(nullptr);

    uint32_t weightedSum = 0;
    uint32_t sum = 0;
    bool onLine = false;

    // Line is white (higher normalized value above 200)
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t val = _lastValues[i];
        if (val > 200) {
            onLine = true;
        }
        // Exclude noise
        if (val > 50) {
            weightedSum += (uint32_t)val * (i * 1000);
            sum += val;
        }
    }

    if (!onLine || sum == 0) {
        // Line lost: keep last known direction (0 or 7000) to help bot recover
        if (_lastPosition < SENSOR_SETPOINT) {
            position = 0.0f;
        } else {
            position = 7000.0f;
        }
        return false;
    }

    position = (float)weightedSum / (float)sum;
    _lastPosition = position;
    return true;
}

bool SensorArray::isAllWhite() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (_lastValues[i] < 500) return false;
    }
    return true;
}

bool SensorArray::isAllBlack() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (_lastValues[i] > 200) return false;
    }
    return true;
}

bool SensorArray::isLeftDetected() {
    // Outer left sensors (0 and 1)
    return (_lastValues[0] > 500 || _lastValues[1] > 500);
}

bool SensorArray::isRightDetected() {
    // Outer right sensors (6 and 7)
    return (_lastValues[6] > 500 || _lastValues[7] > 500);
}

bool SensorArray::isCenterDetected() {
    // Middle sensors (3 and 4)
    return (_lastValues[3] > 400 || _lastValues[4] > 400);
}

JunctionType SensorArray::detectJunctionImmediate() {
    readSensors(nullptr);

    if (isAllWhite()) {
        return JUNCTION_END_ZONE; // Potential end zone or cross, needs verification
    }

    if (isAllBlack()) {
        return JUNCTION_DEAD_END;
    }

    bool left = isLeftDetected();
    bool right = isRightDetected();
    bool center = isCenterDetected();

    if (left && right && center) {
        return JUNCTION_CROSS_4WAY;
    } else if (left && right && !center) {
        return JUNCTION_T_SPLIT;
    } else if (left && center) {
        return JUNCTION_T_LEFT;
    } else if (right && center) {
        return JUNCTION_T_RIGHT;
    } else if (left && !center) {
        return JUNCTION_LEFT_ONLY;
    } else if (right && !center) {
        return JUNCTION_RIGHT_ONLY;
    }

    return JUNCTION_LINE;
}
