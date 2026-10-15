#include "SensorArray.h"

SensorArray::SensorArray() : _lastPosition(SENSOR_SETPOINT), _isCalibrated(false) {
    resetCalibration();
}

void SensorArray::begin() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }
}

void SensorArray::resetCalibration() {
    _isCalibrated = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
        _calibMin[i] = 4095;
        _calibMax[i] = 0;
        _lastValues[i] = 0;
    }
}

void SensorArray::calibrateMinMax() {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t val = analogRead(SENSOR_PINS[i]);
        if (val < _calibMin[i]) _calibMin[i] = val;
        if (val > _calibMax[i]) _calibMax[i] = val;
    }
    // Verify at least 150 counts difference for meaningful calibration
    uint16_t span = (_calibMax[3] > _calibMin[3]) ? (_calibMax[3] - _calibMin[3]) : 0;
    if (span > 150) {
        _isCalibrated = true;
    }
}

void SensorArray::readSensors(uint16_t* outValues) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t raw = analogRead(SENSOR_PINS[i]);
        
        uint16_t minVal = _calibMin[i];
        uint16_t maxVal = _calibMax[i];

        // Safe fallback if uncalibrated
        if (maxVal <= minVal + 50) {
            minVal = 200;
            maxVal = 3800;
        }

        int32_t norm;
        if (SENSOR_WHITE_IS_HIGH) {
            // Raw higher on white
            norm = ((int32_t)(raw - minVal) * 1000) / (maxVal - minVal);
        } else {
            // Raw lower on white (e.g. QTR-8A phototransistors)
            norm = ((int32_t)(maxVal - raw) * 1000) / (maxVal - minVal);
        }

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

    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t val = _lastValues[i];
        if (val > SENSOR_WHITE_THRESHOLD) {
            onLine = true;
        }
        if (val > SENSOR_NOISE_THRESHOLD) {
            weightedSum += (uint32_t)val * (i * 1000);
            sum += val;
        }
    }

    if (!onLine || sum == 0) {
        // Line lost: retain last known lateral direction to steer back
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

bool SensorArray::isAllWhite() const {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (_lastValues[i] < SENSOR_WHITE_THRESHOLD) return false;
    }
    return true;
}

bool SensorArray::isAllBlack() const {
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (_lastValues[i] > SENSOR_WHITE_THRESHOLD) return false;
    }
    return true;
}

bool SensorArray::isLeftDetected() const {
    // Detects line presence on left wing (sensors 0, 1, or 2)
    return (_lastValues[0] > SENSOR_WHITE_THRESHOLD || 
            _lastValues[1] > SENSOR_WHITE_THRESHOLD || 
            _lastValues[2] > SENSOR_WHITE_THRESHOLD);
}

bool SensorArray::isRightDetected() const {
    // Detects line presence on right wing (sensors 5, 6, or 7)
    return (_lastValues[5] > SENSOR_WHITE_THRESHOLD || 
            _lastValues[6] > SENSOR_WHITE_THRESHOLD || 
            _lastValues[7] > SENSOR_WHITE_THRESHOLD);
}

bool SensorArray::isCenterDetected() const {
    // Detects line directly under center (sensors 3 or 4)
    return (_lastValues[3] > SENSOR_WHITE_THRESHOLD || _lastValues[4] > SENSOR_WHITE_THRESHOLD);
}

void SensorArray::printDiagnostics() const {
    Serial.print("Sensors: [");
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(_lastValues[i]);
        if (i < NUM_SENSORS - 1) Serial.print(", ");
    }
    Serial.print("] Pos: ");
    Serial.println(_lastPosition);
}
