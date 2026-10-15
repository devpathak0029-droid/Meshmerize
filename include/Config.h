#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. PIN DEFINITIONS (ESP32 DevKit V1)
// ==========================================
// Note on Strapping Pins:
// GPIO 0, 2, 5, 12, 15 are strapping pins on classic ESP32.
// Take care when wiring pull-ups/pull-downs.

// TB6612FNG Dual H-Bridge Motor Driver
#define PIN_MOTOR_AIN1     18
#define PIN_MOTOR_AIN2     19
#define PIN_MOTOR_PWMA     4
#define PIN_MOTOR_BIN1     21
#define PIN_MOTOR_BIN2     22
#define PIN_MOTOR_PWMB     23
#define PIN_MOTOR_STBY     5

// Motor Direction Inversion Configuration
#define INVERT_LEFT_MOTOR  false
#define INVERT_RIGHT_MOTOR false

// LEDC PWM Configurations (ESP32 hardware PWM)
#define PWM_FREQ           20000 // 20 kHz ultrasonic (silent motors)
#define PWM_RESOLUTION     8     // 8-bit resolution (0-255)
#define PWM_CHAN_A         0     // Used for ESP32 Arduino Core 2.x
#define PWM_CHAN_B         1     // Used for ESP32 Arduino Core 2.x

// 8-Channel Analog IR Sensor Array Pins
// ADC1 channels on Classic ESP32: GPIO 32, 33, 34, 35, 36, 39
// ADC2 channels on Classic ESP32: GPIO 0, 2, 4, 12, 13, 14, 15, 25, 26, 27
// NOTICE: GPIO 25 and 26 are ADC2. ADC2 cannot be used concurrently with Wi-Fi/BT.
// Since Meshmerize rules explicitly prohibit wireless control, ADC2 is fully operational,
// but Wi-Fi and Bluetooth MUST remain disabled in firmware.
#define NUM_SENSORS        8
const uint8_t SENSOR_PINS[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// Sensor Polarity & Reflectance Characteristics
// true  = Higher raw analog value on WHITE line (e.g., active high or pull-up sensors)
// false = Lower raw analog value on WHITE line (e.g., QTR-8A phototransistor pull-down)
#define SENSOR_WHITE_IS_HIGH true

// Thresholds for normalized readings (0 to 1000)
#define SENSOR_WHITE_THRESHOLD 450
#define SENSOR_NOISE_THRESHOLD 80

// User Interface & Rule Mandates
#define PIN_RED_LED        2     // Mandatory: Must glow when End Zone is reached
#define PIN_CALIB_BUTTON   15    // Push button to trigger auto-calibration sweep
#define PIN_MODE_SWITCH    13    // Toggle switch: LOW = Dry Run, HIGH = Actual Run

// ==========================================
// 2. SPEED & MOTION PARAMETERS
// ==========================================
#define DRY_RUN_BASE_SPEED     140  // Safe exploration speed (0-255)
#define DRY_RUN_MAX_SPEED      190
#define ACTUAL_RUN_BASE_SPEED  210  // High-speed optimal run
#define ACTUAL_RUN_MAX_SPEED   255
#define TURN_SPEED             135  // Pivot turn speed

// Inching parameters (Forward verification of 400x400 mm end zone vs 30 mm line)
#define INCH_SPEED             115
#define INCH_TIME_MS           120  // Time to advance past a 30mm transverse stripe
#define CLEAR_JUNCTION_TIME_MS 100  // Advance to clear intersection after turning

// Turn safety timeout (prevents infinite spinning if line is lost)
#define TURN_TIMEOUT_MS        1400

// ==========================================
// 3. PID CONTROLLER TUNING CONSTANTS
// ==========================================
// Setpoint is centered between sensor 0 and sensor 7:
// Range: 0 to 7000, Setpoint = 3500
#define SENSOR_SETPOINT        3500.0f

#define DEFAULT_KP             0.045f
#define DEFAULT_KI             0.0001f
#define DEFAULT_KD             0.35f
#define PID_MAX_CORRECTION     180.0f // Clamping limit for differential steering

// Maximum time delta permitted per PID step (prevents dt explosion after blocking turns)
#define PID_MAX_DT_SEC         0.05f

// ==========================================
// 4. MAZE SOLVER PARAMETERS
// ==========================================
#define MAX_PATH_LENGTH        200

#endif // CONFIG_H
