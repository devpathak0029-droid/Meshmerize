#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. PIN DEFINITIONS (ESP32)
// ==========================================

// TB6612FNG Motor Driver Pins
#define PIN_MOTOR_AIN1     18
#define PIN_MOTOR_AIN2     19
#define PIN_MOTOR_PWMA     4
#define PIN_MOTOR_BIN1     21
#define PIN_MOTOR_BIN2     22
#define PIN_MOTOR_PWMB     23
#define PIN_MOTOR_STBY     5

// LEDC PWM Channels (ESP32 hardware PWM)
#define PWM_FREQ           20000 // 20 kHz ultrasonic (silent motors)
#define PWM_RESOLUTION     8     // 8-bit resolution (0-255)
#define PWM_CHAN_A         0
#define PWM_CHAN_B         1

// 8-Channel IR Sensor Array Pins (ADC1 pins recommended for ESP32 with WiFi)
// White line on black arena
#define NUM_SENSORS        8
const uint8_t SENSOR_PINS[NUM_SENSORS] = {36, 39, 34, 35, 32, 33, 25, 26};

// User Interface & Rule Mandates
#define PIN_RED_LED        2     // Mandatory: Must glow when End Zone is reached
#define PIN_CALIB_BUTTON   15    // Push button to trigger auto-calibration sweep
#define PIN_MODE_SWITCH    13    // Toggle switch / Jumper: LOW = Dry Run, HIGH = Actual Run

// ==========================================
// 2. SPEED & MOTION PARAMETERS
// ==========================================
#define DRY_RUN_BASE_SPEED     140  // Safe exploration speed (0-255)
#define DRY_RUN_MAX_SPEED      190
#define ACTUAL_RUN_BASE_SPEED  200  // High-speed optimal run
#define ACTUAL_RUN_MAX_SPEED   255
#define TURN_SPEED             130  // Pivot turn speed

// Inching parameters (Forward verification of 400x400 mm end zone vs 30 mm line)
#define INCH_SPEED             110
#define INCH_TIME_MS           120  // Time to advance past a 30mm stripe

// Turn safety timeout (prevents infinite spinning if line is lost)
#define TURN_TIMEOUT_MS        1200

// ==========================================
// 3. PID CONTROLLER TUNING CONSTANTS
// ==========================================
// Setpoint is centered between sensor 0 and sensor 7:
// Range: 0 to 7000, Setpoint = 3500
#define SENSOR_SETPOINT        3500.0f

#define DEFAULT_KP             0.045f
#define DEFAULT_KI             0.0001f
#define DEFAULT_KD             0.35f

// ==========================================
// 4. MAZE SOLVER PARAMETERS
// ==========================================
#define MAX_PATH_LENGTH        150

#endif // CONFIG_H
