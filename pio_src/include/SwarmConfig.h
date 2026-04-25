/*
 * SwarmConfig.h
 * 
 * Project-level configuration for the robotic swarm.
 * All pin assignments, swarm parameters, and tuning constants
 * are centralised here for easy hardware adaptation.
 */

#ifndef SWARM_CONFIG_H
#define SWARM_CONFIG_H

// =============================================================================
// Robot Identity — CHANGE THIS PER ROBOT BEFORE FLASHING
// =============================================================================
#define MY_ROBOT_ID       0       // Unique ID for this robot (0, 1, 2, ...)
#define SWARM_SIZE        3       // Total number of robots in the swarm

// =============================================================================
// Pin Assignments — ESP32-DOIT-DevKit-V1
// =============================================================================

// Motor driver pins (L298N or similar H-bridge)
#define MOTOR1_PIN1       25      // Motor 1 forward
#define MOTOR1_PIN2       26      // Motor 1 reverse
#define MOTOR2_PIN1       27      // Motor 2 forward
#define MOTOR2_PIN2       14      // Motor 2 reverse

// Encoder pins
#define ENCODER1_PIN_A    21      // Motor 1 encoder channel A
#define ENCODER1_PIN_B    20      // Motor 1 encoder direction
#define ENCODER2_PIN_A    19      // Motor 2 encoder channel A
#define ENCODER2_PIN_B    18      // Motor 2 encoder direction

// Ultrasonic sensor (HC-SR04)
#define ULTRASONIC_TRIG   13
#define ULTRASONIC_ECHO   12

// IR sensor (analog)
#define IR_SENSOR_PIN     34      // ADC pin for IR proximity sensor

// nRF24L01+ radio
#define RF24_CE_PIN       4
#define RF24_CSN_PIN      5

// Battery voltage monitoring (via voltage divider)
#define BATTERY_ADC_PIN   35      // ADC pin for battery voltage
#define BATTERY_VREF      3.3f    // ESP32 ADC reference voltage
#define BATTERY_DIVIDER   2.0f    // Voltage divider ratio

// PWM servo driver (PCA9685)
#define PWM_I2C_ADDR      0x40

// =============================================================================
// Timing Parameters
// =============================================================================

#define ELECTION_INTERVAL_MS    500     // Run election every 500ms
#define BROADCAST_INTERVAL_MS   200     // Broadcast score data every 200ms
#define HEARTBEAT_TIMEOUT_MS    5000    // Declare peer dead after 5s silence
#define SENSOR_UPDATE_MS        50      // Read sensors every 50ms
#define STATUS_PRINT_MS         2000    // Print debug info every 2s
#define LEADER_LED_BLINK_MS     250     // LED blink rate when leader

// =============================================================================
// Algorithm Tuning
// =============================================================================

// Composite score weights (alpha + beta + gamma should sum to ~1.0)
#define SCORE_ALPHA       0.55f   // Sensor health weight
#define SCORE_BETA        0.25f   // Proximity-to-centroid weight
#define SCORE_GAMMA       0.20f   // Battery level weight

// Hysteresis threshold — minimum margin required to switch leader
#define HYSTERESIS_TAU    0.08f

// Sensor weight profile (will be normalised internally)
#define WEIGHT_IMU        0.20f
#define WEIGHT_ULTRASONIC 0.15f
#define WEIGHT_IR         0.10f
#define WEIGHT_GPS        0.30f
#define WEIGHT_ENCODER    0.25f

// =============================================================================
// LED Indicators
// =============================================================================
#define LED_BUILTIN_PIN   2       // ESP32 built-in LED (GPIO2)

#endif // SWARM_CONFIG_H
