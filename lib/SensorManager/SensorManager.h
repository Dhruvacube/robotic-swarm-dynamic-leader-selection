/*
 * SensorManager.h
 * 
 * Manages all onboard sensors and computes per-sensor health values
 * for the leader election algorithm. Wraps MPU6050 (IMU), HC-SR04
 * (ultrasonic), IR sensor, GPS, and wheel encoders.
 * 
 * Health metric: Each sensor's health is reported as a float [0.0 - 1.0]
 * based on whether it's responding, its update rate, and signal quality.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <HCSR04.h>
#include "LeaderElection.h"  // for NUM_SENSORS, SensorIndex

class SensorManager {
public:
    SensorManager();

    // Initialise all sensors
    bool begin(int ultraTrigPin, int ultraEchoPin,
               int irPin,
               int enc1PinA, int enc1PinB,
               int enc2PinA, int enc2PinB);

    // Call every loop iteration to read sensors and update health
    void update();

    // Get health values for all sensors [0.0 - 1.0]
    void getHealthArray(float outHealth[NUM_SENSORS]) const;
    float getHealth(SensorIndex sensor) const;

    // Raw sensor accessors
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
    float getDistanceCm() const;
    float getIRValue() const;
    int   getEncoder1Ticks() const;
    int   getEncoder2Ticks() const;

    // Battery monitoring (via ADC pin)
    void  setBatteryPin(int pin, float vRef = 3.3f, float dividerRatio = 2.0f);
    float readBatteryVoltage();
    float readBatteryPercent(float vMin = 3.0f, float vMax = 4.2f);

private:
    // IMU
    Adafruit_MPU6050 _mpu;
    Madgwick         _filter;
    bool             _imuOk;
    unsigned long    _lastImuUpdateMs;
    float            _roll, _pitch, _yaw;
    float            _ax, _ay, _az, _gx, _gy, _gz;

    // Ultrasonic
    UltraSonicDistanceSensor* _ultrasonic;
    float            _distanceCm;
    bool             _ultraOk;
    unsigned long    _lastUltraMs;

    // IR sensor (analog)
    int              _irPin;
    float            _irValue;
    bool             _irOk;

    // Encoders
    int              _enc1PinA, _enc1PinB;
    int              _enc2PinA, _enc2PinB;
    volatile int     _enc1Ticks;
    volatile int     _enc2Ticks;
    bool             _encOk;
    unsigned long    _lastEncChangeMs;
    int              _lastEnc1Ticks;

    // Battery ADC
    int              _battPin;
    float            _battVRef;
    float            _battDividerRatio;
    bool             _battEnabled;

    // Health values
    float            _health[NUM_SENSORS];

    // Helper
    float computeIMUHealth();
    float computeUltrasonicHealth();
    float computeIRHealth();
    float computeGPSHealth();     // Placeholder — GPS not yet wired
    float computeEncoderHealth();
};

#endif // SENSOR_MANAGER_H
