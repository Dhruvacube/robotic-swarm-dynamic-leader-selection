/*
 * SensorManager.cpp
 * 
 * Implementation of the sensor manager. Reads all onboard sensors
 * and computes health metrics for the leader election algorithm.
 */

#include "SensorManager.h"

// =============================================================================
// Constructor
// =============================================================================

SensorManager::SensorManager()
    : _imuOk(false), _lastImuUpdateMs(0),
      _roll(0), _pitch(0), _yaw(0),
      _ax(0), _ay(0), _az(0), _gx(0), _gy(0), _gz(0),
      _ultrasonic(nullptr), _distanceCm(-1), _ultraOk(false), _lastUltraMs(0),
      _irPin(-1), _irValue(0), _irOk(false),
      _enc1PinA(-1), _enc1PinB(-1), _enc2PinA(-1), _enc2PinB(-1),
      _enc1Ticks(0), _enc2Ticks(0), _encOk(false),
      _lastEncChangeMs(0), _lastEnc1Ticks(0),
      _battPin(-1), _battVRef(3.3f), _battDividerRatio(2.0f), _battEnabled(false) {
    for (int i = 0; i < NUM_SENSORS; i++) _health[i] = 0.0f;
}

// =============================================================================
// Initialisation
// =============================================================================

bool SensorManager::begin(int ultraTrigPin, int ultraEchoPin,
                          int irPin,
                          int enc1PinA, int enc1PinB,
                          int enc2PinA, int enc2PinB) {
    bool allOk = true;

    // --- IMU (MPU6050 + Madgwick) ---
    if (_mpu.begin()) {
        _mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        _mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        _mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        _imuOk = true;
        _lastImuUpdateMs = millis();
        Serial.println(F("[SensorMgr] MPU6050 OK"));
    } else {
        _imuOk = false;
        allOk = false;
        Serial.println(F("[SensorMgr] MPU6050 FAIL"));
    }

    // --- Ultrasonic (HC-SR04) ---
    _ultrasonic = new UltraSonicDistanceSensor(ultraTrigPin, ultraEchoPin);
    _ultraOk = true;  // HC-SR04 doesn't have an init check
    _lastUltraMs = millis();
    Serial.println(F("[SensorMgr] Ultrasonic OK"));

    // --- IR sensor (analog) ---
    _irPin = irPin;
    if (_irPin >= 0) {
        pinMode(_irPin, INPUT);
        _irOk = true;
        Serial.println(F("[SensorMgr] IR sensor OK"));
    } else {
        _irOk = false;
    }

    // --- Encoders ---
    _enc1PinA = enc1PinA;
    _enc1PinB = enc1PinB;
    _enc2PinA = enc2PinA;
    _enc2PinB = enc2PinB;
    if (_enc1PinA >= 0) {
        pinMode(_enc1PinA, INPUT_PULLUP);
        pinMode(_enc1PinB, INPUT_PULLUP);
        pinMode(_enc2PinA, INPUT_PULLUP);
        pinMode(_enc2PinB, INPUT_PULLUP);
        _encOk = true;
        _lastEncChangeMs = millis();
        Serial.println(F("[SensorMgr] Encoders OK"));
    }

    delay(100);
    return allOk;
}

// =============================================================================
// Update — Read all sensors
// =============================================================================

void SensorManager::update() {
    unsigned long now = millis();

    // --- IMU ---
    if (_imuOk) {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);

        // Convert to Madgwick-compatible format
        _gx = g.gyro.x * (250.0f / 32768.0f);
        _gy = g.gyro.y * (250.0f / 32768.0f);
        _gz = g.gyro.z * (250.0f / 32768.0f);
        _ax = a.acceleration.x * (2.0f / 32768.0f);
        _ay = a.acceleration.y * (2.0f / 32768.0f);
        _az = a.acceleration.z * (2.0f / 32768.0f);

        _filter.updateIMU(_gx, _gy, _gz, _ax, _ay, _az);
        _roll  = _filter.getRoll();
        _pitch = _filter.getPitch();
        _yaw   = _filter.getYaw();
        _lastImuUpdateMs = now;
    }

    // --- Ultrasonic ---
    if (_ultraOk && _ultrasonic) {
        float d = _ultrasonic->measureDistanceCm();
        if (d > 0) {
            _distanceCm = d;
            _lastUltraMs = now;
        }
    }

    // --- IR ---
    if (_irOk && _irPin >= 0) {
        int raw = analogRead(_irPin);
        _irValue = raw / 4095.0f;  // Normalise to [0, 1]
    }

    // --- Encoders ---
    if (_encOk) {
        // Track whether ticks are changing (motor is moving)
        if (_enc1Ticks != _lastEnc1Ticks) {
            _lastEncChangeMs = now;
            _lastEnc1Ticks = _enc1Ticks;
        }
    }

    // --- Compute health values ---
    _health[SENSOR_IMU]        = computeIMUHealth();
    _health[SENSOR_ULTRASONIC] = computeUltrasonicHealth();
    _health[SENSOR_IR]         = computeIRHealth();
    _health[SENSOR_GPS]        = computeGPSHealth();
    _health[SENSOR_ENCODER]    = computeEncoderHealth();
}

// =============================================================================
// Health computation
// =============================================================================

float SensorManager::computeIMUHealth() {
    if (!_imuOk) return 0.0f;
    // Health degrades if IMU hasn't updated recently
    unsigned long age = millis() - _lastImuUpdateMs;
    if (age > 2000) return 0.1f;  // Very stale
    if (age > 500)  return 0.5f;  // Somewhat stale
    // Check for reasonable acceleration magnitude (should be ~1g when still)
    float magSq = _ax * _ax + _ay * _ay + _az * _az;
    if (magSq < 0.001f) return 0.3f;  // Likely disconnected
    return 0.95f;
}

float SensorManager::computeUltrasonicHealth() {
    if (!_ultraOk) return 0.0f;
    unsigned long age = millis() - _lastUltraMs;
    if (age > 3000) return 0.1f;
    if (_distanceCm < 0) return 0.2f;  // Invalid reading
    if (_distanceCm > 400) return 0.5f;  // Out of range
    return 0.9f;
}

float SensorManager::computeIRHealth() {
    if (!_irOk) return 0.0f;
    // If IR reads extremes, might be malfunctioning
    if (_irValue < 0.01f || _irValue > 0.99f) return 0.4f;
    return 0.85f;
}

float SensorManager::computeGPSHealth() {
    // GPS is not yet wired in the current hardware revision.
    // Return a moderate baseline so the algorithm still functions.
    // In production, this would check NMEA fix quality, HDOP, etc.
    return 0.6f;
}

float SensorManager::computeEncoderHealth() {
    if (!_encOk) return 0.0f;
    // Encoder is "healthy" if it has changed recently (i.e. motors are running)
    unsigned long age = millis() - _lastEncChangeMs;
    if (age > 10000) return 0.3f;   // Very stale — robot might be stationary
    if (age > 3000)  return 0.6f;   // Somewhat stale
    return 0.9f;
}

// =============================================================================
// Accessors
// =============================================================================

void SensorManager::getHealthArray(float outHealth[NUM_SENSORS]) const {
    for (int i = 0; i < NUM_SENSORS; i++) outHealth[i] = _health[i];
}

float SensorManager::getHealth(SensorIndex sensor) const {
    if (sensor < NUM_SENSORS) return _health[sensor];
    return 0.0f;
}

float SensorManager::getRoll() const { return _roll; }
float SensorManager::getPitch() const { return _pitch; }
float SensorManager::getYaw() const { return _yaw; }
float SensorManager::getDistanceCm() const { return _distanceCm; }
float SensorManager::getIRValue() const { return _irValue; }
int   SensorManager::getEncoder1Ticks() const { return _enc1Ticks; }
int   SensorManager::getEncoder2Ticks() const { return _enc2Ticks; }

// =============================================================================
// Battery
// =============================================================================

void SensorManager::setBatteryPin(int pin, float vRef, float dividerRatio) {
    _battPin = pin;
    _battVRef = vRef;
    _battDividerRatio = dividerRatio;
    _battEnabled = (pin >= 0);
    if (_battEnabled) {
        pinMode(pin, INPUT);
    }
}

float SensorManager::readBatteryVoltage() {
    if (!_battEnabled) return -1.0f;
    int raw = analogRead(_battPin);
    float voltage = (raw / 4095.0f) * _battVRef * _battDividerRatio;
    return voltage;
}

float SensorManager::readBatteryPercent(float vMin, float vMax) {
    float v = readBatteryVoltage();
    if (v < 0) return 100.0f;  // No battery pin — assume full
    return constrain((v - vMin) / (vMax - vMin) * 100.0f, 0.0f, 100.0f);
}
