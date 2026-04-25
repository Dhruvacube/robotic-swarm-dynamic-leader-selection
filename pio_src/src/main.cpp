/*
 * main.cpp — Robotic Swarm Dynamic Leader Selection
 * 
 * Main firmware for each robot in the swarm. Integrates:
 *   - SensorManager:   Reads all onboard sensors, computes health values
 *   - LeaderElection:  Weighted multi-criteria scoring with hysteresis
 *   - SwarmComm:       RF24 radio communication for score exchange
 *   - MotorControl:    Differential drive motor control
 * 
 * Each robot runs this same firmware with a different MY_ROBOT_ID
 * configured in SwarmConfig.h. The algorithm automatically elects
 * the best robot as Master; all others become Slaves.
 * 
 * The Master robot controls swarm navigation. Slave robots follow
 * the Master's commands relayed via radio.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Project libraries
#include "SwarmConfig.h"
#include "MotorControl.h"
#include "LeaderElection.h"
#include "SensorManager.h"
#include "SwarmComm.h"

// =============================================================================
// Forward declarations
// =============================================================================

void masterBehaviour();
void slaveBehaviour(uint8_t leaderId);
void printStatus();

// =============================================================================
// Global objects
// =============================================================================

MotorControl     motors(MOTOR1_PIN1, MOTOR1_PIN2, MOTOR2_PIN1, MOTOR2_PIN2);
SensorManager    sensors;
LeaderElection   election;
SwarmComm        radio(RF24_CE_PIN, RF24_CSN_PIN);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDR);

// Encoder tick counters (updated by ISR)
volatile int enc1Ticks = 0;
volatile int enc2Ticks = 0;

// Timing
unsigned long lastSensorMs   = 0;
unsigned long lastElectionMs = 0;
unsigned long lastStatusMs   = 0;
unsigned long lastLedMs      = 0;
bool          ledState       = false;

// =============================================================================
// Encoder ISRs
// =============================================================================

void IRAM_ATTR encoderISR1() {
    if (digitalRead(ENCODER1_PIN_A) == digitalRead(ENCODER1_PIN_B)) {
        enc1Ticks++;
    } else {
        enc1Ticks--;
    }
}

void IRAM_ATTR encoderISR2() {
    if (digitalRead(ENCODER2_PIN_A) == digitalRead(ENCODER2_PIN_B)) {
        enc2Ticks++;
    } else {
        enc2Ticks--;
    }
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println(F("==========================================="));
    Serial.println(F("  Robotic Swarm - Dynamic Leader Selection"));
    Serial.print(F("  Robot ID: "));
    Serial.print(MY_ROBOT_ID);
    Serial.print(F(" / Swarm Size: "));
    Serial.println(SWARM_SIZE);
    Serial.println(F("==========================================="));

    // --- LED ---
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN_PIN, LOW);

    // --- I2C & PWM ---
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(60);

    // --- Encoder interrupts ---
    attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), encoderISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), encoderISR2, RISING);

    // --- Sensor manager ---
    Serial.println(F("\n[Init] Sensor Manager..."));
    sensors.begin(
        ULTRASONIC_TRIG, ULTRASONIC_ECHO,
        IR_SENSOR_PIN,
        ENCODER1_PIN_A, ENCODER1_PIN_B,
        ENCODER2_PIN_A, ENCODER2_PIN_B
    );
    sensors.setBatteryPin(BATTERY_ADC_PIN, BATTERY_VREF, BATTERY_DIVIDER);

    // --- Leader election engine ---
    Serial.println(F("[Init] Leader Election..."));
    election.begin(MY_ROBOT_ID, SWARM_SIZE);

    // Configure sensor weights
    SensorWeightProfile profile;
    profile.weights[SENSOR_IMU]        = WEIGHT_IMU;
    profile.weights[SENSOR_ULTRASONIC] = WEIGHT_ULTRASONIC;
    profile.weights[SENSOR_IR]         = WEIGHT_IR;
    profile.weights[SENSOR_GPS]        = WEIGHT_GPS;
    profile.weights[SENSOR_ENCODER]    = WEIGHT_ENCODER;
    election.setWeights(profile);

    // Configure election parameters
    ElectionConfig eCfg;
    eCfg.alpha = SCORE_ALPHA;
    eCfg.beta  = SCORE_BETA;
    eCfg.gamma = SCORE_GAMMA;
    eCfg.hysteresisThreshold = HYSTERESIS_TAU;
    election.setElectionConfig(eCfg);

    // --- Radio communication ---
    Serial.println(F("[Init] Swarm Communication..."));
    if (!radio.begin(MY_ROBOT_ID, SWARM_SIZE)) {
        Serial.println(F("[ERROR] Radio init failed!"));
    }
    radio.setBroadcastInterval(BROADCAST_INTERVAL_MS);

    Serial.println(F("\n[Init] Complete. Starting main loop.\n"));
}

// =============================================================================
// Main loop
// =============================================================================

void loop() {
    unsigned long now = millis();

    // -----------------------------------------------------------------
    // 1. Read sensors (fast — every SENSOR_UPDATE_MS)
    // -----------------------------------------------------------------
    if (now - lastSensorMs >= SENSOR_UPDATE_MS) {
        lastSensorMs = now;

        sensors.update();

        // Push sensor health into election engine
        float health[NUM_SENSORS];
        sensors.getHealthArray(health);
        election.updateAllSensorHealth(health);

        // Update battery level
        float battPercent = sensors.readBatteryPercent();
        election.updateBattery(battPercent);

        // TODO: Compute proximity to centroid from GPS/position data
        // For now use a default value; will be computed from peer positions
        election.updateProximity(0.5f);
    }

    // -----------------------------------------------------------------
    // 2. Radio communication (handles broadcasting + receiving)
    // -----------------------------------------------------------------
    bool gotNewData = radio.update(election.getMyData(), election.getCurrentLeader());

    if (gotNewData) {
        const SwarmPacket& pkt = radio.getLastReceived();
        RobotScoreData peerData = pkt.toScoreData();
        election.updatePeerData(pkt.senderId, peerData);
    }

    // -----------------------------------------------------------------
    // 3. Run leader election (every ELECTION_INTERVAL_MS)
    // -----------------------------------------------------------------
    if (now - lastElectionMs >= ELECTION_INTERVAL_MS) {
        lastElectionMs = now;

        // Check for peer timeouts
        election.checkHeartbeats(now, HEARTBEAT_TIMEOUT_MS);

        // Run the election
        uint8_t leader = election.runElection();

        // --- Behaviour based on role ---
        if (election.amILeader()) {
            // MASTER behaviour: autonomous navigation
            masterBehaviour();
        } else {
            // SLAVE behaviour: follow leader commands
            slaveBehaviour(leader);
        }
    }

    // -----------------------------------------------------------------
    // 4. LED indicator
    // -----------------------------------------------------------------
    if (election.amILeader()) {
        // Fast blink when leader
        if (now - lastLedMs >= LEADER_LED_BLINK_MS) {
            lastLedMs = now;
            ledState = !ledState;
            digitalWrite(LED_BUILTIN_PIN, ledState ? HIGH : LOW);
        }
    } else {
        // Solid ON when alive follower
        digitalWrite(LED_BUILTIN_PIN, HIGH);
    }

    // -----------------------------------------------------------------
    // 5. Debug status print (every STATUS_PRINT_MS)
    // -----------------------------------------------------------------
    if (now - lastStatusMs >= STATUS_PRINT_MS) {
        lastStatusMs = now;
        printStatus();
    }
}

// =============================================================================
// Role behaviours
// =============================================================================

void masterBehaviour() {
    // Master robot: obstacle avoidance + waypoint navigation
    float dist = sensors.getDistanceCm();

    if (dist > 0 && dist < 15.0f) {
        // Obstacle detected — avoid
        motors.stop();
        delay(100);
        motors.backward();
        delay(300);
        motors.right90();
        delay(400);
    } else {
        // Drive forward
        motors.forward();
    }
}

void slaveBehaviour(uint8_t leaderId) {
    // Slave robot: maintain formation, follow leader commands
    // In a full implementation, the leader would broadcast waypoints/commands
    // and slaves would navigate to maintain formation around the leader.
    
    // For now: basic obstacle avoidance while moving forward
    float dist = sensors.getDistanceCm();

    if (dist > 0 && dist < 10.0f) {
        motors.stop();
    } else {
        motors.forward();
    }
}

// =============================================================================
// Debug output
// =============================================================================

void printStatus() {
    Serial.println(F("-------------------------------------------"));
    Serial.print(F("Robot "));
    Serial.print(MY_ROBOT_ID);
    if (election.amILeader()) {
        Serial.print(F(" [MASTER]"));
    } else {
        Serial.print(F(" [SLAVE]"));
    }
    Serial.print(F("  Leader=R"));
    Serial.print(election.getCurrentLeader());
    Serial.print(F("  MyScore="));
    Serial.print(election.getMyScore(), 3);
    Serial.print(F("  Alive="));
    Serial.print(election.getAliveCount());
    Serial.print(F("/"));
    Serial.println(SWARM_SIZE);

    // Sensor readings
    Serial.print(F("  IMU: R="));
    Serial.print(sensors.getRoll(), 1);
    Serial.print(F(" P="));
    Serial.print(sensors.getPitch(), 1);
    Serial.print(F(" Y="));
    Serial.print(sensors.getYaw(), 1);
    Serial.print(F("  Dist="));
    Serial.print(sensors.getDistanceCm(), 1);
    Serial.print(F("cm  Batt="));
    Serial.print(sensors.readBatteryPercent(), 0);
    Serial.println(F("%"));

    // Election scores
    election.printScores();

    // Radio stats
    radio.printStatus();
    Serial.println(F("-------------------------------------------\n"));
}
