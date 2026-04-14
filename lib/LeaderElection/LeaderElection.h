/*
 * LeaderElection.h
 * 
 * Dynamic Leader Selection Algorithm for Robotic Swarms.
 * Implements weighted multi-criteria scoring with hysteresis-based
 * anti-flapping to select the best leader in a master-slave swarm.
 * 
 * Algorithm:
 *   LeaderScore = alpha * SensorScore + beta * ProximityScore + gamma * BatteryScore
 * 
 * Features:
 *   - Weighted sensor health scoring (configurable per mission profile)
 *   - Proximity-to-centroid factor
 *   - Battery level integration
 *   - Hysteresis threshold to prevent rapid leader switching (flapping)
 *   - Supports N robots (up to MAX_SWARM_SIZE)
 */

#ifndef LEADER_ELECTION_H
#define LEADER_ELECTION_H

#include <Arduino.h>

// --- Configuration -----------------------------------------------------------

#define MAX_SWARM_SIZE      8     // Maximum number of robots in the swarm
#define NUM_SENSORS         5     // IMU, Ultrasonic, IR, GPS, Encoder

// Sensor indices (canonical order)
enum SensorIndex {
    SENSOR_IMU        = 0,
    SENSOR_ULTRASONIC = 1,
    SENSOR_IR         = 2,
    SENSOR_GPS        = 3,
    SENSOR_ENCODER    = 4
};

// --- Sensor Weight Profile ---------------------------------------------------

struct SensorWeightProfile {
    float weights[NUM_SENSORS];   // Weights for each sensor (will be normalised)

    // Default outdoor navigation profile
    SensorWeightProfile()
        : weights{0.20f, 0.15f, 0.10f, 0.30f, 0.25f} {}

    float normalise() {
        float sum = 0;
        for (int i = 0; i < NUM_SENSORS; i++) sum += weights[i];
        if (sum > 0) {
            for (int i = 0; i < NUM_SENSORS; i++) weights[i] /= sum;
        }
        return sum;
    }
};

// --- Election Configuration --------------------------------------------------

struct ElectionConfig {
    float alpha;                  // Sensor health weight in composite score
    float beta;                   // Proximity-to-centroid weight
    float gamma;                  // Battery level weight
    float hysteresisThreshold;    // Minimum margin to unseat current leader

    ElectionConfig()
        : alpha(0.55f), beta(0.25f), gamma(0.20f),
          hysteresisThreshold(0.08f) {}
};

// --- Robot Score Data --------------------------------------------------------

struct RobotScoreData {
    uint8_t  robotId;
    float    sensorHealth[NUM_SENSORS];  // Per-sensor health [0.0 - 1.0]
    float    batteryLevel;               // Battery percentage [0.0 - 100.0]
    float    proximityScore;             // Proximity to centroid [0.0 - 1.0]
    float    compositeScore;             // Computed leader score
    bool     isAlive;                    // Whether this robot is online
    unsigned long lastHeartbeatMs;       // Timestamp of last heartbeat

    RobotScoreData()
        : robotId(0), batteryLevel(100.0f), proximityScore(0.5f),
          compositeScore(0.0f), isAlive(false), lastHeartbeatMs(0) {
        for (int i = 0; i < NUM_SENSORS; i++) sensorHealth[i] = 0.0f;
    }
};

// --- Leader Election Engine --------------------------------------------------

class LeaderElection {
public:
    LeaderElection();

    // Configuration
    void begin(uint8_t myRobotId, uint8_t swarmSize);
    void setWeights(const SensorWeightProfile& profile);
    void setElectionConfig(const ElectionConfig& config);

    // Update own sensor health values
    void updateSensorHealth(SensorIndex sensor, float health);
    void updateAllSensorHealth(const float health[NUM_SENSORS]);
    void updateBattery(float batteryPercent);
    void updateProximity(float proximityScore);

    // Update peer data (received via radio)
    void updatePeerData(uint8_t peerId, const RobotScoreData& data);
    void markPeerAlive(uint8_t peerId, unsigned long timestampMs);

    // Run election
    uint8_t runElection();          // Returns the elected leader ID
    bool    amILeader() const;
    uint8_t getCurrentLeader() const;
    float   getMyScore() const;
    float   getPeerScore(uint8_t peerId) const;

    // Fault detection
    void    checkHeartbeats(unsigned long nowMs, unsigned long timeoutMs = 5000);

    // Get own data (for broadcasting)
    const RobotScoreData& getMyData() const;

    // Debug
    void    printScores() const;
    uint8_t getAliveCount() const;

private:
    float computeSensorScore(const RobotScoreData& data) const;
    float computeCompositeScore(const RobotScoreData& data) const;

    uint8_t             _myId;
    uint8_t             _swarmSize;
    uint8_t             _currentLeader;
    bool                _leaderKnown;
    SensorWeightProfile _weights;
    ElectionConfig      _electionCfg;
    RobotScoreData      _robots[MAX_SWARM_SIZE];
};

#endif // LEADER_ELECTION_H
