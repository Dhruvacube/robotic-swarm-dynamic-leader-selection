/*
 * LeaderElection.cpp
 * 
 * Implementation of the improved dynamic leader selection algorithm.
 */

#include "LeaderElection.h"

// =============================================================================
// Constructor
// =============================================================================

LeaderElection::LeaderElection()
    : _myId(0), _swarmSize(1), _currentLeader(0), _leaderKnown(false) {
    _weights.normalise();
}

// =============================================================================
// Configuration
// =============================================================================

void LeaderElection::begin(uint8_t myRobotId, uint8_t swarmSize) {
    _myId = myRobotId;
    _swarmSize = min(swarmSize, (uint8_t)MAX_SWARM_SIZE);
    _leaderKnown = false;
    _currentLeader = 0;

    for (uint8_t i = 0; i < _swarmSize; i++) {
        _robots[i].robotId = i;
        _robots[i].isAlive = (i == _myId);  // Only we are confirmed alive at start
        if (i == _myId) {
            // Set initial self health to high
            for (int s = 0; s < NUM_SENSORS; s++) {
                _robots[i].sensorHealth[s] = 0.9f;
            }
            _robots[i].batteryLevel = 100.0f;
            _robots[i].proximityScore = 0.5f;
            _robots[i].lastHeartbeatMs = millis();
        }
    }
}

void LeaderElection::setWeights(const SensorWeightProfile& profile) {
    _weights = profile;
    _weights.normalise();
}

void LeaderElection::setElectionConfig(const ElectionConfig& config) {
    _electionCfg = config;
}

// =============================================================================
// Update own data
// =============================================================================

void LeaderElection::updateSensorHealth(SensorIndex sensor, float health) {
    if (sensor < NUM_SENSORS) {
        _robots[_myId].sensorHealth[sensor] = constrain(health, 0.0f, 1.0f);
    }
}

void LeaderElection::updateAllSensorHealth(const float health[NUM_SENSORS]) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        _robots[_myId].sensorHealth[i] = constrain(health[i], 0.0f, 1.0f);
    }
}

void LeaderElection::updateBattery(float batteryPercent) {
    _robots[_myId].batteryLevel = constrain(batteryPercent, 0.0f, 100.0f);
}

void LeaderElection::updateProximity(float proximityScore) {
    _robots[_myId].proximityScore = constrain(proximityScore, 0.0f, 1.0f);
}

// =============================================================================
// Update peer data (from radio)
// =============================================================================

void LeaderElection::updatePeerData(uint8_t peerId, const RobotScoreData& data) {
    if (peerId < _swarmSize && peerId != _myId) {
        _robots[peerId] = data;
        _robots[peerId].robotId = peerId;
        _robots[peerId].isAlive = true;
        _robots[peerId].lastHeartbeatMs = millis();
    }
}

void LeaderElection::markPeerAlive(uint8_t peerId, unsigned long timestampMs) {
    if (peerId < _swarmSize) {
        _robots[peerId].isAlive = true;
        _robots[peerId].lastHeartbeatMs = timestampMs;
    }
}

// =============================================================================
// Score computation
// =============================================================================

float LeaderElection::computeSensorScore(const RobotScoreData& data) const {
    float score = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) {
        score += _weights.weights[i] * data.sensorHealth[i];
    }
    return score;
}

float LeaderElection::computeCompositeScore(const RobotScoreData& data) const {
    float sensorScore    = computeSensorScore(data);
    float batteryScore   = data.batteryLevel / 100.0f;
    float proximityScore = data.proximityScore;

    return _electionCfg.alpha * sensorScore
         + _electionCfg.beta  * proximityScore
         + _electionCfg.gamma * batteryScore;
}

// =============================================================================
// Election — the core algorithm
// =============================================================================

uint8_t LeaderElection::runElection() {
    // 1. Compute composite scores for all alive robots
    for (uint8_t i = 0; i < _swarmSize; i++) {
        if (_robots[i].isAlive) {
            _robots[i].compositeScore = computeCompositeScore(_robots[i]);
        } else {
            _robots[i].compositeScore = -1.0f;
        }
    }

    // 2. Find the candidate with the highest score (tie-break: lowest ID)
    uint8_t candidateId = _myId;
    float   candidateScore = -1.0f;

    for (uint8_t i = 0; i < _swarmSize; i++) {
        if (!_robots[i].isAlive) continue;
        if (_robots[i].compositeScore > candidateScore ||
            (_robots[i].compositeScore == candidateScore && i < candidateId)) {
            candidateScore = _robots[i].compositeScore;
            candidateId = i;
        }
    }

    // 3. If no leader is known yet, or current leader is dead -> elect immediately
    if (!_leaderKnown || !_robots[_currentLeader].isAlive) {
        _currentLeader = candidateId;
        _leaderKnown = true;
        return _currentLeader;
    }

    // 4. Hysteresis check — only switch if margin exceeds threshold
    float currentScore = _robots[_currentLeader].compositeScore;
    float margin = candidateScore - currentScore;

    if (candidateId != _currentLeader && margin > _electionCfg.hysteresisThreshold) {
        // Switch to new leader
        _currentLeader = candidateId;
        Serial.print(F("[Election] Leader changed to Robot "));
        Serial.print(_currentLeader);
        Serial.print(F(" (margin="));
        Serial.print(margin, 3);
        Serial.println(F(")"));
    }

    return _currentLeader;
}

// =============================================================================
// Queries
// =============================================================================

bool LeaderElection::amILeader() const {
    return _leaderKnown && (_currentLeader == _myId);
}

uint8_t LeaderElection::getCurrentLeader() const {
    return _currentLeader;
}

float LeaderElection::getMyScore() const {
    return _robots[_myId].compositeScore;
}

float LeaderElection::getPeerScore(uint8_t peerId) const {
    if (peerId < _swarmSize) return _robots[peerId].compositeScore;
    return -1.0f;
}

const RobotScoreData& LeaderElection::getMyData() const {
    return _robots[_myId];
}

uint8_t LeaderElection::getAliveCount() const {
    uint8_t count = 0;
    for (uint8_t i = 0; i < _swarmSize; i++) {
        if (_robots[i].isAlive) count++;
    }
    return count;
}

// =============================================================================
// Fault detection
// =============================================================================

void LeaderElection::checkHeartbeats(unsigned long nowMs, unsigned long timeoutMs) {
    for (uint8_t i = 0; i < _swarmSize; i++) {
        if (i == _myId) continue;  // Don't timeout ourselves
        if (_robots[i].isAlive && (nowMs - _robots[i].lastHeartbeatMs > timeoutMs)) {
            _robots[i].isAlive = false;
            Serial.print(F("[Fault] Robot "));
            Serial.print(i);
            Serial.println(F(" timed out — marked DEAD"));
        }
    }
}

// =============================================================================
// Debug
// =============================================================================

void LeaderElection::printScores() const {
    Serial.println(F("--- Swarm Scores ---"));
    for (uint8_t i = 0; i < _swarmSize; i++) {
        Serial.print(F("  R"));
        Serial.print(i);
        if (i == _currentLeader && _leaderKnown) {
            Serial.print(F("*LEADER"));
        }
        if (!_robots[i].isAlive) {
            Serial.println(F("  [DEAD]"));
            continue;
        }
        Serial.print(F("  score="));
        Serial.print(_robots[i].compositeScore, 3);
        Serial.print(F("  batt="));
        Serial.print(_robots[i].batteryLevel, 0);
        Serial.print(F("%  sens=["));
        for (int s = 0; s < NUM_SENSORS; s++) {
            Serial.print(_robots[i].sensorHealth[s], 2);
            if (s < NUM_SENSORS - 1) Serial.print(F(","));
        }
        Serial.println(F("]"));
    }
    Serial.println(F("--------------------"));
}
