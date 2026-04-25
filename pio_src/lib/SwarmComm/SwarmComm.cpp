/*
 * SwarmComm.cpp
 * 
 * Implementation of the RF24-based swarm communication protocol.
 * Handles periodic broadcasting and receiving of leader election data.
 */

#include "SwarmComm.h"

// Pipe addresses for swarm communication (shared across all nodes)
const uint8_t SwarmComm::PIPE_ADDRESSES[][6] = {
    "Swrm0", "Swrm1", "Swrm2", "Swrm3", "Swrm4", "Swrm5"
};

// =============================================================================
// Constructor
// =============================================================================

SwarmComm::SwarmComm(uint8_t cePin, uint8_t csnPin)
    : _radio(cePin, csnPin),
      _myId(0), _swarmSize(1), _seqNum(0),
      _broadcastIntervalMs(200), _lastBroadcastMs(0),
      _txCount(0), _rxCount(0), _failCount(0), _newData(false) {}

// =============================================================================
// Initialisation
// =============================================================================

bool SwarmComm::begin(uint8_t myRobotId, uint8_t swarmSize) {
    _myId = myRobotId;
    _swarmSize = min(swarmSize, (uint8_t)6);  // nRF24 has max 6 pipes

    if (!_radio.begin()) {
        Serial.println(F("[SwarmComm] Radio hardware FAIL"));
        return false;
    }

    Serial.println(F("[SwarmComm] Radio hardware OK"));

    // Configure radio
    _radio.setPALevel(RF24_PA_LOW);       // Low power for close-range swarm
    _radio.setDataRate(RF24_1MBPS);       // Good balance of range and speed
    _radio.setChannel(108);               // Use channel away from WiFi
    _radio.enableDynamicPayloads();
    _radio.enableAckPayload();
    _radio.setRetries(3, 5);              // 3 * 250us delay, 5 retries

    // Open writing pipe (our own address)
    _radio.openWritingPipe(PIPE_ADDRESSES[_myId]);

    // Open reading pipes for all other robots
    uint8_t pipeIdx = 1;  // Pipe 0 is reserved for writing
    for (uint8_t i = 0; i < _swarmSize; i++) {
        if (i != _myId && pipeIdx <= 5) {
            _radio.openReadingPipe(pipeIdx, PIPE_ADDRESSES[i]);
            pipeIdx++;
        }
    }

    _radio.startListening();
    _lastBroadcastMs = millis();

    Serial.print(F("[SwarmComm] Initialised as Robot "));
    Serial.print(_myId);
    Serial.print(F(" in swarm of "));
    Serial.println(_swarmSize);

    return true;
}

// =============================================================================
// Configuration
// =============================================================================

void SwarmComm::setBroadcastInterval(unsigned long intervalMs) {
    _broadcastIntervalMs = intervalMs;
}

// =============================================================================
// Main update loop
// =============================================================================

bool SwarmComm::update(const RobotScoreData& myData, uint8_t currentLeader) {
    unsigned long now = millis();
    _newData = false;

    // --- Check for incoming data ---
    uint8_t pipe;
    if (_radio.available(&pipe)) {
        _radio.read(&_rxPacket, sizeof(SwarmPacket));
        _rxCount++;
        _newData = true;

        // Prepare ACK payload with our own data
        _txPacket.fromScoreData(myData, currentLeader, _seqNum);
        _radio.writeAckPayload(pipe, &_txPacket, sizeof(SwarmPacket));
    }

    // --- Periodic broadcast ---
    if (now - _lastBroadcastMs >= _broadcastIntervalMs) {
        _lastBroadcastMs = now;
        _seqNum++;

        _txPacket.fromScoreData(myData, currentLeader, _seqNum);

        // Briefly switch to TX mode to broadcast
        _radio.stopListening();

        // Broadcast to each peer (round-robin to different addresses)
        for (uint8_t i = 0; i < _swarmSize; i++) {
            if (i == _myId) continue;

            _radio.openWritingPipe(PIPE_ADDRESSES[i]);
            bool ok = _radio.write(&_txPacket, sizeof(SwarmPacket));

            if (ok) {
                _txCount++;
                // Check for ACK payload
                if (_radio.isAckPayloadAvailable()) {
                    SwarmPacket ackPacket;
                    _radio.read(&ackPacket, sizeof(SwarmPacket));
                    // Treat the ACK packet as received data too
                    _rxPacket = ackPacket;
                    _rxCount++;
                    _newData = true;
                }
            } else {
                _failCount++;
            }
        }

        // Switch back to RX mode
        _radio.openWritingPipe(PIPE_ADDRESSES[_myId]);
        _radio.startListening();
    }

    return _newData;
}

// =============================================================================
// Accessors
// =============================================================================

const SwarmPacket& SwarmComm::getLastReceived() const {
    return _rxPacket;
}

bool SwarmComm::hasNewData() const {
    return _newData;
}

void SwarmComm::clearNewData() {
    _newData = false;
}

unsigned long SwarmComm::getTxCount() const { return _txCount; }
unsigned long SwarmComm::getRxCount() const { return _rxCount; }
unsigned long SwarmComm::getFailCount() const { return _failCount; }

// =============================================================================
// Debug
// =============================================================================

void SwarmComm::printStatus() const {
    Serial.print(F("[SwarmComm] TX="));
    Serial.print(_txCount);
    Serial.print(F("  RX="));
    Serial.print(_rxCount);
    Serial.print(F("  FAIL="));
    Serial.println(_failCount);
}
