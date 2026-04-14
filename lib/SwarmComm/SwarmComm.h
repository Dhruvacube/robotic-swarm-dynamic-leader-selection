/*
 * SwarmComm.h
 * 
 * RF24-based communication protocol for the robotic swarm.
 * Handles broadcasting leader election data (sensor health, battery,
 * scores) between robots using nRF24L01+ radios.
 * 
 * Protocol:
 *   - Each robot periodically broadcasts its RobotScoreData
 *   - Other robots receive and update their peer tables
 *   - ACK payloads carry the responder's data for two-way exchange
 */

#ifndef SWARM_COMM_H
#define SWARM_COMM_H

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "LeaderElection.h"  // for RobotScoreData, NUM_SENSORS

// --- Communication packet ---------------------------------------------------

// Compact packet for over-the-air transmission (kept small for nRF24)
struct __attribute__((packed)) SwarmPacket {
    uint8_t  senderId;                        // Who sent this
    uint8_t  leaderId;                        // Who this robot thinks is leader
    uint8_t  sensorHealthPacked[NUM_SENSORS]; // Health * 200, packed as uint8
    uint8_t  batteryPacked;                   // Battery % * 2 (0-200)
    uint8_t  proximityPacked;                 // Proximity * 200
    uint16_t sequenceNum;                     // For duplicate detection

    // Convert from RobotScoreData
    void fromScoreData(const RobotScoreData& data, uint8_t leader, uint16_t seq) {
        senderId = data.robotId;
        leaderId = leader;
        for (int i = 0; i < NUM_SENSORS; i++) {
            sensorHealthPacked[i] = (uint8_t)(constrain(data.sensorHealth[i], 0.0f, 1.0f) * 200);
        }
        batteryPacked   = (uint8_t)(constrain(data.batteryLevel, 0.0f, 100.0f) * 2);
        proximityPacked = (uint8_t)(constrain(data.proximityScore, 0.0f, 1.0f) * 200);
        sequenceNum = seq;
    }

    // Convert to RobotScoreData
    RobotScoreData toScoreData() const {
        RobotScoreData data;
        data.robotId = senderId;
        for (int i = 0; i < NUM_SENSORS; i++) {
            data.sensorHealth[i] = sensorHealthPacked[i] / 200.0f;
        }
        data.batteryLevel   = batteryPacked / 2.0f;
        data.proximityScore = proximityPacked / 200.0f;
        data.isAlive = true;
        data.lastHeartbeatMs = millis();
        return data;
    }
};

// --- SwarmComm class ---------------------------------------------------------

class SwarmComm {
public:
    SwarmComm(uint8_t cePin, uint8_t csnPin);

    // Initialise the radio for swarm communication
    bool begin(uint8_t myRobotId, uint8_t swarmSize);

    // Set the broadcast interval (default 200ms)
    void setBroadcastInterval(unsigned long intervalMs);

    // Call every loop — handles sending and receiving
    //   Returns true if a new packet was received this call
    bool update(const RobotScoreData& myData, uint8_t currentLeader);

    // Get the last received packet data
    const SwarmPacket& getLastReceived() const;
    bool  hasNewData() const;
    void  clearNewData();

    // Statistics
    unsigned long getTxCount() const;
    unsigned long getRxCount() const;
    unsigned long getFailCount() const;

    // Debug
    void printStatus() const;

private:
    RF24            _radio;
    uint8_t         _myId;
    uint8_t         _swarmSize;
    uint16_t        _seqNum;
    unsigned long   _broadcastIntervalMs;
    unsigned long   _lastBroadcastMs;
    unsigned long   _txCount;
    unsigned long   _rxCount;
    unsigned long   _failCount;
    bool            _newData;
    SwarmPacket     _txPacket;
    SwarmPacket     _rxPacket;

    // Pipe addresses (up to 6 for nRF24)
    static const uint8_t PIPE_ADDRESSES[][6];
};

#endif // SWARM_COMM_H
