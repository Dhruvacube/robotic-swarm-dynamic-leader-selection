# Robotic Swarm with Dynamic Leader Selection (Master-Slave Configuration)

A research project implementing an **improved dynamic leader selection algorithm** for robotic swarms. Each robot in the swarm continuously evaluates its eligibility to become the leader (Master) based on multi-criteria scoring — including sensor health, battery level, and proximity to the swarm centroid. The algorithm includes hysteresis-based anti-flapping, distributed consensus voting, and automatic fault detection with failover.

## Key Innovations

| Feature | Old Algorithm | Improved Algorithm |
|---------|:---:|:---:|
| Sensor scoring | Equal weight sum | Mission-profile weighted scoring |
| Score composition | Sensors only | Sensors + Battery + Proximity |
| Leader switching | Immediate (flapping) | Hysteresis threshold (anti-flapping) |
| Consensus | None | Majority vote confirmation |
| Fault tolerance | None | Heartbeat timeout + auto re-election |
| Scalability | Fixed 3 robots | N robots (configurable) |
| Tie-breaking | Undefined | Lowest robot ID wins |

## Algorithm Overview

### Composite Leader Score

Each robot computes its eligibility score as:

```
L_i = α × SensorScore_i + β × ProximityScore_i + γ × BatteryScore_i
```

Where:
- **SensorScore** = weighted sum of individual sensor health values, normalized to [0, 1]
- **ProximityScore** = exponential decay based on distance to swarm centroid
- **BatteryScore** = current battery level normalized to [0, 1]
- **α = 0.55, β = 0.25, γ = 0.20** (configurable per mission profile)

### Sensor Weights (Configurable)

| Sensor | Weight | Rationale |
|--------|:---:|-----------|
| GPS | 0.30 | Most critical for outdoor navigation |
| Encoder | 0.25 | Essential for odometry and movement accuracy |
| IMU | 0.20 | Important for orientation and stability |
| Ultrasonic | 0.15 | Obstacle avoidance |
| IR | 0.10 | Supplementary proximity sensing |

### Hysteresis (Anti-Flapping)

A new leader is only elected if its score exceeds the current leader by a margin **τ** (default 0.08):

```
Switch to new leader only if: Score_new - Score_current > τ
```

This prevents rapid oscillation between leaders when scores are close.

### Consensus Protocol

Before a leadership change takes effect, the swarm runs a lightweight voting round:
1. Each alive robot independently computes all scores
2. Each robot votes for the highest-scoring candidate
3. The switch only happens if **≥ 50%** of the swarm agrees

### Fault Detection & Re-Election

- Each robot broadcasts a heartbeat every step
- If a robot misses **5 consecutive heartbeats**, it is declared dead
- If the current leader dies, an immediate emergency election is triggered

## Project Structure

```
├── simulation/
│   └── simulation.slx         # MATLAB/Simulink model (improved algorithm)
│
├── python/                    # Python simulation package
│   ├── main.py                # CLI entry point
│   ├── swarm_config.py        # All configurable parameters
│   ├── robot.py               # Robot agent (sensors, battery, dynamics)
│   ├── leader_election.py     # Election engine (scoring, hysteresis, consensus)
│   ├── swarm.py               # Swarm orchestrator
│   ├── simulation.py          # Time-stepped simulation driver
│   ├── visualizer.py          # Matplotlib dashboard & charts
│   └── requirements.txt       # Python dependencies
│
├── src/                       # ESP32 firmware (PlatformIO)
│   ├── main.cpp               # Main firmware (RF24 radio + sensors)
│   ├── MotorControl.cpp       # Motor control driver
│   └── MPU6050MadgwickAHRS.cpp # IMU sensor fusion
│
├── include/                   # C++ header files
│   ├── MotorControl.h
│   └── MPU6050MadgwickAHRS.h
│
└── platformio.ini             # PlatformIO project config (ESP32)
```

## Quick Start — Python Simulation

### Prerequisites

- Python 3.9+
- pip

### Install

```bash
cd python
pip install -r requirements.txt
```

### Run Scenarios

```bash
# Normal operation (5 robots, 300 steps, default fault schedule)
python main.py

# Aggressive fault injection
python main.py --scenario fault

# No faults — steady-state behavior
python main.py --scenario stable

# OLD vs NEW algorithm comparison
python main.py --scenario compare

# Custom configuration
python main.py --robots 8 --steps 500 --seed 123

# Save dashboard without displaying
python main.py --no-show --save results.png
```

### Output

The simulation generates:
- **Console output**: Real-time leader status, score updates, fault events
- **Dashboard image** (`dashboard.png`): 7-panel visualization including:
  - Swarm positions with trails
  - Leader scores over time
  - Leader assignment timeline
  - Sensor health heatmap
  - Battery levels
  - Alive robot count
  - Transition log

## MATLAB/Simulink

Open `simulation/simulation.slx` in MATLAB R2023a+ with Simulink. The model implements the improved weighted scoring algorithm with:
- Configurable sensor weights (W_IMU, W_Ultrasonic, W_IR, W_GPS, W_Encoder)
- Battery level inputs per robot
- Proximity-to-centroid inputs
- Hysteresis-based leader switching
- Display blocks for leader ID and score

## Hardware (ESP32)

The firmware runs on **ESP32-DOIT-DevKit-V1** with:
- **MPU6050** IMU (Madgwick AHRS filter)
- **HC-SR04** ultrasonic distance sensor
- **RF24** radio for inter-robot communication
- **PCA9685** PWM servo driver for motor control
- Quadrature encoder feedback

Build with PlatformIO:
```bash
pio run -t upload
```

## Results

### Algorithm Comparison

Running both algorithms on the same 5-robot scenario with identical fault injection:

| Metric | Old Algorithm | Improved Algorithm |
|--------|:---:|:---:|
| Leader transitions | 21 | 1 |
| Flapping reduction | — | **100%** |
| Fault recovery | No | Yes (automatic) |
| Score stability | Low | High |

## License

This project is part of academic research. Please cite appropriately if used.
