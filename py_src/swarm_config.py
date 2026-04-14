"""
Swarm Configuration — All tunable parameters for the simulation.

This module centralises every knob so experiments can be reproduced by
changing a single file (or overriding from the CLI).
"""

from dataclasses import dataclass, field
from typing import Dict
import numpy as np


@dataclass
class SensorWeights:
    """
    Mission-profile-aware weights for each sensor modality.
    Higher weight = more important for leader eligibility.
    All weights are normalised internally so they sum to 1.
    """
    imu: float = 0.20
    ultrasonic: float = 0.15
    ir: float = 0.10
    gps: float = 0.30
    encoder: float = 0.25

    def as_array(self) -> np.ndarray:
        """Return weights as a NumPy array in canonical sensor order."""
        w = np.array([self.imu, self.ultrasonic, self.ir, self.gps, self.encoder])
        return w / w.sum()  # normalise

    @property
    def names(self):
        return ["IMU", "Ultrasonic", "IR", "GPS", "Encoder"]


@dataclass
class ElectionConfig:
    """Parameters governing the leader election algorithm."""
    # Score composition weights  (α sensor, β proximity, γ battery)
    alpha: float = 0.55          # sensor health weight
    beta: float = 0.25           # proximity-to-centroid weight
    gamma: float = 0.20          # battery level weight

    # Hysteresis — minimum margin to unseat the current leader
    hysteresis_threshold: float = 0.08

    # Consensus — fraction of swarm that must agree on a new leader
    consensus_quorum: float = 0.5

    # Fault detection
    heartbeat_timeout_steps: int = 5   # steps of silence before declaring a robot dead
    min_sensor_health: float = 0.15    # below this the sensor is considered failed


@dataclass
class SimulationConfig:
    """Top-level simulation parameters."""
    num_robots: int = 5
    total_steps: int = 300
    dt: float = 0.1              # seconds per step
    arena_size: float = 20.0     # metres — square arena side length
    random_seed: int = 42

    # Robot physical parameters
    max_speed: float = 1.5       # m/s
    battery_full: float = 100.0
    battery_drain_rate: float = 0.08   # per step (nominal)
    battery_drain_leader_penalty: float = 0.04  # extra drain for leader (more computation)

    # Sensor noise parameters  (σ for Gaussian noise on health)
    sensor_noise_std: float = 0.03

    # Fault injection schedule  {step: (robot_id, fault_type)}
    #   fault_type: 'sensor_degrade', 'battery_drain', 'crash', 'sensor_fail'
    fault_schedule: Dict = field(default_factory=lambda: {
        100: (0, 'sensor_degrade'),    # leader's GPS degrades at t=100
        150: (0, 'battery_drain'),     # same robot gets battery hit at t=150
        200: (2, 'crash'),             # robot 2 crashes at t=200
    })

    sensor_weights: SensorWeights = field(default_factory=SensorWeights)
    election: ElectionConfig = field(default_factory=ElectionConfig)
