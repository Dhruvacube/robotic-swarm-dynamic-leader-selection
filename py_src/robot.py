"""
Robot — Models a single robot in the swarm.

Each robot maintains:
  • 2-D position & velocity
  • Per-sensor health values ∈ [0, 1]
  • Battery level ∈ [0, 100]
  • Heartbeat counter (for fault detection)
  • Computed leader score
"""

from __future__ import annotations
import numpy as np
from swarm_config import SimulationConfig, SensorWeights


class Robot:
    """Represents a single robot agent in the swarm."""

    SENSOR_NAMES = ["IMU", "Ultrasonic", "IR", "GPS", "Encoder"]

    def __init__(self, robot_id: int, config: SimulationConfig, rng: np.random.Generator):
        self.id = robot_id
        self.cfg = config
        self.rng = rng

        # --- Physical state ---
        self.position = rng.uniform(0, config.arena_size, size=2)
        self.velocity = rng.uniform(-0.5, 0.5, size=2)

        # --- Sensor health  [0 = dead, 1 = perfect] ---
        # Initialise near-perfect with slight per-robot variation
        self.sensor_health = np.clip(
            rng.normal(0.90, 0.05, size=5), 0.0, 1.0
        )

        # --- Battery ---
        self.battery = config.battery_full

        # --- Role ---
        self.is_leader = False
        self.leader_score = 0.0
        self.is_alive = True

        # --- Heartbeat (steps since last broadcast) ---
        self.heartbeat_counter = 0

        # --- History (for plotting) ---
        self.history = {
            "position": [],
            "sensor_health": [],
            "battery": [],
            "leader_score": [],
            "is_leader": [],
        }

    # ------------------------------------------------------------------
    # Dynamics
    # ------------------------------------------------------------------
    def step(self, dt: float):
        """Advance the robot's physical state by one time step."""
        if not self.is_alive:
            return

        # Random walk with momentum
        accel = self.rng.normal(0, 0.3, size=2)
        self.velocity = 0.95 * self.velocity + accel * dt
        speed = np.linalg.norm(self.velocity)
        if speed > self.cfg.max_speed:
            self.velocity *= self.cfg.max_speed / speed

        self.position += self.velocity * dt

        # Bounce off arena walls
        for dim in range(2):
            if self.position[dim] < 0:
                self.position[dim] = -self.position[dim]
                self.velocity[dim] *= -1
            elif self.position[dim] > self.cfg.arena_size:
                self.position[dim] = 2 * self.cfg.arena_size - self.position[dim]
                self.velocity[dim] *= -1

        # Sensor health: slow random drift (simulate real-world noise)
        drift = self.rng.normal(0, self.cfg.sensor_noise_std, size=5)
        self.sensor_health = np.clip(self.sensor_health + drift, 0.0, 1.0)

        # Battery drain
        drain = self.cfg.battery_drain_rate
        if self.is_leader:
            drain += self.cfg.battery_drain_leader_penalty
        self.battery = max(0.0, self.battery - drain)

        # If battery is dead, the robot is dead
        if self.battery <= 0:
            self.is_alive = False

        self.heartbeat_counter += 1

    def broadcast_heartbeat(self):
        """Reset heartbeat counter (called each step the robot is alive)."""
        if self.is_alive:
            self.heartbeat_counter = 0

    # ------------------------------------------------------------------
    # Scoring
    # ------------------------------------------------------------------
    def compute_sensor_score(self, weights: SensorWeights) -> float:
        """
        Weighted sensor health score ∈ [0, 1].
        
        Score = Σ(w_i × h_i)  where w is normalised weight and h is health.
        """
        w = weights.as_array()
        return float(np.dot(w, self.sensor_health))

    def compute_proximity_score(self, centroid: np.ndarray) -> float:
        """
        Proximity score ∈ [0, 1].
        Closer to the swarm centroid → higher score.
        Uses an exponential decay: exp(−dist / arena_half).
        """
        dist = np.linalg.norm(self.position - centroid)
        half = self.cfg.arena_size / 2.0
        return float(np.exp(-dist / half))

    def compute_battery_score(self) -> float:
        """Battery score ∈ [0, 1]."""
        return self.battery / self.cfg.battery_full

    def compute_leader_score(self, weights: SensorWeights, centroid: np.ndarray,
                             alpha: float, beta: float, gamma: float) -> float:
        """
        Composite leader eligibility score:
            L = α × SensorScore + β × ProximityScore + γ × BatteryScore
        """
        s = self.compute_sensor_score(weights)
        p = self.compute_proximity_score(centroid)
        b = self.compute_battery_score()
        self.leader_score = alpha * s + beta * p + gamma * b
        return self.leader_score

    # ------------------------------------------------------------------
    # Fault injection
    # ------------------------------------------------------------------
    def inject_sensor_degrade(self, sensor_idx: int | None = None, severity: float = 0.5):
        """
        Degrade a sensor's health.
        If sensor_idx is None, degrade the highest-weighted sensor.
        """
        if sensor_idx is None:
            sensor_idx = int(np.argmax(self.sensor_health))
        self.sensor_health[sensor_idx] *= (1 - severity)

    def inject_battery_drain(self, amount: float = 30.0):
        """Instant battery drain."""
        self.battery = max(0.0, self.battery - amount)

    def inject_crash(self):
        """Robot goes offline."""
        self.is_alive = False
        self.sensor_health[:] = 0.0
        self.battery = 0.0

    def inject_sensor_fail(self, sensor_idx: int | None = None):
        """Complete failure of a specific sensor."""
        if sensor_idx is None:
            sensor_idx = int(np.argmax(self.sensor_health))
        self.sensor_health[sensor_idx] = 0.0

    # ------------------------------------------------------------------
    # History
    # ------------------------------------------------------------------
    def record(self):
        """Snapshot current state into history."""
        self.history["position"].append(self.position.copy())
        self.history["sensor_health"].append(self.sensor_health.copy())
        self.history["battery"].append(self.battery)
        self.history["leader_score"].append(self.leader_score)
        self.history["is_leader"].append(self.is_leader)

    # ------------------------------------------------------------------
    # Repr
    # ------------------------------------------------------------------
    def __repr__(self):
        role = "LEADER" if self.is_leader else "follower"
        alive = "alive" if self.is_alive else "DEAD"
        return (f"Robot(id={self.id}, {role}, {alive}, "
                f"score={self.leader_score:.3f}, batt={self.battery:.1f}%)")
