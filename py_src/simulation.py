"""
Simulation — Main time-stepped simulation driver.

Runs the swarm for a configurable number of steps, injecting faults
per the schedule, and collecting data for post-run analysis.
"""

from __future__ import annotations
import time
from swarm import Swarm
from swarm_config import SimulationConfig


class Simulation:
    """Drives the swarm simulation and collects results."""

    def __init__(self, config: SimulationConfig | None = None):
        self.cfg = config or SimulationConfig()
        self.swarm = Swarm(self.cfg)
        self.wall_time = 0.0

    def run(self, verbose: bool = True) -> Swarm:
        """
        Execute the full simulation.
        
        Returns the Swarm object with complete history for analysis.
        """
        if verbose:
            print("=" * 58)
            print(f"  SWARM SIMULATION -- {self.cfg.num_robots} robots, "
                  f"{self.cfg.total_steps} steps")
            print(f"  Algorithm: Weighted + Hysteresis + Consensus")
            print("=" * 58)
            print()

        t0 = time.perf_counter()

        for step in range(self.cfg.total_steps):
            self.swarm.step(step)

            if verbose and step % 50 == 0:
                leader = self.swarm.leader
                leader_str = f"Robot {leader.id}" if leader else "NONE"
                alive = len(self.swarm.alive_robots)
                print(f"  Step {step:>4d}  |  Leader: {leader_str:>8s}  |  "
                      f"Alive: {alive}/{self.cfg.num_robots}  |  "
                      f"Scores: {self._score_str()}")

        self.wall_time = time.perf_counter() - t0

        if verbose:
            print()
            print(self.swarm.summary())
            print(f"\nSimulation completed in {self.wall_time:.2f}s")

        return self.swarm

    def _score_str(self) -> str:
        """Format current scores as a compact string."""
        parts = []
        for r in self.swarm.robots:
            tag = "*" if r.is_leader else " "
            if r.is_alive:
                parts.append(f"R{r.id}{tag}{r.leader_score:.2f}")
            else:
                parts.append(f"R{r.id}X")
        return "  ".join(parts)


# ------------------------------------------------------------------
# Comparison: Old vs New algorithm
# ------------------------------------------------------------------
class OldAlgorithmSimulation:
    """
    Baseline: reproduces the original Simulink algorithm.
    Simple sum of all sensor values, no weights, no hysteresis, no consensus.
    Used for comparison against the improved algorithm.
    """

    def __init__(self, config: SimulationConfig | None = None):
        self.cfg = config or SimulationConfig()
        self.swarm = Swarm(self.cfg)
        # Override election to use simple max (no hysteresis, no consensus)
        self.swarm.election.ec.hysteresis_threshold = 0.0
        self.swarm.election.ec.consensus_quorum = 0.0  # always passes
        # Set all sensor weights equal
        self.cfg.sensor_weights.imu = 1.0
        self.cfg.sensor_weights.ultrasonic = 1.0
        self.cfg.sensor_weights.ir = 1.0
        self.cfg.sensor_weights.gps = 1.0
        self.cfg.sensor_weights.encoder = 1.0
        # Only sensor score matters
        self.swarm.election.ec.alpha = 1.0
        self.swarm.election.ec.beta = 0.0
        self.swarm.election.ec.gamma = 0.0

    def run(self, verbose: bool = False) -> Swarm:
        if verbose:
            print(f"Running BASELINE (old algorithm) — {self.cfg.num_robots} robots, "
                  f"{self.cfg.total_steps} steps")
        for step in range(self.cfg.total_steps):
            self.swarm.step(step)
        return self.swarm
