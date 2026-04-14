"""
Swarm — Orchestrates N robots and the leader election engine.

This is the high-level container that owns the robots, drives their
physics, triggers elections, and exposes the full simulation state.
"""

from __future__ import annotations
from typing import List, Optional
import numpy as np

from robot import Robot
from leader_election import LeaderElectionEngine
from swarm_config import SimulationConfig


class Swarm:
    """Manages the collective of robots."""

    def __init__(self, config: SimulationConfig):
        self.cfg = config
        self.rng = np.random.default_rng(config.random_seed)

        # Create robots
        self.robots: List[Robot] = [
            Robot(i, config, self.rng) for i in range(config.num_robots)
        ]

        # Election engine
        self.election = LeaderElectionEngine(config)

        # Swarm-level history
        self.centroid_history: List[np.ndarray] = []
        self.alive_count_history: List[int] = []
        self.leader_id_history: List[int] = []

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def alive_robots(self) -> List[Robot]:
        return [r for r in self.robots if r.is_alive]

    @property
    def centroid(self) -> np.ndarray:
        alive = self.alive_robots
        if not alive:
            return np.array([self.cfg.arena_size / 2, self.cfg.arena_size / 2])
        return np.mean([r.position for r in alive], axis=0)

    @property
    def leader(self) -> Optional[Robot]:
        for r in self.robots:
            if r.is_leader and r.is_alive:
                return r
        return None

    # ------------------------------------------------------------------
    # Step
    # ------------------------------------------------------------------
    def step(self, step_num: int):
        """
        Advance the swarm by one simulation tick:
          1. Apply any scheduled faults
          2. Move robots (physics step)
          3. Broadcast heartbeats
          4. Detect faults
          5. Run leader election
          6. Record history
        """
        # 1. Fault injection
        self._apply_faults(step_num)

        # 2. Physics
        for r in self.robots:
            r.step(self.cfg.dt)

        # 3. Heartbeats
        for r in self.robots:
            r.broadcast_heartbeat()

        # 4. Fault detection
        faulted = self.election.detect_faults(self.robots)
        if faulted:
            print(f"  [Step {step_num}] Faulted robots: {faulted}")

        # 5. Election
        leader_id = self.election.elect_leader(self.robots, step_num)

        # 6. Record
        self.centroid_history.append(self.centroid.copy())
        self.alive_count_history.append(len(self.alive_robots))
        self.leader_id_history.append(leader_id)
        for r in self.robots:
            r.record()

    # ------------------------------------------------------------------
    # Fault injection
    # ------------------------------------------------------------------
    def _apply_faults(self, step_num: int):
        """Check the fault schedule and inject if due."""
        if step_num not in self.cfg.fault_schedule:
            return

        robot_id, fault_type = self.cfg.fault_schedule[step_num]
        if robot_id >= len(self.robots):
            return

        robot = self.robots[robot_id]
        print(f"  [Step {step_num}] Injecting fault '{fault_type}' on Robot {robot_id}")

        if fault_type == "sensor_degrade":
            robot.inject_sensor_degrade(sensor_idx=3, severity=0.6)  # GPS degrade
        elif fault_type == "battery_drain":
            robot.inject_battery_drain(amount=35.0)
        elif fault_type == "crash":
            robot.inject_crash()
        elif fault_type == "sensor_fail":
            robot.inject_sensor_fail(sensor_idx=3)  # GPS total fail
        else:
            print(f"  [Step {step_num}] Unknown fault type: {fault_type}")

    # ------------------------------------------------------------------
    # Add / Remove robots (scalability)
    # ------------------------------------------------------------------
    def add_robot(self) -> Robot:
        """Dynamically add a new robot to the swarm."""
        new_id = len(self.robots)
        r = Robot(new_id, self.cfg, self.rng)
        self.robots.append(r)
        print(f"  Robot {new_id} joined the swarm.")
        return r

    def remove_robot(self, robot_id: int):
        """Gracefully remove a robot (e.g. recalled for maintenance)."""
        if 0 <= robot_id < len(self.robots):
            self.robots[robot_id].inject_crash()
            print(f"  Robot {robot_id} removed from swarm.")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    def summary(self) -> str:
        lines = ["=" * 60, "SWARM SUMMARY", "=" * 60]
        for r in self.robots:
            lines.append(str(r))
        lines.append(f"\nLeader transitions: {len(self.election.transition_log)}")
        for t in self.election.transition_log:
            step, old, new, os, ns = t
            lines.append(f"  Step {step}: Robot {old} (score={os:.3f}) -> "
                         f"Robot {new} (score={ns:.3f})")
        lines.append("=" * 60)
        return "\n".join(lines)
