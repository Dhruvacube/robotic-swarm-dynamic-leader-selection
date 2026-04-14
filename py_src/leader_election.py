"""
Leader Election Engine — Implements the improved dynamic leader selection algorithm.

Algorithm improvements over the original Simulink model:
  1. Weighted multi-criteria scoring  (sensor health × mission weights + proximity + battery)
  2. Hysteresis (anti-flapping)       — leader only changes if margin > τ
  3. Distributed consensus            — majority vote confirms the switch
  4. Fault detection & re-election    — heartbeat timeout triggers emergency election
  5. Tie-breaking                     — lowest robot ID wins ties
  6. Scalable to N robots
"""

from __future__ import annotations
from typing import List, Optional, Tuple
import numpy as np

from robot import Robot
from swarm_config import SimulationConfig


class LeaderElectionEngine:
    """
    Runs one election round per simulation step.

    Lifecycle:
        1. compute_scores()        — each robot's composite score
        2. detect_faults()         — check heartbeats, remove dead robots
        3. elect_leader()          — weighted max + hysteresis + consensus
    """

    def __init__(self, config: SimulationConfig):
        self.cfg = config
        self.ec = config.election
        self.current_leader_id: Optional[int] = None
        self.election_history: List[dict] = []
        self.transition_log: List[Tuple[int, int, int, float, float]] = []
        # (step, old_leader, new_leader, old_score, new_score)

    # ------------------------------------------------------------------
    # 1. Score computation
    # ------------------------------------------------------------------
    def compute_scores(self, robots: List[Robot]) -> np.ndarray:
        """
        Compute the composite leader score for every alive robot.
        Returns an array of scores indexed by robot ID.
        Dead robots get score = -∞.
        """
        alive = [r for r in robots if r.is_alive]
        if not alive:
            return np.full(len(robots), -np.inf)

        # Swarm centroid (only from alive robots)
        positions = np.array([r.position for r in alive])
        centroid = positions.mean(axis=0)

        scores = np.full(len(robots), -np.inf)
        for r in alive:
            scores[r.id] = r.compute_leader_score(
                weights=self.cfg.sensor_weights,
                centroid=centroid,
                alpha=self.ec.alpha,
                beta=self.ec.beta,
                gamma=self.ec.gamma,
            )
        return scores

    # ------------------------------------------------------------------
    # 2. Fault detection
    # ------------------------------------------------------------------
    def detect_faults(self, robots: List[Robot]) -> List[int]:
        """
        Return a list of robot IDs that have missed their heartbeat
        beyond the configured timeout.
        """
        faulted = []
        for r in robots:
            if not r.is_alive:
                continue
            if r.heartbeat_counter > self.ec.heartbeat_timeout_steps:
                faulted.append(r.id)
                r.inject_crash()  # consider it dead
        return faulted

    # ------------------------------------------------------------------
    # 3. Consensus vote
    # ------------------------------------------------------------------
    def _consensus_vote(self, robots: List[Robot], proposed_leader_id: int) -> bool:
        """
        Simple majority vote: each alive robot independently checks
        whether the proposed leader has the highest score from their
        own perspective.  Returns True if quorum is met.
        """
        alive = [r for r in robots if r.is_alive]
        if not alive:
            return False

        votes_for = 0
        positions = np.array([r.position for r in alive])
        centroid = positions.mean(axis=0)

        # Each robot computes all scores from its own "view"
        # (In a real system, each robot would only have noisy local info;
        #  here we add a small perception noise to simulate that.)
        for voter in alive:
            # Voter computes scores with slight noise
            best_id = proposed_leader_id
            best_score = -np.inf
            for r in alive:
                s = r.compute_leader_score(
                    self.cfg.sensor_weights, centroid,
                    self.ec.alpha, self.ec.beta, self.ec.gamma
                )
                # Add voter perception noise
                s += np.random.normal(0, 0.01)
                if s > best_score or (s == best_score and r.id < best_id):
                    best_score = s
                    best_id = r.id
            if best_id == proposed_leader_id:
                votes_for += 1

        fraction = votes_for / len(alive)
        return fraction >= self.ec.consensus_quorum

    # ------------------------------------------------------------------
    # 4. Main election
    # ------------------------------------------------------------------
    def elect_leader(self, robots: List[Robot], step: int) -> int:
        """
        Run full election round.  Returns the ID of the elected leader.

        Steps:
          a) Compute scores
          b) If no current leader (first step or leader died) → pick max score
          c) If current leader alive → apply hysteresis check
          d) If challenger wins → run consensus vote
          e) Update roles
        """
        scores = self.compute_scores(robots)
        alive_ids = [r.id for r in robots if r.is_alive]

        if not alive_ids:
            return -1  # entire swarm is dead

        # Candidate: robot with highest score (tie-break: lowest ID)
        candidate_id = min(alive_ids, key=lambda i: (-scores[i], i))
        candidate_score = scores[candidate_id]

        old_leader_id = self.current_leader_id

        # ---- First election or leader is dead -------------------------
        if (self.current_leader_id is None or
                not robots[self.current_leader_id].is_alive):
            self._set_leader(robots, candidate_id, step, old_leader_id,
                             scores[old_leader_id] if old_leader_id is not None and old_leader_id < len(scores) else 0.0,
                             candidate_score, reason="initial_or_failover")
            return candidate_id

        # ---- Hysteresis check -----------------------------------------
        current_score = scores[self.current_leader_id]
        margin = candidate_score - current_score

        if candidate_id == self.current_leader_id:
            # Current leader is still the best — no change
            self._record(step, self.current_leader_id, scores)
            return self.current_leader_id

        if margin <= self.ec.hysteresis_threshold:
            # Challenger doesn't beat the threshold — keep current leader
            self._record(step, self.current_leader_id, scores)
            return self.current_leader_id

        # ---- Consensus vote -------------------------------------------
        if self._consensus_vote(robots, candidate_id):
            self._set_leader(robots, candidate_id, step, old_leader_id,
                             current_score, candidate_score, reason="consensus")
            return candidate_id
        else:
            # Consensus failed — keep current leader
            self._record(step, self.current_leader_id, scores)
            return self.current_leader_id

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _set_leader(self, robots: List[Robot], new_id: int, step: int,
                    old_id: Optional[int], old_score: float, new_score: float,
                    reason: str = ""):
        """Update robot roles and log the transition."""
        # Clear old leader
        for r in robots:
            r.is_leader = False
        robots[new_id].is_leader = True
        self.current_leader_id = new_id

        self.transition_log.append((step, old_id if old_id is not None else -1,
                                    new_id, old_score, new_score))
        scores_arr = np.array([r.leader_score for r in robots])
        self._record(step, new_id, scores_arr)

    def _record(self, step: int, leader_id: int, scores: np.ndarray):
        """Record election state for later analysis."""
        self.election_history.append({
            "step": step,
            "leader_id": leader_id,
            "scores": scores.copy(),
        })
