"""
Visualizer — Rich Matplotlib-based plots for swarm simulation analysis.

Generates:
  1. Swarm positions (animated-style snapshot or final frame)
  2. Leader score timeline
  3. Leader transition timeline
  4. Sensor health heatmap
  5. Battery level chart
  6. Comparison: old vs new algorithm (flapping frequency)
  7. Combined dashboard figure
"""

from __future__ import annotations
from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from matplotlib.colors import LinearSegmentedColormap
import matplotlib.ticker as mticker

from swarm import Swarm
from swarm_config import SimulationConfig


# ── Colour palette ─────────────────────────────────────────────────
ROBOT_COLORS = [
    "#4FC3F7",  # light blue
    "#81C784",  # green
    "#FFB74D",  # orange
    "#E57373",  # red
    "#BA68C8",  # purple
    "#4DB6AC",  # teal
    "#FF8A65",  # deep orange
    "#A1887F",  # brown
]
LEADER_COLOR = "#FFD600"
DEAD_COLOR = "#616161"
BG_COLOR = "#1A1A2E"
PANEL_COLOR = "#16213E"
TEXT_COLOR = "#E0E0E0"
GRID_COLOR = "#2C2C54"
ACCENT = "#E94560"


def _style_axes(ax, title=""):
    """Apply the dark theme to an axes."""
    ax.set_facecolor(PANEL_COLOR)
    ax.set_title(title, color=TEXT_COLOR, fontsize=11, fontweight="bold", pad=8)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(GRID_COLOR)
    ax.grid(True, color=GRID_COLOR, alpha=0.3, linewidth=0.5)


class Visualizer:
    """Generates all plots from a completed Swarm simulation."""

    def __init__(self, swarm: Swarm, config: SimulationConfig):
        self.swarm = swarm
        self.cfg = config
        self.n_robots = config.num_robots
        self.steps = config.total_steps

        plt.rcParams.update({
            "figure.facecolor": BG_COLOR,
            "axes.facecolor": PANEL_COLOR,
            "text.color": TEXT_COLOR,
            "axes.labelcolor": TEXT_COLOR,
            "xtick.color": TEXT_COLOR,
            "ytick.color": TEXT_COLOR,
            "font.family": "sans-serif",
            "font.size": 9,
        })

    # ------------------------------------------------------------------
    # 1. Swarm positions (final frame)
    # ------------------------------------------------------------------
    def plot_positions(self, ax: Optional[plt.Axes] = None, step: int = -1):
        if ax is None:
            fig, ax = plt.subplots(figsize=(6, 6))
        _style_axes(ax, f"Swarm Positions (Step {step if step >= 0 else self.steps - 1})")

        # Arena boundary
        arena = plt.Rectangle((0, 0), self.cfg.arena_size, self.cfg.arena_size,
                               fill=False, edgecolor=ACCENT, linewidth=1.5, linestyle="--")
        ax.add_patch(arena)

        idx = step if step >= 0 else -1
        for r in self.swarm.robots:
            pos = r.history["position"][idx]
            is_leader = r.history["is_leader"][idx]
            alive = r.is_alive if idx == -1 else r.history["battery"][idx] > 0

            if not alive:
                color = DEAD_COLOR
                marker = "x"
                size = 80
            elif is_leader:
                color = LEADER_COLOR
                marker = "*"
                size = 250
            else:
                color = ROBOT_COLORS[r.id % len(ROBOT_COLORS)]
                marker = "o"
                size = 100

            ax.scatter(pos[0], pos[1], c=color, marker=marker,
                       s=size, zorder=5, edgecolors="white", linewidths=0.5)
            ax.annotate(f"R{r.id}", (pos[0], pos[1]), fontsize=7, color="white",
                        ha="center", va="bottom", xytext=(0, 8),
                        textcoords="offset points",
                        fontweight="bold" if is_leader else "normal")

            # Draw trail (last 30 positions)
            trail_len = min(30, len(r.history["position"]))
            if trail_len > 1 and alive:
                trail = np.array(r.history["position"][max(0, idx - trail_len):idx if idx >= 0 else None])
                if len(trail) > 1:
                    alphas = np.linspace(0.05, 0.4, len(trail))
                    for j in range(len(trail) - 1):
                        ax.plot(trail[j:j+2, 0], trail[j:j+2, 1],
                                color=color, alpha=float(alphas[j]), linewidth=1)

        # Centroid
        centroid = self.swarm.centroid_history[idx]
        ax.scatter(centroid[0], centroid[1], c=ACCENT, marker="+", s=150,
                   zorder=6, linewidths=2)
        ax.annotate("centroid", (centroid[0], centroid[1]), fontsize=6,
                    color=ACCENT, ha="center", va="top", xytext=(0, -10),
                    textcoords="offset points")

        ax.set_xlim(-1, self.cfg.arena_size + 1)
        ax.set_ylim(-1, self.cfg.arena_size + 1)
        ax.set_aspect("equal")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

    # ------------------------------------------------------------------
    # 2. Leader score timeline
    # ------------------------------------------------------------------
    def plot_scores(self, ax: Optional[plt.Axes] = None):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 4))
        _style_axes(ax, "Leader Scores Over Time")

        steps = np.arange(self.steps)
        for r in self.swarm.robots:
            scores = np.array(r.history["leader_score"])
            is_leader = np.array(r.history["is_leader"])
            color = ROBOT_COLORS[r.id % len(ROBOT_COLORS)]
            ax.plot(steps, scores, color=color, alpha=0.8, linewidth=1.2,
                    label=f"Robot {r.id}")

            # Mark leader periods with thicker line
            leader_steps = np.where(is_leader)[0]
            if len(leader_steps) > 0:
                ax.scatter(leader_steps, scores[leader_steps], color=LEADER_COLOR,
                           s=3, zorder=4, alpha=0.6)

        # Mark fault injection points
        for step, (rid, ftype) in self.cfg.fault_schedule.items():
            if step < self.steps:
                ax.axvline(x=step, color=ACCENT, linestyle=":", alpha=0.5, linewidth=1)
                ax.annotate(f"! {ftype}\n(R{rid})", (step, ax.get_ylim()[1]),
                            fontsize=6, color=ACCENT, ha="center", va="top",
                            rotation=0)

        ax.set_xlabel("Simulation Step")
        ax.set_ylabel("Leader Score")
        ax.legend(loc="lower left", fontsize=7, ncol=3, framealpha=0.3)

    # ------------------------------------------------------------------
    # 3. Leader transition timeline
    # ------------------------------------------------------------------
    def plot_leader_timeline(self, ax: Optional[plt.Axes] = None):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 2))
        _style_axes(ax, "Leader Assignment Over Time")

        leaders = np.array(self.swarm.leader_id_history)
        steps = np.arange(len(leaders))

        # Create colored spans for each leader
        for rid in range(self.n_robots):
            mask = leaders == rid
            color = ROBOT_COLORS[rid % len(ROBOT_COLORS)]
            ax.fill_between(steps, 0, 1, where=mask, color=color, alpha=0.7,
                            label=f"R{rid}", step="mid")

        # Mark transitions
        for (step, old, new, os, ns) in self.swarm.election.transition_log:
            if step < len(leaders):
                ax.axvline(x=step, color="white", linewidth=0.8, alpha=0.6, linestyle="--")

        ax.set_xlabel("Simulation Step")
        ax.set_yticks([])
        ax.set_ylim(0, 1)
        ax.legend(loc="upper right", fontsize=7, ncol=self.n_robots, framealpha=0.3)

    # ------------------------------------------------------------------
    # 4. Sensor health heatmap
    # ------------------------------------------------------------------
    def plot_sensor_heatmap(self, robot_id: int = 0, ax: Optional[plt.Axes] = None):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 3))

        r = self.swarm.robots[robot_id]
        health = np.array(r.history["sensor_health"])  # (steps, 5)
        _style_axes(ax, f"Sensor Health — Robot {robot_id}")

        cmap = LinearSegmentedColormap.from_list("health", ["#B71C1C", "#FDD835", "#43A047"])
        im = ax.imshow(health.T, aspect="auto", cmap=cmap, vmin=0, vmax=1,
                       interpolation="nearest")
        ax.set_yticks(range(5))
        ax.set_yticklabels(["IMU", "Ultra", "IR", "GPS", "Enc"], fontsize=8)
        ax.set_xlabel("Simulation Step")
        plt.colorbar(im, ax=ax, label="Health", shrink=0.8)

    # ------------------------------------------------------------------
    # 5. Battery levels
    # ------------------------------------------------------------------
    def plot_battery(self, ax: Optional[plt.Axes] = None):
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 3))
        _style_axes(ax, "Battery Levels Over Time")

        steps = np.arange(self.steps)
        for r in self.swarm.robots:
            batt = np.array(r.history["battery"])
            color = ROBOT_COLORS[r.id % len(ROBOT_COLORS)]
            ax.plot(steps, batt, color=color, linewidth=1.2, label=f"R{r.id}")

        ax.axhline(y=0, color=ACCENT, linestyle="--", alpha=0.5, linewidth=1)
        ax.set_xlabel("Simulation Step")
        ax.set_ylabel("Battery (%)")
        ax.set_ylim(-5, 105)
        ax.legend(loc="upper right", fontsize=7, ncol=3, framealpha=0.3)

    # ------------------------------------------------------------------
    # 6. Flapping comparison
    # ------------------------------------------------------------------
    @staticmethod
    def plot_flapping_comparison(swarm_new: Swarm, swarm_old: Swarm,
                                 ax: Optional[plt.Axes] = None):
        if ax is None:
            fig, ax = plt.subplots(figsize=(8, 4))
        _style_axes(ax, "Leader Stability: Old vs. Improved Algorithm")

        def count_transitions(leader_history):
            transitions = 0
            for i in range(1, len(leader_history)):
                if leader_history[i] != leader_history[i - 1]:
                    transitions += 1
            return transitions

        new_transitions = count_transitions(swarm_new.leader_id_history)
        old_transitions = count_transitions(swarm_old.leader_id_history)

        bars = ax.bar(["Old Algorithm\n(Simple Max)","Improved Algorithm\n(Weighted+Hysteresis+Consensus)"],
                      [old_transitions, new_transitions],
                      color=[ACCENT, "#4FC3F7"], width=0.5, edgecolor="white", linewidth=0.5)

        for bar, val in zip(bars, [old_transitions, new_transitions]):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                    str(val), ha="center", va="bottom", fontsize=12,
                    fontweight="bold", color=TEXT_COLOR)

        ax.set_ylabel("Number of Leader Transitions")
        ax.set_ylim(0, max(old_transitions, new_transitions) * 1.3 + 1)

    # ------------------------------------------------------------------
    # 7. Combined dashboard
    # ------------------------------------------------------------------
    def dashboard(self, save_path: Optional[str] = None, show: bool = True):
        """Generate the full analysis dashboard."""
        fig = plt.figure(figsize=(18, 14))
        fig.suptitle("Robotic Swarm — Dynamic Leader Selection Dashboard",
                     color=LEADER_COLOR, fontsize=16, fontweight="bold", y=0.98)

        gs = GridSpec(4, 3, figure=fig, hspace=0.45, wspace=0.35,
                      left=0.06, right=0.96, top=0.93, bottom=0.04)

        # Row 1: Positions + Scores
        ax_pos = fig.add_subplot(gs[0, 0])
        ax_scores = fig.add_subplot(gs[0, 1:])
        self.plot_positions(ax_pos)
        self.plot_scores(ax_scores)

        # Row 2: Leader timeline + Battery
        ax_timeline = fig.add_subplot(gs[1, :2])
        ax_batt = fig.add_subplot(gs[1, 2])
        self.plot_leader_timeline(ax_timeline)
        self.plot_battery(ax_batt)

        # Row 3: Sensor heatmaps for leader and a follower
        leader_id = self.swarm.leader_id_history[-1] if self.swarm.leader_id_history else 0
        follower_id = (leader_id + 1) % self.n_robots
        ax_heat1 = fig.add_subplot(gs[2, :2])
        ax_heat2 = fig.add_subplot(gs[2, 2])
        self.plot_sensor_heatmap(leader_id, ax_heat1)

        # Alive count
        _style_axes(ax_heat2, "Alive Robot Count")
        steps = np.arange(len(self.swarm.alive_count_history))
        ax_heat2.fill_between(steps, self.swarm.alive_count_history,
                              color="#4FC3F7", alpha=0.5)
        ax_heat2.plot(steps, self.swarm.alive_count_history, color="#4FC3F7", linewidth=1.5)
        ax_heat2.set_xlabel("Step")
        ax_heat2.set_ylabel("Alive")
        ax_heat2.set_ylim(0, self.n_robots + 1)
        ax_heat2.yaxis.set_major_locator(mticker.MaxNLocator(integer=True))

        # Row 4: Summary text + transition log
        ax_info = fig.add_subplot(gs[3, :])
        ax_info.axis("off")
        info_lines = [
            f"Total steps: {self.steps}   |   Robots: {self.n_robots}   |   "
            f"Sensor weights: α={self.cfg.election.alpha}  β={self.cfg.election.beta}  "
            f"γ={self.cfg.election.gamma}   |   "
            f"Hysteresis τ={self.cfg.election.hysteresis_threshold}   |   "
            f"Consensus quorum={self.cfg.election.consensus_quorum}",
            "",
            "LEADER TRANSITIONS:",
        ]
        for (step, old, new, os, ns) in self.swarm.election.transition_log:
            reason = "failover" if old == -1 else "elected"
            info_lines.append(
                f"    Step {step:>4d}:  R{old} -> R{new}  "
                f"(score {os:.3f} -> {ns:.3f})  [{reason}]"
            )
        if not self.swarm.election.transition_log:
            info_lines.append("    (no transitions)")

        ax_info.text(0.02, 0.95, "\n".join(info_lines), transform=ax_info.transAxes,
                     fontsize=8, verticalalignment="top", fontfamily="monospace",
                     color=TEXT_COLOR, bbox=dict(boxstyle="round,pad=0.5",
                                                 facecolor=PANEL_COLOR, edgecolor=GRID_COLOR))

        if save_path:
            fig.savefig(save_path, dpi=150, facecolor=BG_COLOR)
            print(f"  Dashboard saved to {save_path}")

        if show:
            plt.show()

        return fig
