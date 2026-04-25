"""
main.py — Entry point for the Robotic Swarm Dynamic Leader Selection simulation.

Usage:
    python main.py                     # Normal 5-robot simulation
    python main.py --scenario fault    # Leader failure scenario
    python main.py --scenario compare  # Old vs new algorithm comparison
    python main.py --robots 8          # Custom robot count
    python main.py --steps 500         # Custom step count
    python main.py --no-show           # Save dashboard to file only
"""

from __future__ import annotations
import argparse
import sys
import os

# Ensure we can import from the same directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from swarm_config import SimulationConfig
from simulation import Simulation, OldAlgorithmSimulation
from visualizer import Visualizer


def parse_args():
    parser = argparse.ArgumentParser(
        description="Robotic Swarm Dynamic Leader Selection Simulation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Scenarios:
  normal   — Standard operation with default fault schedule
  fault    — Aggressive fault injection (sensor fail + crash + battery)
  stable   — No faults, test steady-state behaviour
  compare  — Side-by-side old vs improved algorithm comparison
        """
    )
    parser.add_argument("--scenario", type=str, default="normal",
                        choices=["normal", "fault", "stable", "compare"],
                        help="Simulation scenario to run")
    parser.add_argument("--robots", type=int, default=5,
                        help="Number of robots (default: 5)")
    parser.add_argument("--steps", type=int, default=300,
                        help="Simulation steps (default: 300)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducibility")
    parser.add_argument("--no-show", action="store_true",
                        help="Don't display plots, just save to file")
    parser.add_argument("--save", type=str, default=None,
                        help="Path to save dashboard image")
    return parser.parse_args()


def build_config(args) -> SimulationConfig:
    """Create SimulationConfig based on CLI arguments and scenario."""
    cfg = SimulationConfig(
        num_robots=args.robots,
        total_steps=args.steps,
        random_seed=args.seed,
    )

    if args.scenario == "normal":
        # Default fault schedule (already set in config)
        pass

    elif args.scenario == "fault":
        # Aggressive: multiple faults on multiple robots
        cfg.fault_schedule = {
            50: (0, 'sensor_degrade'),
            80: (1, 'sensor_fail'),
            120: (0, 'battery_drain'),
            160: (2, 'crash'),
            200: (3, 'sensor_degrade'),
            240: (1, 'battery_drain'),
        }

    elif args.scenario == "stable":
        # No faults at all
        cfg.fault_schedule = {}

    elif args.scenario == "compare":
        # Use a moderate fault schedule for comparison
        cfg.fault_schedule = {
            100: (0, 'sensor_degrade'),
            200: (2, 'crash'),
        }

    return cfg


def run_normal(args, cfg: SimulationConfig):
    """Run the improved algorithm simulation."""
    sim = Simulation(cfg)
    swarm = sim.run(verbose=True)

    default_save_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    os.makedirs(default_save_dir, exist_ok=True)
    save_path = args.save or os.path.join(default_save_dir, "dashboard.png")
    viz = Visualizer(swarm, cfg)
    viz.dashboard(save_path=save_path, show=not args.no_show)


def run_comparison(args, cfg: SimulationConfig):
    """Run both old and new algorithms and compare."""
    print("=" * 60)
    print("  COMPARISON: Old Algorithm vs Improved Algorithm")
    print("=" * 60)

    # Run improved algorithm
    print("\n> Running IMPROVED algorithm...")
    sim_new = Simulation(cfg)
    swarm_new = sim_new.run(verbose=True)

    # Run old algorithm (same config, same seed)
    print("\n> Running OLD algorithm (baseline)...")
    cfg_old = SimulationConfig(
        num_robots=cfg.num_robots,
        total_steps=cfg.total_steps,
        random_seed=cfg.random_seed,
        fault_schedule=cfg.fault_schedule,
    )
    sim_old = OldAlgorithmSimulation(cfg_old)
    swarm_old = sim_old.run(verbose=True)

    # Count transitions
    def count_transitions(history):
        return sum(1 for i in range(1, len(history))
                   if history[i] != history[i - 1])

    new_t = count_transitions(swarm_new.leader_id_history)
    old_t = count_transitions(swarm_old.leader_id_history)

    print(f"\n{'-' * 50}")
    print(f"  Old algorithm transitions:      {old_t}")
    print(f"  Improved algorithm transitions:  {new_t}")
    reduction = ((old_t - new_t) / max(old_t, 1)) * 100
    print(f"  Reduction in flapping:           {reduction:.1f}%")
    print(f"{'-' * 50}")

    # Dashboard for new
    default_save_dir = os.path.join(os.path.dirname(__file__), "..", "images")
    os.makedirs(default_save_dir, exist_ok=True)
    save_path = args.save or os.path.join(default_save_dir, "dashboard_improved.png")
    viz = Visualizer(swarm_new, cfg)
    viz.dashboard(save_path=save_path, show=False)

    # Comparison chart
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 5))
    fig.patch.set_facecolor("#1A1A2E")
    Visualizer.plot_flapping_comparison(swarm_new, swarm_old, ax)
    comp_path = os.path.join(default_save_dir, "comparison.png")
    fig.savefig(comp_path, dpi=150, facecolor="#1A1A2E")
    print(f"  Comparison chart saved to {comp_path}")

    if not args.no_show:
        plt.show()


def main():
    args = parse_args()
    cfg = build_config(args)

    if args.scenario == "compare":
        run_comparison(args, cfg)
    else:
        run_normal(args, cfg)


if __name__ == "__main__":
    main()
