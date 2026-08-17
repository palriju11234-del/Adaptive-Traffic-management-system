"""
SignalX -- Part 6: Baseline and Performance Metrics

Experimental evaluation comparing fixed-time vs adaptive traffic signal control.

Runs both controllers on the SAME traffic scenario (same network, routes,
simulation duration, and random seed) and produces a side-by-side comparison
of key performance metrics.

Usage:
    python evaluate.py            Run both experiments headless
    python evaluate.py --gui      Run both experiments with SUMO-GUI
"""

import os
import sys
import time
import traci

# Add project root to path
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from controller.traffic_state import TrafficStateCollector
from controller.signal_controller import FixedSignalController
from controller.adaptive_controller import AdaptiveController
from evaluation.metrics import MetricsCollector


# -- Configuration ---------------------------------------------------------

SUMO_CONFIG = os.path.join(PROJECT_ROOT, "sumo", "simulation.sumocfg")
RANDOM_SEED = 42  # Same seed for both runs ensures fair comparison


# -- SUMO detection (reuses logic from main.py) ----------------------------

def find_sumo():
    """Locate SUMO installation and return paths to sumo and sumo-gui binaries."""
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
        sumo_gui = os.path.join(sumo_home, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            return sumo_bin, sumo_gui

    common_paths = [
        r"C:\Program Files (x86)\Eclipse\Sumo",
        r"C:\Program Files\Eclipse\Sumo",
        r"C:\sumo",
    ]
    for path in common_paths:
        sumo_bin = os.path.join(path, "bin", "sumo.exe")
        sumo_gui = os.path.join(path, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            return sumo_bin, sumo_gui

    print("ERROR: SUMO installation not found.")
    print("Please set the SUMO_HOME environment variable.")
    sys.exit(1)


# -- Experiment runner ------------------------------------------------------

def run_experiment(mode, use_gui=False, seed=RANDOM_SEED):
    """
    Run a complete SUMO simulation with the specified controller mode.

    Args:
        mode:    "fixed" or "adaptive"
        use_gui: whether to open SUMO-GUI
        seed:    random seed for SUMO (ensures reproducibility)

    Returns:
        dict of aggregate performance metrics from MetricsCollector
    """
    sumo_bin, sumo_gui = find_sumo()
    binary = sumo_gui if (use_gui and os.path.isfile(sumo_gui)) else sumo_bin

    sumo_cmd = [
        binary, "-c", SUMO_CONFIG,
        "--seed", str(seed),
        "--no-step-log", "true",
    ]
    if use_gui:
        sumo_cmd.append("--start")

    # Start SUMO
    traci.start(sumo_cmd)

    # Initialize components
    collector = TrafficStateCollector()
    metrics = MetricsCollector()

    if mode == "fixed":
        controller = FixedSignalController()
    else:
        controller = AdaptiveController(verbose=False)

    # Simulation loop
    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

            # Advance signal controller
            if mode == "fixed":
                controller.step(dt=1.0)
            else:
                controller.step(collector)

            # Collect metrics
            metrics.step()

    except traci.exceptions.FatalTraCIError:
        pass

    # Get results before closing
    results = metrics.get_results()

    try:
        traci.close()
    except Exception:
        pass

    return results


# -- Result formatting ------------------------------------------------------

def print_comparison(fixed, adaptive):
    """Print a formatted side-by-side comparison of experiment results."""

    def pct_improvement(fixed_val, adaptive_val, lower_is_better=True):
        """Calculate percentage improvement. Returns string."""
        if fixed_val == 0:
            return "N/A"
        if lower_is_better:
            pct = ((fixed_val - adaptive_val) / fixed_val) * 100
        else:
            pct = ((adaptive_val - fixed_val) / fixed_val) * 100
        sign = "+" if pct > 0 else ""
        return f"{sign}{pct:.1f}%"

    w = 62  # total width

    print()
    print("=" * w)
    print("   SignalX -- Experiment Results")
    print("=" * w)
    print()

    # Header
    print(f"  {'Metric':<28s} {'Fixed':>10s} {'Adaptive':>10s} {'Change':>10s}")
    print("  " + "-" * (w - 4))

    # Row helper
    def row(label, fval, aval, unit="", lower_is_better=True):
        f_str = f"{fval}{unit}"
        a_str = f"{aval}{unit}"
        change = pct_improvement(fval, aval, lower_is_better)
        print(f"  {label:<28s} {f_str:>10s} {a_str:>10s} {change:>10s}")

    row("Avg Waiting Time (s)",
        fixed["avg_waiting_time"], adaptive["avg_waiting_time"],
        lower_is_better=True)

    row("Max Queue Length",
        fixed["max_queue_length"], adaptive["max_queue_length"],
        lower_is_better=True)

    row("Avg Queue Length",
        fixed["avg_queue_length"], adaptive["avg_queue_length"],
        lower_is_better=True)

    row("Throughput (vehicles)",
        fixed["throughput"], adaptive["throughput"],
        lower_is_better=False)

    row("Simulation Time (s)",
        fixed["total_sim_time"], adaptive["total_sim_time"])

    row("Vehicles Remaining",
        fixed["vehicles_remaining"], adaptive["vehicles_remaining"],
        lower_is_better=True)

    print()

    # Per-corridor breakdown
    print("  " + "-" * (w - 4))
    print(f"  {'Per-Corridor Breakdown':<28s}")
    print("  " + "-" * (w - 4))

    row("Max Queue (NS)",
        fixed["max_queue_ns"], adaptive["max_queue_ns"],
        lower_is_better=True)

    row("Max Queue (EW)",
        fixed["max_queue_ew"], adaptive["max_queue_ew"],
        lower_is_better=True)

    row("Avg Queue (NS)",
        fixed["avg_queue_ns"], adaptive["avg_queue_ns"],
        lower_is_better=True)

    row("Avg Queue (EW)",
        fixed["avg_queue_ew"], adaptive["avg_queue_ew"],
        lower_is_better=True)

    print()

    # Summary
    print("  " + "-" * (w - 4))
    print("  Key Improvements (positive = adaptive is better):")
    print()

    wt_imp = pct_improvement(
        fixed["avg_waiting_time"], adaptive["avg_waiting_time"],
        lower_is_better=True)
    q_imp = pct_improvement(
        fixed["max_queue_length"], adaptive["max_queue_length"],
        lower_is_better=True)
    tp_imp = pct_improvement(
        fixed["throughput"], adaptive["throughput"],
        lower_is_better=False)

    print(f"    Waiting time reduction : {wt_imp}")
    print(f"    Peak queue reduction   : {q_imp}")
    print(f"    Throughput improvement : {tp_imp}")
    print()
    print("=" * w)


# -- Main -------------------------------------------------------------------

def main():
    use_gui = "--gui" in sys.argv

    print()
    print("=" * 62)
    print("   SignalX -- Part 6: Baseline and Performance Metrics")
    print("=" * 62)
    print()
    print(f"  SUMO config  : {SUMO_CONFIG}")
    print(f"  Random seed  : {RANDOM_SEED}")
    print(f"  GUI mode     : {'Yes' if use_gui else 'No'}")
    print()

    # -- Experiment 1: Fixed-time control --
    print("-" * 62)
    print("  Running Experiment 1: FIXED-TIME CONTROL (30s/30s)")
    print("-" * 62)
    t0 = time.time()
    fixed_results = run_experiment("fixed", use_gui=use_gui)
    t1 = time.time()
    print(f"  Completed in {t1 - t0:.1f}s wall-clock time")
    print(f"  Throughput: {fixed_results['throughput']} vehicles")
    print(f"  Avg waiting time: {fixed_results['avg_waiting_time']}s")
    print()

    # -- Experiment 2: Adaptive control --
    print("-" * 62)
    print("  Running Experiment 2: ADAPTIVE CONTROL (demand-based)")
    print("-" * 62)
    t0 = time.time()
    adaptive_results = run_experiment("adaptive", use_gui=use_gui)
    t1 = time.time()
    print(f"  Completed in {t1 - t0:.1f}s wall-clock time")
    print(f"  Throughput: {adaptive_results['throughput']} vehicles")
    print(f"  Avg waiting time: {adaptive_results['avg_waiting_time']}s")
    print()

    # -- Comparison --
    print_comparison(fixed_results, adaptive_results)


if __name__ == "__main__":
    main()
