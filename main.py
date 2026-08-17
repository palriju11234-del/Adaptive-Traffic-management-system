"""
SignalX — Main Simulation Runner (Phase 1 + Phase 2 + Part 3)

Phase 1: SUMO + Python + TraCI connection
Phase 2: Single four-arm junction with fixed two-phase traffic signal
Part 3:  Real-time traffic state collection through TraCI

Usage:
    python main.py          Run headless (no GUI window)
    python main.py --gui    Run with SUMO-GUI (visual)
"""

import os
import sys
import traci
import sumolib

# Add project root to path for controller imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from controller.traffic_state import TrafficStateCollector
from controller.signal_controller import FixedSignalController


# ── Configuration ───────────────────────────────────────────────────
STATE_UPDATE_INTERVAL = 5   # Collect and print traffic state every N seconds
CSV_LOG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "evaluation", "traffic_state.csv"
)


def find_sumo():
    """Locate SUMO installation and return paths to sumo and sumo-gui binaries."""
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
        sumo_gui = os.path.join(sumo_home, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            print(f"SUMO_HOME detected: {sumo_home}")
            return sumo_bin, sumo_gui

    # Fallback: common Windows installation paths
    common_paths = [
        r"C:\Program Files (x86)\Eclipse\Sumo",
        r"C:\Program Files\Eclipse\Sumo",
        r"C:\sumo",
    ]
    for path in common_paths:
        sumo_bin = os.path.join(path, "bin", "sumo.exe")
        sumo_gui = os.path.join(path, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            print(f"SUMO detected at: {path}")
            return sumo_bin, sumo_gui

    print("ERROR: SUMO installation not found.")
    print("Please set the SUMO_HOME environment variable.")
    print("Example: set SUMO_HOME=C:\\Program Files (x86)\\Eclipse\\Sumo")
    sys.exit(1)


def main():
    use_gui = "--gui" in sys.argv

    print()
    print("=" * 56)
    print("   SignalX - Adaptive Traffic Management System")
    print("   Part 3: Real-Time Traffic State Collection")
    print("=" * 56)
    print()

    # ── Detect SUMO ─────────────────────────────────────────────────
    sumo_bin, sumo_gui = find_sumo()
    binary = sumo_gui if (use_gui and os.path.isfile(sumo_gui)) else sumo_bin
    mode_label = "SUMO-GUI" if use_gui else "SUMO (headless)"
    print(f"Mode      : {mode_label}")
    print(f"Binary    : {binary}")

    # ── Locate config ───────────────────────────────────────────────
    project_root = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(project_root, "sumo", "simulation.sumocfg")
    if not os.path.isfile(config_path):
        print(f"ERROR: SUMO config not found at {config_path}")
        sys.exit(1)
    print(f"Config    : {config_path}")
    print(f"CSV Log   : {CSV_LOG_PATH}")
    print()

    # ── Remove old CSV log for a fresh run ──────────────────────────
    if os.path.isfile(CSV_LOG_PATH):
        os.remove(CSV_LOG_PATH)

    # ── Start SUMO ──────────────────────────────────────────────────
    sumo_cmd = [binary, "-c", config_path]
    if use_gui:
        sumo_cmd.append("--start")

    print(f"Starting {mode_label}...")
    traci.start(sumo_cmd)
    print("SUMO started successfully")
    print("TraCI connection established")
    print()

    # ── Initialize controllers ──────────────────────────────────────
    signal_controller = FixedSignalController()
    collector = TrafficStateCollector()

    print("Signal controller : Fixed timing (30s green / 3s yellow / 1s all-red)")
    print("State collector   : TrafficStateCollector (lane-level, per-vehicle)")
    print(f"Report interval   : every {STATE_UPDATE_INTERVAL}s")
    print()

    # ── Simulation loop ────────────────────────────────────────────
    step = 0
    last_report_time = -STATE_UPDATE_INTERVAL  # Force first report

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()
            step += 1

            # Advance the signal controller
            phase_name = signal_controller.step(dt=1.0)

            # Periodic traffic state collection
            if sim_time - last_report_time >= STATE_UPDATE_INTERVAL:
                state = collector.get_state(signal_phase_name=phase_name)

                # Print to console
                print()
                collector.print_state(state)

                # Log to CSV
                collector.log_state(state, CSV_LOG_PATH)

                last_report_time = sim_time

    except traci.exceptions.FatalTraCIError:
        print("\nSUMO connection lost (simulation may have been closed).")
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")

    # ── Clean shutdown ──────────────────────────────────────────────
    try:
        traci.close()
        print("\nTraCI connection closed successfully")
    except Exception:
        pass

    print()
    print("=" * 56)
    print("   Simulation Complete")
    print(f"   Total steps : {step}")
    print(f"   CSV log     : {CSV_LOG_PATH}")
    print("=" * 56)


if __name__ == "__main__":
    main()