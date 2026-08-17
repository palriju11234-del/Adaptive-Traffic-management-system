"""
SignalX — Part 3: Real-Time Traffic State Collection

Reads live traffic data from SUMO via TraCI at the lane and vehicle level.
Provides structured traffic state for the North/South and East/West corridors.

Collected metrics (per corridor):
    - vehicle_count        : number of vehicles on approach lanes
    - queue_length         : vehicles with speed <= QUEUE_SPEED_THRESHOLD
    - average_waiting_time : mean accumulated waiting time (seconds)
    - average_speed        : mean speed in m/s

Also reads the current traffic signal phase from the TLS controller.

Architecture:
    SUMO  →  TraCI  →  TrafficStateCollector  →  Structured Traffic State

This module is observation-only. It does NOT make signal-control decisions.
"""

import csv
import os
import traci


# ════════════════════════════════════════════════════════════════════
# LANE GROUP CONFIGURATION
# ════════════════════════════════════════════════════════════════════
#
# Actual lane IDs from sumo/network.net.xml
# Each incoming edge has 2 lanes (index 0 and 1).
#
# Incoming edges:
#   North → Junction: N_to_C   (lanes: N_to_C_0, N_to_C_1)
#   South → Junction: S_to_C   (lanes: S_to_C_0, S_to_C_1)
#   East  → Junction: E_to_C   (lanes: E_to_C_0, E_to_C_1)
#   West  → Junction: W_to_C   (lanes: W_to_C_0, W_to_C_1)

APPROACH_LANES = {
    "north_south": [
        "N_to_C_0", "N_to_C_1",   # North approach
        "S_to_C_0", "S_to_C_1",   # South approach
    ],
    "east_west": [
        "E_to_C_0", "E_to_C_1",   # East approach
        "W_to_C_0", "W_to_C_1",   # West approach
    ],
}

# Traffic light ID (matches the junction node ID in the network)
TLS_ID = "C"

# A vehicle is considered "queued" if its speed is at or below this (m/s)
QUEUE_SPEED_THRESHOLD = 0.1

# Map signal-controller phase names to human-readable descriptions
PHASE_DISPLAY_NAMES = {
    "NS_GREEN":  "NORTH/SOUTH GREEN",
    "NS_YELLOW": "NORTH/SOUTH YELLOW",
    "EW_GREEN":  "EAST/WEST GREEN",
    "EW_YELLOW": "EAST/WEST YELLOW",
    "ALL_RED":   "ALL RED",
}


# ════════════════════════════════════════════════════════════════════
# TRAFFIC STATE COLLECTOR
# ════════════════════════════════════════════════════════════════════

class TrafficStateCollector:
    """
    Reads real-time traffic data from SUMO through TraCI.

    Uses lane-level and vehicle-level APIs to compute per-corridor
    metrics. Designed to be consumed by a future adaptive controller
    without any coupling to signal-control logic.

    Usage:
        collector = TrafficStateCollector()
        state = collector.get_state()        # dict
        collector.print_state(state)         # formatted console output
        collector.log_state(state, path)     # append row to CSV
    """

    def __init__(self, tls_id=TLS_ID, approach_lanes=None,
                 queue_speed_threshold=QUEUE_SPEED_THRESHOLD):
        """
        Args:
            tls_id:               Traffic-light controller ID in the network.
            approach_lanes:       Dict mapping corridor name → list of lane IDs.
                                  Defaults to APPROACH_LANES.
            queue_speed_threshold: Speed (m/s) at or below which a vehicle
                                  is counted as queued. Default 0.1 m/s.
        """
        self.tls_id = tls_id
        self.approach_lanes = approach_lanes or APPROACH_LANES
        self.queue_speed_threshold = queue_speed_threshold

    # ── Core public API ────────────────────────────────────────────

    def get_state(self, signal_phase_name=None):
        """
        Collect the full traffic state from SUMO.

        Args:
            signal_phase_name:  Optional phase name string from the
                                FixedSignalController (e.g. "NS_GREEN").
                                If None, the phase is read directly from
                                the SUMO TLS program.

        Returns:
            dict:
                {
                    "timestamp": float,
                    "north_south": { vehicle_count, queue_length,
                                     average_waiting_time, average_speed },
                    "east_west":   { vehicle_count, queue_length,
                                     average_waiting_time, average_speed },
                    "signal": { traffic_light_id, phase_index, phase_name }
                }
        """
        timestamp = traci.simulation.getTime()

        ns_state = self._collect_corridor("north_south")
        ew_state = self._collect_corridor("east_west")
        signal_state = self._collect_signal(signal_phase_name)

        return {
            "timestamp": timestamp,
            "north_south": ns_state,
            "east_west": ew_state,
            "signal": signal_state,
        }

    # ── Corridor metrics (private) ─────────────────────────────────

    def _collect_corridor(self, corridor_name):
        """
        Compute metrics for one corridor by iterating over its lanes
        and the individual vehicles on those lanes.

        Uses a set of vehicle IDs to avoid double-counting vehicles
        that might appear during lane changes.
        """
        lane_ids = self.approach_lanes[corridor_name]

        seen_vehicles = set()
        total_waiting_time = 0.0
        total_speed = 0.0
        queue_length = 0

        for lane_id in lane_ids:
            veh_ids = traci.lane.getLastStepVehicleIDs(lane_id)
            for vid in veh_ids:
                if vid in seen_vehicles:
                    continue
                seen_vehicles.add(vid)

                speed = traci.vehicle.getSpeed(vid)
                waiting = traci.vehicle.getAccumulatedWaitingTime(vid)

                total_speed += speed
                total_waiting_time += waiting

                if speed <= self.queue_speed_threshold:
                    queue_length += 1

        vehicle_count = len(seen_vehicles)

        # Safe averages (handle zero-vehicle case)
        if vehicle_count > 0:
            average_waiting_time = round(total_waiting_time / vehicle_count, 1)
            average_speed = round(total_speed / vehicle_count, 2)
        else:
            average_waiting_time = 0.0
            average_speed = 0.0

        return {
            "vehicle_count": vehicle_count,
            "queue_length": queue_length,
            "average_waiting_time": average_waiting_time,
            "average_speed": average_speed,
        }

    # ── Signal phase (private) ─────────────────────────────────────

    def _collect_signal(self, phase_name_override=None):
        """
        Read the current traffic-light phase.

        If phase_name_override is provided (from the FixedSignalController),
        use that. Otherwise, read the phase index from SUMO directly and
        map it to a name.
        """
        phase_index = traci.trafficlight.getPhase(self.tls_id)

        if phase_name_override:
            display = PHASE_DISPLAY_NAMES.get(phase_name_override,
                                               phase_name_override)
        else:
            # Fallback: map SUMO's built-in program phase indices
            # (only used if controller doesn't pass a name)
            _sumo_phase_map = {
                0: "NORTH/SOUTH GREEN",
                1: "NORTH/SOUTH YELLOW",
                2: "EAST/WEST GREEN",
                3: "EAST/WEST YELLOW",
            }
            display = _sumo_phase_map.get(phase_index,
                                           f"PHASE_{phase_index}")

        return {
            "traffic_light_id": self.tls_id,
            "phase_index": phase_index,
            "phase_name": display,
        }

    # ── Console output ─────────────────────────────────────────────

    @staticmethod
    def print_state(state):
        """Print a formatted traffic-state block to the console."""
        ts = state["timestamp"]
        ns = state["north_south"]
        ew = state["east_west"]
        sig = state["signal"]

        # Convert m/s → km/h for display
        ns_kmh = round(ns["average_speed"] * 3.6, 1)
        ew_kmh = round(ew["average_speed"] * 3.6, 1)

        print("=" * 52)
        print(f"  TRAFFIC STATE  |  Simulation Time: {ts:.1f} s")
        print("=" * 52)
        print()
        print("  NORTH / SOUTH")
        print(f"    Vehicles       : {ns['vehicle_count']}")
        print(f"    Queue          : {ns['queue_length']}")
        print(f"    Avg Wait       : {ns['average_waiting_time']} s")
        print(f"    Avg Speed      : {ns['average_speed']} m/s  ({ns_kmh} km/h)")
        print()
        print("  EAST / WEST")
        print(f"    Vehicles       : {ew['vehicle_count']}")
        print(f"    Queue          : {ew['queue_length']}")
        print(f"    Avg Wait       : {ew['average_waiting_time']} s")
        print(f"    Avg Speed      : {ew['average_speed']} m/s  ({ew_kmh} km/h)")
        print()
        print("  SIGNAL")
        print(f"    Controller     : {sig['traffic_light_id']}")
        print(f"    Current Phase  : {sig['phase_name']}")
        print()
        print("=" * 52)

    # ── CSV logging ────────────────────────────────────────────────

    @staticmethod
    def log_state(state, csv_path):
        """
        Append one row of traffic-state data to a CSV file.

        Creates the file with a header row if it does not yet exist.

        Args:
            state:    dict returned by get_state().
            csv_path: absolute path to the output CSV file.
        """
        fieldnames = [
            "timestamp",
            "ns_vehicle_count", "ns_queue_length",
            "ns_avg_waiting_time", "ns_avg_speed",
            "ew_vehicle_count", "ew_queue_length",
            "ew_avg_waiting_time", "ew_avg_speed",
            "phase",
        ]

        file_exists = os.path.isfile(csv_path)

        # Ensure the directory exists
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)

        with open(csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()

            ns = state["north_south"]
            ew = state["east_west"]

            writer.writerow({
                "timestamp":            state["timestamp"],
                "ns_vehicle_count":     ns["vehicle_count"],
                "ns_queue_length":      ns["queue_length"],
                "ns_avg_waiting_time":  ns["average_waiting_time"],
                "ns_avg_speed":         ns["average_speed"],
                "ew_vehicle_count":     ew["vehicle_count"],
                "ew_queue_length":      ew["queue_length"],
                "ew_avg_waiting_time":  ew["average_waiting_time"],
                "ew_avg_speed":         ew["average_speed"],
                "phase":                state["signal"]["phase_name"],
            })