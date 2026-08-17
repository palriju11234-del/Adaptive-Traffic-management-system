"""
SignalX — Part 5: Closed-Loop Adaptive Signal Controller

Continuously adapts green time based on the latest traffic state.

Closed-loop architecture:
    ┌─────────────────────────────────────────────────────┐
    │  Traffic Measurement (TrafficStateCollector)        │
    │         ↓                                          │
    │  Traffic State (vehicle counts per corridor)        │
    │         ↓                                          │
    │  Adaptive Green-Time Decision (get_action)          │
    │         ↓                                          │
    │  Safety-Constrained Signal Change (state machine)   │
    │         ↓                                          │
    │  Vehicles Move in SUMO (simulation step)            │
    │         ↓                                          │
    │  Traffic Conditions Change                          │
    │         ↓                                          │
    │  New Traffic Measurement  ←───── repeat ────────────┘
    └─────────────────────────────────────────────────────┘

Green time formula:
    green_time = 20 + (direction_demand / total_demand) * 40

Constraints:
    MIN_GREEN  = 20 seconds
    MAX_GREEN  = 60 seconds
    YELLOW     = 3  seconds
    ALL_RED    = 1  second

Phase sequence (safe transitions are guaranteed):
    NS_GREEN → NS_YELLOW → ALL_RED → EW_GREEN → EW_YELLOW → ALL_RED → repeat

Each full cycle:
    1. Measure current traffic
    2. Calculate NS and EW green durations
    3. Execute NS_GREEN  (calculated duration)
    4. Execute NS_YELLOW (3 s, fixed)
    5. Execute ALL_RED   (1 s, fixed)
    6. Measure/update traffic for EW decision
    7. Execute EW_GREEN  (calculated duration)
    8. Execute EW_YELLOW (3 s, fixed)
    9. Execute ALL_RED   (1 s, fixed)
   10. Start next adaptive cycle → go to step 1

This module is designed so the demand-based logic in get_action() can later
be replaced by a PPO/MAPPO policy without modifying the signal state machine.
"""

import traci


# ════════════════════════════════════════════════════════════════════
# CONSTANTS
# ════════════════════════════════════════════════════════════════════

TLS_ID = "C"

# Timing constraints (seconds)
MIN_GREEN = 20
MAX_GREEN = 60
DEFAULT_GREEN = 30      # Used when both directions have zero vehicles
YELLOW = 3
ALL_RED = 1

# Signal state strings (16 characters, matching signal_controller.py)
#   Indices: 0123 4567 89AB CDEF
#            N→   E→   S→   W→
STATE_NS_GREEN  = "GGGgrrrrGGGgrrrr"
STATE_NS_YELLOW = "yyyyrrrryyyyrrrr"
STATE_ALL_RED   = "rrrrrrrrrrrrrrrr"
STATE_EW_GREEN  = "rrrrGGGgrrrrGGGg"
STATE_EW_YELLOW = "rrrryyyyrrrryyyy"

# Phase definitions: (state_string, phase_name, is_green_phase)
# Durations for green phases are computed dynamically; yellow/all-red are fixed.
PHASE_SEQUENCE = [
    # index 0: NS green (duration set dynamically)
    {"state": STATE_NS_GREEN,  "name": "NS_GREEN",  "is_green": True,  "direction": "ns"},
    # index 1: NS yellow (fixed 3s)
    {"state": STATE_NS_YELLOW, "name": "NS_YELLOW", "is_green": False, "duration": YELLOW},
    # index 2: All-red (fixed 1s)
    {"state": STATE_ALL_RED,   "name": "ALL_RED",   "is_green": False, "duration": ALL_RED},
    # index 3: EW green (duration set dynamically)
    {"state": STATE_EW_GREEN,  "name": "EW_GREEN",  "is_green": True,  "direction": "ew"},
    # index 4: EW yellow (fixed 3s)
    {"state": STATE_EW_YELLOW, "name": "EW_YELLOW", "is_green": False, "duration": YELLOW},
    # index 5: All-red (fixed 1s)
    {"state": STATE_ALL_RED,   "name": "ALL_RED",   "is_green": False, "duration": ALL_RED},
]

# Phase indices for readability
_IDX_NS_GREEN  = 0
_IDX_NS_YELLOW = 1
_IDX_ALL_RED_1 = 2
_IDX_EW_GREEN  = 3
_IDX_EW_YELLOW = 4
_IDX_ALL_RED_2 = 5


# ════════════════════════════════════════════════════════════════════
# ADAPTIVE CONTROLLER
# ════════════════════════════════════════════════════════════════════

class AdaptiveController:
    """
    Closed-loop demand-based adaptive traffic signal controller.

    Reads traffic state from a TrafficStateCollector at the start of each
    half-cycle and computes green durations proportional to vehicle demand.
    The signal state machine guarantees safe transitions through yellow and
    all-red phases regardless of the computed green times.

    Measurement points:
        - Before NS_GREEN: measure traffic, compute NS green duration
        - Before EW_GREEN: measure traffic again, compute EW green duration
    This ensures EW green is based on conditions that may have changed
    during the NS phase, closing the feedback loop more tightly.

    Public interface:
        step(collector)   — advance by 1 second, returns current phase name
        get_action(state) — compute green durations from a traffic state dict

    The get_action() method is the only piece that needs to change when
    swapping in a PPO/MAPPO policy.
    """

    def __init__(self, tls_id=TLS_ID, verbose=True):
        self.tls_id = tls_id
        self.verbose = verbose

        # State machine
        self.current_phase_index = 0
        self.time_in_phase = 0.0

        # Green durations for the current cycle (computed adaptively)
        self.ns_green_duration = DEFAULT_GREEN
        self.ew_green_duration = DEFAULT_GREEN

        # Cycle counter (a full cycle = NS_GREEN..ALL_RED..EW_GREEN..ALL_RED)
        self.cycle_number = 0

        # Flags: have we measured for NS / EW in this cycle?
        self._ns_measured = False
        self._ew_measured = False

        # Last computed actions (for logging and inspection)
        self.last_ns_action = None
        self.last_ew_action = None

        # Phase transition log for the current cycle
        self._cycle_phase_log = []

        # Apply initial phase
        self._apply_current_phase()

    # ── Main simulation interface ──────────────────────────────────

    def step(self, collector):
        """
        Advance the adaptive controller by one simulation second.

        Measurement points:
            - At the start of NS_GREEN (phase 0): measure traffic and
              compute both NS and EW green durations for the cycle.
            - At the start of EW_GREEN (phase 3): re-measure traffic and
              update the EW green duration with fresh data.

        Args:
            collector: TrafficStateCollector instance (from Part 3).

        Returns:
            str: Name of the current phase after this step.
        """
        # ── Measurement point 1: start of NS_GREEN ─────────────────
        if self.current_phase_index == _IDX_NS_GREEN and not self._ns_measured:
            self.cycle_number += 1
            self._cycle_phase_log = []

            state = collector.get_state()
            action = self.get_action(state)
            self.ns_green_duration = action["ns_green"]
            self.ew_green_duration = action["ew_green"]  # preliminary
            self.last_ns_action = action
            self._ns_measured = True

            if self.verbose:
                self._log_cycle_header(action, state)

        # ── Measurement point 2: start of EW_GREEN ─────────────────
        if self.current_phase_index == _IDX_EW_GREEN and not self._ew_measured:
            state = collector.get_state()
            action = self.get_action(state)
            self.ew_green_duration = action["ew_green"]  # updated with fresh data
            self.last_ew_action = action
            self._ew_measured = True

            if self.verbose:
                self._log_ew_update(action, state)

        # ── Track phase transitions for the cycle log ──────────────
        current_name = PHASE_SEQUENCE[self.current_phase_index]["name"]
        if not self._cycle_phase_log or self._cycle_phase_log[-1] != current_name:
            self._cycle_phase_log.append(current_name)
            if self.verbose:
                duration = self._get_current_duration()
                self._log_phase_enter(current_name, duration)

        # ── Determine the duration of the current phase ────────────
        duration = self._get_current_duration()

        # ── Advance time ───────────────────────────────────────────
        self.time_in_phase += 1.0

        if self.time_in_phase >= duration:
            # Transition to next phase
            next_index = (self.current_phase_index + 1) % len(PHASE_SEQUENCE)

            # If wrapping back to phase 0, reset measurement flags
            if next_index == _IDX_NS_GREEN:
                if self.verbose:
                    self._log_cycle_footer()
                self._ns_measured = False
                self._ew_measured = False

            self.current_phase_index = next_index
            self.time_in_phase = 0.0
            self._apply_current_phase()

        return self.get_current_phase_name()

    # ── Demand-based action computation ────────────────────────────

    def get_action(self, traffic_state):
        """
        Compute green durations from the current traffic state.

        This is the single method that would be replaced by a PPO policy.

        Args:
            traffic_state: dict from TrafficStateCollector.get_state()
                {
                    "north_south": {"vehicle_count": int, ...},
                    "east_west":   {"vehicle_count": int, ...},
                    ...
                }

        Returns:
            dict: {
                "ns_green":   int,   # green duration for NS (clamped 20–60)
                "ew_green":   int,   # green duration for EW (clamped 20–60)
                "ns_vehicles": int,
                "ew_vehicles": int,
                "ns_demand_pct": float,
                "ew_demand_pct": float,
                "decision": str,     # human-readable decision summary
            }
        """
        ns_vehicles = traffic_state["north_south"]["vehicle_count"]
        ew_vehicles = traffic_state["east_west"]["vehicle_count"]
        total = ns_vehicles + ew_vehicles

        # Edge case: no vehicles in either direction
        if total == 0:
            return {
                "ns_green": DEFAULT_GREEN,
                "ew_green": DEFAULT_GREEN,
                "ns_vehicles": 0,
                "ew_vehicles": 0,
                "ns_demand_pct": 50.0,
                "ew_demand_pct": 50.0,
                "decision": "No vehicles detected \u2014 using default 30/30",
            }

        # Demand ratios
        ns_ratio = ns_vehicles / total
        ew_ratio = ew_vehicles / total

        # Green time formula: 20 + (direction_demand / total_demand) * 40
        ns_green_raw = 20 + ns_ratio * 40
        ew_green_raw = 20 + ew_ratio * 40

        # Clamp to [MIN_GREEN, MAX_GREEN]
        ns_green = int(round(max(MIN_GREEN, min(MAX_GREEN, ns_green_raw))))
        ew_green = int(round(max(MIN_GREEN, min(MAX_GREEN, ew_green_raw))))

        # Decision description
        if ns_vehicles > ew_vehicles:
            decision = "NS receives longer green"
        elif ew_vehicles > ns_vehicles:
            decision = "EW receives longer green"
        else:
            decision = "Equal demand \u2014 balanced green"

        return {
            "ns_green": ns_green,
            "ew_green": ew_green,
            "ns_vehicles": ns_vehicles,
            "ew_vehicles": ew_vehicles,
            "ns_demand_pct": round(ns_ratio * 100, 1),
            "ew_demand_pct": round(ew_ratio * 100, 1),
            "decision": decision,
        }

    # ── Internal helpers ───────────────────────────────────────────

    def _get_current_duration(self):
        """Return the duration for the current phase."""
        phase = PHASE_SEQUENCE[self.current_phase_index]
        if phase["is_green"]:
            if phase["direction"] == "ns":
                return self.ns_green_duration
            else:
                return self.ew_green_duration
        else:
            return phase["duration"]

    def _apply_current_phase(self):
        """Set the TLS to the current phase state string via TraCI."""
        state_str = PHASE_SEQUENCE[self.current_phase_index]["state"]
        traci.trafficlight.setRedYellowGreenState(self.tls_id, state_str)

    def get_current_phase_name(self):
        """Return the human-readable name of the current phase."""
        return PHASE_SEQUENCE[self.current_phase_index]["name"]

    def get_phase_remaining(self):
        """Return seconds remaining in the current phase."""
        duration = self._get_current_duration()
        return max(0.0, duration - self.time_in_phase)

    # ── Logging ────────────────────────────────────────────────────

    def _log_cycle_header(self, action, state):
        """Print the start-of-cycle adaptive decision block."""
        sim_time = state.get("timestamp", "?")
        ns_v = action["ns_vehicles"]
        ew_v = action["ew_vehicles"]
        ns_pct = action["ns_demand_pct"]
        ew_pct = action["ew_demand_pct"]
        ns_g = action["ns_green"]
        ew_g = action["ew_green"]
        decision = action["decision"]

        print()
        print(f"========== ADAPTIVE CYCLE {self.cycle_number} "
              f"(t={sim_time}s) ==========")
        print()
        print("  Current Traffic:")
        print(f"    NS vehicles : {ns_v}")
        print(f"    EW vehicles : {ew_v}")
        print()
        print("  Calculated:")
        print(f"    NS demand   : {ns_pct}%")
        print(f"    EW demand   : {ew_pct}%")
        print(f"    NS green    : {ns_g} sec")
        print(f"    EW green    : {ew_g} sec  (preliminary, will re-measure)")
        print()
        print(f"  Decision      : {decision}")
        print()
        print("  Executing:")

    @staticmethod
    def _log_ew_update(action, state):
        """Print the mid-cycle EW re-measurement update."""
        sim_time = state.get("timestamp", "?")
        ns_v = action["ns_vehicles"]
        ew_v = action["ew_vehicles"]
        ew_g = action["ew_green"]

        print()
        print(f"  [Mid-cycle re-measurement at t={sim_time}s]")
        print(f"    NS vehicles : {ns_v}  |  EW vehicles : {ew_v}")
        print(f"    EW green updated to : {ew_g} sec")
        print()

    @staticmethod
    def _log_phase_enter(phase_name, duration):
        """Log a single phase transition."""
        print(f"    -> {phase_name:<12s}  ({duration}s)")

    def _log_cycle_footer(self):
        """Print the end-of-cycle separator."""
        print()
        print("=" * 50)
