"""
SignalX — Fixed Signal Controller
Manages a two-phase traffic signal with safe transitions.

Phase cycle:
    Phase A (NS Green) → Yellow → All-Red → Phase B (EW Green) → Yellow → All-Red → repeat

Signal state string has 16 link indices:
    Index  0- 3: N_to_C connections (right, straight, straight, left)
    Index  4- 7: E_to_C connections (right, straight, straight, left)
    Index  8-11: S_to_C connections (right, straight, straight, left)
    Index 12-15: W_to_C connections (right, straight, straight, left)
"""

import traci

# Traffic light ID (matches the junction ID in the network)
TLS_ID = "C"

# ── Timing constants (seconds) ──────────────────────────────────────
MIN_GREEN = 30   # Fixed green time per phase
YELLOW = 3       # Yellow transition
ALL_RED = 1      # All-red clearance

# ── Signal state strings (16 characters) ────────────────────────────
#   Indices: 0123 4567 89AB CDEF
#            N→   E→   S→   W→
#
# Phase A: N/S green, E/W red
STATE_NS_GREEN  = "GGGgrrrrGGGgrrrr"   # N+S get green
STATE_NS_YELLOW = "yyyyrrrryyyyrrrr"   # N+S yellow transition
STATE_ALL_RED   = "rrrrrrrrrrrrrrrr"   # All red clearance
# Phase B: E/W green, N/S red
STATE_EW_GREEN  = "rrrrGGGgrrrrGGGg"   # E+W get green
STATE_EW_YELLOW = "rrrryyyyrrrryyyy"   # E+W yellow transition


# Phase definitions: (state_string, duration, phase_name)
PHASE_CYCLE = [
    (STATE_NS_GREEN,  MIN_GREEN, "NS_GREEN"),
    (STATE_NS_YELLOW, YELLOW,    "NS_YELLOW"),
    (STATE_ALL_RED,   ALL_RED,   "ALL_RED"),
    (STATE_EW_GREEN,  MIN_GREEN, "EW_GREEN"),
    (STATE_EW_YELLOW, YELLOW,    "EW_YELLOW"),
    (STATE_ALL_RED,   ALL_RED,   "ALL_RED"),
]

TOTAL_CYCLE_TIME = sum(d for _, d, _ in PHASE_CYCLE)


class FixedSignalController:
    """
    A fixed-timing two-phase signal controller.
    
    Manages the signal cycle using TraCI. The controller tracks elapsed time
    within the current phase and transitions to the next phase when the
    duration expires.
    
    The interface is designed so a future adaptive controller can replace
    the green durations dynamically via set_phase() or apply_signal_action().
    """

    def __init__(self, tls_id=TLS_ID):
        self.tls_id = tls_id
        self.current_phase_index = 0
        self.time_in_phase = 0.0
        self._apply_current_phase()

    def _apply_current_phase(self):
        """Set the TLS to the current phase state string."""
        state_str, _, _ = PHASE_CYCLE[self.current_phase_index]
        traci.trafficlight.setRedYellowGreenState(self.tls_id, state_str)

    def step(self, dt=1.0):
        """
        Advance the signal controller by dt seconds.
        
        Call this once per simulation step. Handles phase transitions
        including yellow and all-red clearance phases.
        
        Returns:
            str: Name of the current phase after this step.
        """
        self.time_in_phase += dt
        _, duration, _ = PHASE_CYCLE[self.current_phase_index]

        if self.time_in_phase >= duration:
            # Move to next phase
            self.current_phase_index = (self.current_phase_index + 1) % len(PHASE_CYCLE)
            self.time_in_phase = 0.0
            self._apply_current_phase()

        _, _, phase_name = PHASE_CYCLE[self.current_phase_index]
        return phase_name

    def get_current_phase_name(self):
        """Return the human-readable name of the current phase."""
        _, _, phase_name = PHASE_CYCLE[self.current_phase_index]
        return phase_name

    def get_phase_remaining(self):
        """Return seconds remaining in the current phase."""
        _, duration, _ = PHASE_CYCLE[self.current_phase_index]
        return max(0.0, duration - self.time_in_phase)

    # ── Future adaptive interface ───────────────────────────────────
    def set_phase(self, phase_index):
        """
        Force a specific phase (for future adaptive controller).
        Only allowed to set green phases (index 0 or 3).
        """
        if phase_index in (0, 3):
            self.current_phase_index = phase_index
            self.time_in_phase = 0.0
            self._apply_current_phase()

    def apply_signal_action(self, action):
        """
        Apply an action from the adaptive controller (future use).
        
        Placeholder interface. The adaptive controller will call this
        with an action dict specifying desired green durations or
        phase selection.
        
        For now, this is a no-op.
        """
        # Will be implemented in Phase 3+
        pass