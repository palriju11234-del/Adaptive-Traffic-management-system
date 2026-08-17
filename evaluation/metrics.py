"""
SignalX — Part 6: Performance Metrics Collector

Collects per-vehicle and per-step traffic metrics during a SUMO simulation
for experimental evaluation.

Metrics collected:
    - Average waiting time   : mean accumulated waiting time of completed vehicles
    - Maximum queue length   : peak total queue across all approach lanes
    - Average queue length   : mean total queue across all steps
    - Throughput             : number of vehicles that completed their route
    - Total simulation time  : simulation duration in seconds

Queue definition:
    A vehicle is counted as "queued" (halted) when its speed is below SUMO's
    default halting threshold (~0.1 m/s).  This is consistent with the
    Part 3 TrafficStateCollector queue definition.

Architecture:
    SUMO -> TraCI -> MetricsCollector -> Aggregate Results
"""

import traci

# Approach lane IDs (same as traffic_state.py)
_APPROACH_LANES = {
    "north_south": [
        "N_to_C_0", "N_to_C_1",
        "S_to_C_0", "S_to_C_1",
    ],
    "east_west": [
        "E_to_C_0", "E_to_C_1",
        "W_to_C_0", "W_to_C_1",
    ],
}


class MetricsCollector:
    """
    Accumulates per-step and per-vehicle metrics over an entire simulation.

    Usage:
        mc = MetricsCollector()
        # Inside simulation loop, after traci.simulationStep():
        mc.step()
        # After simulation ends:
        results = mc.get_results()
    """

    def __init__(self, approach_lanes=None):
        self.approach_lanes = approach_lanes or _APPROACH_LANES

        # Per-vehicle tracking
        self._vehicle_waiting = {}          # vid -> last known accumulated waiting time
        self._completed_waiting_times = []  # final waiting times of vehicles that arrived

        # Per-step queue samples
        self._queue_samples_ns = []
        self._queue_samples_ew = []

        # Counters
        self.total_arrived = 0
        self.total_steps = 0
        self.sim_end_time = 0.0

    def step(self):
        """
        Record metrics for one simulation step.

        Must be called once per step, AFTER traci.simulationStep().
        """
        self.total_steps += 1
        self.sim_end_time = traci.simulation.getTime()

        # -- Per-vehicle waiting time tracking --------------------------
        # Record waiting time for every active vehicle
        for vid in traci.vehicle.getIDList():
            try:
                self._vehicle_waiting[vid] = (
                    traci.vehicle.getAccumulatedWaitingTime(vid)
                )
            except traci.exceptions.TraCIException:
                pass

        # Detect vehicles that completed their route this step
        for vid in traci.simulation.getArrivedIDList():
            self.total_arrived += 1
            if vid in self._vehicle_waiting:
                self._completed_waiting_times.append(
                    self._vehicle_waiting.pop(vid)
                )

        # -- Per-step queue length (halted vehicles on approach lanes) --
        ns_queue = 0
        ew_queue = 0
        for lane_id in self.approach_lanes["north_south"]:
            ns_queue += traci.lane.getLastStepHaltingNumber(lane_id)
        for lane_id in self.approach_lanes["east_west"]:
            ew_queue += traci.lane.getLastStepHaltingNumber(lane_id)

        self._queue_samples_ns.append(ns_queue)
        self._queue_samples_ew.append(ew_queue)

    def get_results(self):
        """
        Compute final aggregate metrics.

        Returns:
            dict with keys:
                avg_waiting_time   (float)  seconds
                max_queue_length   (int)    vehicles (combined NS + EW)
                avg_queue_length   (float)  vehicles (combined NS + EW)
                max_queue_ns       (int)
                max_queue_ew       (int)
                avg_queue_ns       (float)
                avg_queue_ew       (float)
                throughput         (int)    completed vehicles
                total_sim_time     (float)  seconds
                total_steps        (int)
                vehicles_remaining (int)    still in network at end
        """
        # Waiting time
        if self._completed_waiting_times:
            avg_wt = (
                sum(self._completed_waiting_times)
                / len(self._completed_waiting_times)
            )
        else:
            avg_wt = 0.0

        # Combined queue
        combined = [
            ns + ew
            for ns, ew in zip(self._queue_samples_ns, self._queue_samples_ew)
        ]
        max_q = max(combined) if combined else 0
        avg_q = sum(combined) / len(combined) if combined else 0.0

        # Per-corridor queue
        max_q_ns = max(self._queue_samples_ns) if self._queue_samples_ns else 0
        max_q_ew = max(self._queue_samples_ew) if self._queue_samples_ew else 0
        avg_q_ns = (
            sum(self._queue_samples_ns) / len(self._queue_samples_ns)
            if self._queue_samples_ns else 0.0
        )
        avg_q_ew = (
            sum(self._queue_samples_ew) / len(self._queue_samples_ew)
            if self._queue_samples_ew else 0.0
        )

        return {
            "avg_waiting_time": round(avg_wt, 2),
            "max_queue_length": max_q,
            "avg_queue_length": round(avg_q, 2),
            "max_queue_ns": max_q_ns,
            "max_queue_ew": max_q_ew,
            "avg_queue_ns": round(avg_q_ns, 2),
            "avg_queue_ew": round(avg_q_ew, 2),
            "throughput": self.total_arrived,
            "total_sim_time": self.sim_end_time,
            "total_steps": self.total_steps,
            "vehicles_remaining": len(self._vehicle_waiting),
        }