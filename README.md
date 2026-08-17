# SignalX — AI-Powered Adaptive Smart Traffic Management System

> SIH Project: AI-Powered Adaptive Smart Traffic Management and Multi-Junction Coordination System

---

## Current Prototype

**Phase 1:** SUMO + Python + TraCI connection ✅  
**Phase 2:** Single four-arm junction with fixed two-phase traffic signal ✅  
**Part 3:** Real-time traffic state collection through TraCI ✅

### Running the Simulation

```bash
# Headless mode (no GUI)
python main.py

# With SUMO-GUI (visual)
python main.py --gui
```

---

## Architecture

```
SUMO (Simulation)
     ↓
   TraCI
     ↓
TrafficStateCollector        (controller/traffic_state.py)
     ↓
Structured Traffic State     → console output + CSV log
     ↓
Fixed Signal Controller      (controller/signal_controller.py)
     ↓
   SUMO
```

### Signal Cycle

```
Phase A: North/South GREEN  (30s)
         ↓
         Yellow             (3s)
         ↓
         All Red            (1s)
         ↓
Phase B: East/West GREEN    (30s)
         ↓
         Yellow             (3s)
         ↓
         All Red            (1s)
         ↓
         (repeat)
```

### Traffic Demand (Intentionally Unbalanced)

| Corridor    | Vehicles | Purpose                              |
|-------------|----------|--------------------------------------|
| North/South | 60       | Heavy traffic (future adaptive demo) |
| East/West   | 20       | Light traffic (future adaptive demo) |

---

## Part 3 — Traffic State Collection

The system extracts real-time traffic-state metadata from SUMO through TraCI using a reusable `TrafficStateCollector` class.

### Collected State Variables

| Variable             | Unit   | Description                                     |
|----------------------|--------|-------------------------------------------------|
| Vehicle count        | count  | Vehicles on approach lanes (per corridor)        |
| Queue length         | count  | Vehicles with speed ≤ 0.1 m/s                   |
| Average waiting time | s      | Mean accumulated waiting time per vehicle         |
| Average speed        | m/s    | Mean speed of vehicles on approach lanes          |
| Signal phase         | —      | Current TLS phase (e.g. NORTH/SOUTH GREEN)        |

### Data Flow

```
SUMO
 ↓
TraCI (lane-level + vehicle-level APIs)
 ↓
TrafficStateCollector
 ↓
Structured Traffic State (dict)
 ↓
├── Console output (every 5 simulation seconds)
└── CSV log (evaluation/traffic_state.csv)
```

### Lane Grouping

Traffic state is collected separately for two corridors based on actual lane IDs in `sumo/network.net.xml`:

| Corridor    | Approach Lanes                                    |
|-------------|---------------------------------------------------|
| North/South | `N_to_C_0`, `N_to_C_1`, `S_to_C_0`, `S_to_C_1` |
| East/West   | `E_to_C_0`, `E_to_C_1`, `W_to_C_0`, `W_to_C_1` |

### CSV Logging

Each simulation run logs traffic state to `evaluation/traffic_state.csv` with columns:

```
timestamp, ns_vehicle_count, ns_queue_length, ns_avg_waiting_time, ns_avg_speed,
ew_vehicle_count, ew_queue_length, ew_avg_waiting_time, ew_avg_speed, phase
```

### Future Data Source Compatibility

> The current prototype uses SUMO as the traffic-data source. In the future deployment architecture, this same traffic-state interface can receive metadata generated from CCTV + YOLO + tracking. The `TrafficStateCollector` class provides a clean boundary that isolates the data-collection layer from signal-control decisions.

---

## Project Structure

```
SignalX/
│
├── controller/
│   ├── __init__.py
│   ├── adaptive_controller.py   # Reserved for Part 4+
│   ├── signal_controller.py     # Fixed two-phase signal controller
│   └── traffic_state.py         # TrafficStateCollector (Part 3)
│
├── evaluation/
│   ├── metrics.py               # Reserved for future evaluation
│   └── traffic_state.csv        # Auto-generated traffic state log
│
├── sumo/
│   ├── junction.nod.xml         # Node definitions
│   ├── junction.edg.xml         # Edge definitions
│   ├── network.net.xml          # Generated SUMO network
│   ├── routes.rou.xml           # Traffic demand / routes
│   └── simulation.sumocfg       # SUMO configuration
│
├── main.py                      # Simulation entry point
└── README.md
```

---

## Requirements

- **Python 3.10+**
- **SUMO 1.27.1** (Eclipse SUMO)
- Python packages: `traci`, `sumolib` (included with SUMO or `pip install traci sumolib`)
- `SUMO_HOME` environment variable set to SUMO installation directory

---

## Current Limitations

- Single junction only
- Simulated traffic (no real-world data)
- Fixed signal timing (30s green per phase)
- No adaptive signal control
- No CCTV / camera integration
- No YOLO / vehicle detection
- No reinforcement learning (PPO/MAPPO)
- No MQTT communication
- No emergency vehicle priority
- No multi-junction coordination
- No dashboard or web interface
- No cloud deployment

---

## Next Part

**Part 4** will implement:

```
Traffic State
     ↓
Demand-Based Adaptive Controller
     ↓
Safety Validation
     ↓
Traffic Signal
```

The demand-based controller will dynamically allocate green time according to real-time traffic conditions, replacing the current fixed 30-second green phases.
