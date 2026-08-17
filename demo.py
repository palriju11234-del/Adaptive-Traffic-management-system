"""
SignalX -- Part 7: Final Demonstration Interface

Single entry point for the SIH judges' demonstration.

Demo flow:
    1. Welcome banner
    2. Run fixed + adaptive experiments headless (~3 seconds)
    3. Show performance comparison in console
    4. Generate HTML performance report
    5. Launch SUMO-GUI with adaptive controller
    6. Enhanced real-time console display with decision explanations
    7. Final summary

Usage:
    python demo.py           Full demo (evaluation + SUMO-GUI)
    python demo.py --noeval  Skip evaluation, jump straight to SUMO-GUI
    python demo.py --nogui   Run demo headless (no SUMO-GUI)
"""

import os
import sys
import time
import datetime
import traci

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_ROOT)

from controller.traffic_state import TrafficStateCollector
from controller.signal_controller import FixedSignalController
from controller.adaptive_controller import AdaptiveController
from evaluation.metrics import MetricsCollector

SUMO_CONFIG = os.path.join(PROJECT_ROOT, "sumo", "simulation.sumocfg")
RANDOM_SEED = 42
REPORT_PATH = os.path.join(PROJECT_ROOT, "evaluation", "report.html")

# Map phase names to per-corridor signal display
SIGNAL_LABELS = {
    "NS_GREEN":  ("GREEN",  "RED"),
    "NS_YELLOW": ("YELLOW", "RED"),
    "ALL_RED":   ("RED",    "RED"),
    "EW_GREEN":  ("RED",    "GREEN"),
    "EW_YELLOW": ("RED",    "YELLOW"),
}


# ===================================================================
#  SUMO DETECTION
# ===================================================================

def find_sumo():
    """Locate SUMO binaries."""
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home:
        sumo_bin = os.path.join(sumo_home, "bin", "sumo.exe")
        sumo_gui = os.path.join(sumo_home, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            return sumo_bin, sumo_gui

    for path in [r"C:\Program Files (x86)\Eclipse\Sumo",
                 r"C:\Program Files\Eclipse\Sumo", r"C:\sumo"]:
        sumo_bin = os.path.join(path, "bin", "sumo.exe")
        sumo_gui = os.path.join(path, "bin", "sumo-gui.exe")
        if os.path.isfile(sumo_bin):
            return sumo_bin, sumo_gui

    print("ERROR: SUMO not found. Set SUMO_HOME environment variable.")
    sys.exit(1)


# ===================================================================
#  HEADLESS EXPERIMENT (reused from evaluate.py logic)
# ===================================================================

def run_experiment(mode, seed=RANDOM_SEED):
    """Run a headless simulation and return metrics dict."""
    sumo_bin, _ = find_sumo()
    sumo_cmd = [sumo_bin, "-c", SUMO_CONFIG,
                "--seed", str(seed), "--no-step-log", "true"]

    traci.start(sumo_cmd)
    collector = TrafficStateCollector()
    metrics = MetricsCollector()

    if mode == "fixed":
        controller = FixedSignalController()
    else:
        controller = AdaptiveController(verbose=False)

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            if mode == "fixed":
                controller.step(dt=1.0)
            else:
                controller.step(collector)
            metrics.step()
    except traci.exceptions.FatalTraCIError:
        pass

    results = metrics.get_results()
    try:
        traci.close()
    except Exception:
        pass
    return results


def pct_change(fixed_val, adaptive_val, lower_is_better=True):
    """Calculate percentage improvement string."""
    if fixed_val == 0:
        return "N/A", 0.0
    if lower_is_better:
        pct = ((fixed_val - adaptive_val) / fixed_val) * 100
    else:
        pct = ((adaptive_val - fixed_val) / fixed_val) * 100
    sign = "+" if pct > 0 else ""
    return f"{sign}{pct:.1f}%", pct


# ===================================================================
#  CONSOLE DISPLAY
# ===================================================================

def print_welcome():
    """Print the demo welcome banner."""
    print()
    print("=" * 64)
    print()
    print("     SIGNALX  --  Adaptive Traffic Management System")
    print()
    print("     Smart India Hackathon Prototype Demonstration")
    print()
    print("     Features:")
    print("       * SUMO microscopic traffic simulation")
    print("       * Real-time traffic state via TraCI")
    print("       * Demand-based adaptive signal control")
    print("       * Closed-loop per-cycle re-measurement")
    print("       * Fixed vs Adaptive performance comparison")
    print()
    print("=" * 64)
    print()


def print_comparison_table(fixed, adaptive):
    """Print performance comparison to console."""
    w = 64

    print()
    print("=" * w)
    print("  PERFORMANCE COMPARISON  --  Fixed vs Adaptive")
    print("=" * w)
    print()
    print(f"  {'Metric':<30s} {'Fixed':>10s} {'Adaptive':>10s} {'Change':>10s}")
    print("  " + "-" * (w - 4))

    def row(label, fv, av, lower_is_better=True):
        chg, _ = pct_change(fv, av, lower_is_better)
        print(f"  {label:<30s} {str(fv):>10s} {str(av):>10s} {chg:>10s}")

    row("Avg Waiting Time (s)",
        fixed["avg_waiting_time"], adaptive["avg_waiting_time"])
    row("Max Queue Length",
        fixed["max_queue_length"], adaptive["max_queue_length"])
    row("Avg Queue Length",
        fixed["avg_queue_length"], adaptive["avg_queue_length"])
    row("Throughput (vehicles)",
        fixed["throughput"], adaptive["throughput"], lower_is_better=False)
    row("Simulation Time (s)",
        fixed["total_sim_time"], adaptive["total_sim_time"])

    print()
    print("  " + "-" * (w - 4))
    print("  Per-Corridor:")
    print("  " + "-" * (w - 4))
    row("Avg Queue (NS)",
        fixed["avg_queue_ns"], adaptive["avg_queue_ns"])
    row("Avg Queue (EW)",
        fixed["avg_queue_ew"], adaptive["avg_queue_ew"])

    print()
    wt_s, _ = pct_change(fixed["avg_waiting_time"],
                          adaptive["avg_waiting_time"])
    q_s, _ = pct_change(fixed["max_queue_length"],
                         adaptive["max_queue_length"])
    qns_s, _ = pct_change(fixed["avg_queue_ns"],
                           adaptive["avg_queue_ns"])
    print(f"  Waiting time reduction  : {wt_s}")
    print(f"  Peak queue reduction    : {q_s}")
    print(f"  NS queue reduction      : {qns_s}")
    print()
    print("=" * w)


def print_junction_state(sim_time, state, controller):
    """Print a rich junction state block with decision explanation."""
    ns = state["north_south"]
    ew = state["east_west"]
    phase = controller.get_current_phase_name()
    remaining = controller.get_phase_remaining()
    ns_sig, ew_sig = SIGNAL_LABELS.get(phase, ("?", "?"))

    cycle = controller.cycle_number
    ns_g = controller.ns_green_duration
    ew_g = controller.ew_green_duration

    print()
    print("=" * 64)
    print(f"  JUNCTION C   |   Cycle {cycle}   |   "
          f"t = {sim_time:.0f}s   |   {phase}")
    print("=" * 64)
    print()

    # Two-column junction state
    ns_kmh = round(ns["average_speed"] * 3.6, 1)
    ew_kmh = round(ew["average_speed"] * 3.6, 1)
    col = 34  # width of left column
    print(f"  {'NORTH/SOUTH':<{col}}EAST/WEST")
    print(f"    Vehicles : {ns['vehicle_count']:<{col - 15}}Vehicles : {ew['vehicle_count']}")
    print(f"    Queue    : {ns['queue_length']:<{col - 15}}Queue    : {ew['queue_length']}")
    print(f"    Avg Wait : {str(ns['average_waiting_time']) + 's':<{col - 15}}"
          f"Avg Wait : {ew['average_waiting_time']}s")
    print(f"    Speed    : {str(ns_kmh) + ' km/h':<{col - 15}}"
          f"Speed    : {ew_kmh} km/h")
    print(f"    Signal   : {ns_sig:<{col - 15}}Signal   : {ew_sig}")
    print()

    # Controller state
    print(f"  CONTROLLER")
    print(f"    Mode       : ADAPTIVE (demand-based)")
    print(f"    Phase      : {phase}  ({remaining:.0f}s remaining)")
    print(f"    NS green   : {ns_g} sec")
    print(f"    EW green   : {ew_g} sec")

    # Decision explanation
    action = controller.last_ns_action
    if action:
        ns_v = action["ns_vehicles"]
        ew_v = action["ew_vehicles"]
        total = ns_v + ew_v
        print()
        print(f"  DECISION EXPLANATION")
        print(f"    {action['decision']}")
        if total > 0:
            ns_pct = action["ns_demand_pct"]
            ew_pct = action["ew_demand_pct"]
            print(f"    NS demand: {ns_pct}%  ({ns_v} vehicles)")
            print(f"    EW demand: {ew_pct}%  ({ew_v} vehicles)")
            print()
            print(f"    Formula: green = 20 + (demand/total) x 40")
            print(f"    NS: 20 + ({ns_v}/{total}) x 40 = {20 + (ns_v/total)*40:.1f}"
                  f" -> {action['ns_green']}s")
            print(f"    EW: 20 + ({ew_v}/{total}) x 40 = {20 + (ew_v/total)*40:.1f}"
                  f" -> {action['ew_green']}s")
        else:
            print(f"    No vehicles detected -- using default 30/30")

    print()
    print("=" * 64)


# ===================================================================
#  HTML REPORT
# ===================================================================

def generate_html_report(fixed, adaptive, path):
    """Generate a self-contained HTML performance report."""
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def bar_width(val, max_val):
        if max_val == 0:
            return 5
        return max(5, int((val / max_val) * 100))

    wt_chg, _ = pct_change(fixed["avg_waiting_time"],
                            adaptive["avg_waiting_time"])
    q_chg, _ = pct_change(fixed["max_queue_length"],
                           adaptive["max_queue_length"])
    tp_chg, _ = pct_change(fixed["throughput"],
                            adaptive["throughput"], lower_is_better=False)
    qns_chg, _ = pct_change(fixed["avg_queue_ns"],
                             adaptive["avg_queue_ns"])

    # Chart data
    metrics_chart = [
        ("Avg Wait (s)", fixed["avg_waiting_time"],
         adaptive["avg_waiting_time"]),
        ("Max Queue", fixed["max_queue_length"],
         adaptive["max_queue_length"]),
        ("Avg Queue", fixed["avg_queue_length"],
         adaptive["avg_queue_length"]),
        ("Avg Queue NS", fixed["avg_queue_ns"],
         adaptive["avg_queue_ns"]),
        ("Avg Queue EW", fixed["avg_queue_ew"],
         adaptive["avg_queue_ew"]),
    ]

    chart_html = ""
    for label, fv, av in metrics_chart:
        max_val = max(fv, av, 0.01)
        fw = bar_width(fv, max_val)
        aw = bar_width(av, max_val)
        chart_html += f"""
        <div class="chart-row">
          <div class="chart-label">{label}</div>
          <div class="chart-bars">
            <div class="bar bar-fixed" style="width:{fw}%">{fv}</div>
            <div class="bar bar-adaptive" style="width:{aw}%">{av}</div>
          </div>
        </div>"""

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>SignalX Performance Report</title>
<style>
*{{margin:0;padding:0;box-sizing:border-box}}
body{{font-family:'Segoe UI',Tahoma,Geneva,Verdana,sans-serif;
      background:#0f0f1a;color:#e0e0e0;padding:40px;line-height:1.6}}
.header{{text-align:center;padding:48px 24px;
         background:linear-gradient(135deg,#1a1a2e,#16213e);
         border-radius:16px;margin-bottom:32px;
         border:1px solid #1f3a5c}}
.header h1{{font-size:2.8em;color:#00d4ff;margin-bottom:4px;
            letter-spacing:2px}}
.header p{{color:#8892b0;font-size:1.05em}}
.header .sub{{margin-top:8px;color:#4a5568;font-size:0.9em}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));
       gap:20px;margin-bottom:28px}}
.card{{background:#16213e;border-radius:12px;padding:28px;
       border:1px solid #1f3a5c}}
.card h3{{color:#00d4ff;margin-bottom:16px;font-size:0.85em;
          text-transform:uppercase;letter-spacing:1.5px}}
.mv{{font-size:2.6em;font-weight:700;color:#fff}}
.ml{{font-size:0.85em;color:#8892b0;margin-top:4px}}
.tag{{font-size:0.95em;padding:4px 14px;border-radius:20px;
      display:inline-block;margin-top:10px;font-weight:600}}
.pos{{background:#0d4b3e;color:#4ade80}}
.neg{{background:#4b0d1e;color:#f87171}}
.neu{{background:#3d3d00;color:#fbbf24}}
.chart-row{{display:flex;align-items:center;margin:14px 0}}
.chart-label{{width:120px;font-size:0.85em;color:#8892b0;flex-shrink:0}}
.chart-bars{{flex:1}}
.bar{{height:30px;border-radius:6px;margin:3px 0;display:flex;
      align-items:center;padding:0 14px;font-size:0.82em;
      font-weight:600;min-width:40px;color:#fff}}
.bar-fixed{{background:linear-gradient(90deg,#e94560,#c23152)}}
.bar-adaptive{{background:linear-gradient(90deg,#0f3460,#1a5276)}}
.legend{{display:flex;gap:24px;margin-bottom:16px;font-size:0.85em}}
.legend span{{display:flex;align-items:center;gap:6px}}
.dot{{width:14px;height:14px;border-radius:4px;display:inline-block}}
.dot-f{{background:#e94560}}
.dot-a{{background:#0f3460}}
table{{width:100%;border-collapse:collapse;margin-top:16px}}
th,td{{padding:14px 16px;text-align:left;border-bottom:1px solid #1f3a5c}}
th{{color:#00d4ff;font-size:0.8em;text-transform:uppercase;
    letter-spacing:1px}}
.better{{color:#4ade80;font-weight:600}}
.worse{{color:#f87171}}
.footer{{text-align:center;padding:32px;color:#4a5568;font-size:0.85em;
         margin-top:20px}}
.explain{{background:#1a1a2e;border-radius:8px;padding:20px;
          margin-top:16px;border-left:3px solid #00d4ff}}
code{{background:#0f0f1a;padding:2px 8px;border-radius:4px;
      font-size:0.95em;color:#fbbf24}}
</style>
</head>
<body>

<div class="header">
  <h1>SignalX</h1>
  <p>Adaptive Traffic Management System &mdash; Performance Report</p>
  <p class="sub">Generated: {now} &nbsp;|&nbsp; Seed: {RANDOM_SEED}</p>
</div>

<div class="grid">
  <div class="card">
    <h3>Average Waiting Time</h3>
    <div class="mv">{adaptive["avg_waiting_time"]}s</div>
    <div class="ml">Adaptive Controller</div>
    <span class="tag {"pos" if adaptive["avg_waiting_time"] <= fixed["avg_waiting_time"] else "neg"}">{wt_chg} vs Fixed</span>
    <div class="ml" style="margin-top:14px">Fixed: {fixed["avg_waiting_time"]}s</div>
  </div>
  <div class="card">
    <h3>Max Queue Length</h3>
    <div class="mv">{adaptive["max_queue_length"]}</div>
    <div class="ml">Adaptive Controller</div>
    <span class="tag {"pos" if adaptive["max_queue_length"] <= fixed["max_queue_length"] else "neg"}">{q_chg} vs Fixed</span>
    <div class="ml" style="margin-top:14px">Fixed: {fixed["max_queue_length"]}</div>
  </div>
  <div class="card">
    <h3>Throughput</h3>
    <div class="mv">{adaptive["throughput"]}</div>
    <div class="ml">Vehicles Completed</div>
    <span class="tag {"pos" if adaptive["throughput"] >= fixed["throughput"] else "neg"}">{tp_chg} vs Fixed</span>
    <div class="ml" style="margin-top:14px">Fixed: {fixed["throughput"]}</div>
  </div>
  <div class="card">
    <h3>NS Queue Reduction</h3>
    <div class="mv">{qns_chg}</div>
    <div class="ml">Avg Queue (NS corridor)</div>
    <div class="ml" style="margin-top:14px">Fixed: {fixed["avg_queue_ns"]} &rarr; Adaptive: {adaptive["avg_queue_ns"]}</div>
  </div>
</div>

<div class="card">
  <h3>Fixed vs Adaptive Comparison</h3>
  <div class="legend">
    <span><span class="dot dot-f"></span> Fixed (30s/30s)</span>
    <span><span class="dot dot-a"></span> Adaptive (demand-based)</span>
  </div>
  {chart_html}
</div>

<div class="card" style="margin-top:20px">
  <h3>Detailed Results</h3>
  <table>
    <tr><th>Metric</th><th>Fixed</th><th>Adaptive</th><th>Change</th></tr>
    <tr><td>Avg Waiting Time</td><td>{fixed["avg_waiting_time"]}s</td>
        <td>{adaptive["avg_waiting_time"]}s</td><td>{wt_chg}</td></tr>
    <tr><td>Max Queue Length</td><td>{fixed["max_queue_length"]}</td>
        <td>{adaptive["max_queue_length"]}</td><td>{q_chg}</td></tr>
    <tr><td>Avg Queue Length</td><td>{fixed["avg_queue_length"]}</td>
        <td>{adaptive["avg_queue_length"]}</td>
        <td>{pct_change(fixed["avg_queue_length"], adaptive["avg_queue_length"])[0]}</td></tr>
    <tr><td>Throughput</td><td>{fixed["throughput"]}</td>
        <td>{adaptive["throughput"]}</td><td>{tp_chg}</td></tr>
    <tr><td>Avg Queue (NS)</td><td>{fixed["avg_queue_ns"]}</td>
        <td>{adaptive["avg_queue_ns"]}</td><td>{qns_chg}</td></tr>
    <tr><td>Avg Queue (EW)</td><td>{fixed["avg_queue_ew"]}</td>
        <td>{adaptive["avg_queue_ew"]}</td>
        <td>{pct_change(fixed["avg_queue_ew"], adaptive["avg_queue_ew"])[0]}</td></tr>
    <tr><td>Simulation Time</td><td>{fixed["total_sim_time"]}s</td>
        <td>{adaptive["total_sim_time"]}s</td><td>-</td></tr>
  </table>
</div>

<div class="card" style="margin-top:20px">
  <h3>How It Works</h3>
  <div class="explain">
    <p><strong>Adaptive Algorithm:</strong></p>
    <p style="margin:8px 0">
      <code>green_time = 20 + (direction_demand / total_demand) &times; 40</code>
    </p>
    <p style="margin:8px 0;color:#8892b0">
      The controller measures vehicle counts on approach lanes every cycle.
      The direction with more traffic receives proportionally more green time
      (20&ndash;60 seconds). Yellow (3s) and all-red (1s) safety intervals
      are always preserved.
    </p>
    <p style="margin:12px 0"><strong>Example:</strong></p>
    <p style="color:#8892b0">
      NS = 8 vehicles, EW = 3 vehicles, Total = 11<br>
      NS green = 20 + (8/11) &times; 40 = 49s<br>
      EW green = 20 + (3/11) &times; 40 = 31s
    </p>
  </div>
</div>

<div class="footer">
  SignalX &mdash; Adaptive Traffic Management System<br>
  Smart India Hackathon Prototype
</div>

</body>
</html>"""

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(html)


# ===================================================================
#  LIVE SUMO-GUI DEMO
# ===================================================================

def run_live_demo(use_gui=True):
    """Run the adaptive controller with SUMO-GUI and enhanced display."""
    sumo_bin, sumo_gui_bin = find_sumo()

    if use_gui and os.path.isfile(sumo_gui_bin):
        binary = sumo_gui_bin
    else:
        binary = sumo_bin
        if use_gui:
            print("  WARNING: sumo-gui not found, falling back to headless.")
            use_gui = False

    sumo_cmd = [binary, "-c", SUMO_CONFIG,
                "--seed", str(RANDOM_SEED), "--no-step-log", "true"]
    if use_gui:
        sumo_cmd.append("--start")

    print()
    print("-" * 64)
    print("  Starting SUMO-GUI with Adaptive Controller...")
    print("-" * 64)
    print()

    traci.start(sumo_cmd)

    collector = TrafficStateCollector()
    controller = AdaptiveController(verbose=False)  # demo handles display
    metrics = MetricsCollector()

    last_cycle = 0
    last_phase = ""
    last_display = -15.0
    DISPLAY_INTERVAL = 15  # seconds between periodic state displays

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            sim_time = traci.simulation.getTime()

            phase_name = controller.step(collector)
            metrics.step()

            # Display at key moments
            cycle_changed = controller.cycle_number > last_cycle
            phase_changed = phase_name != last_phase
            periodic = (sim_time - last_display) >= DISPLAY_INTERVAL

            if cycle_changed:
                last_cycle = controller.cycle_number
                state = collector.get_state(signal_phase_name=phase_name)
                print_junction_state(sim_time, state, controller)
                last_display = sim_time

            elif phase_changed:
                remaining = controller._get_current_duration()
                print(f"    [{sim_time:>6.0f}s]  -> {phase_name:<12s}  "
                      f"({remaining:.0f}s)")

            elif periodic and not cycle_changed:
                state = collector.get_state(signal_phase_name=phase_name)
                ns = state["north_south"]
                ew = state["east_west"]
                print(f"    [{sim_time:>6.0f}s]  NS: {ns['vehicle_count']} veh "
                      f"/ {ns['queue_length']} queue   "
                      f"EW: {ew['vehicle_count']} veh "
                      f"/ {ew['queue_length']} queue   "
                      f"Phase: {phase_name}")
                last_display = sim_time

            last_phase = phase_name

    except traci.exceptions.FatalTraCIError:
        print("\n  SUMO connection closed.")
    except KeyboardInterrupt:
        print("\n  Demo interrupted.")

    results = metrics.get_results()

    try:
        traci.close()
    except Exception:
        pass

    print()
    print("-" * 64)
    print(f"  Live demo complete  |  {results['throughput']} vehicles  |  "
          f"{results['total_steps']} steps")
    print("-" * 64)

    return results


# ===================================================================
#  MAIN
# ===================================================================

def main():
    skip_eval = "--noeval" in sys.argv
    no_gui = "--nogui" in sys.argv

    print_welcome()

    fixed_results = None
    adaptive_results = None

    # -- Phase 1: Evaluation (headless) ---------------------------------
    if not skip_eval:
        print("  Running baseline experiments (headless)...")
        print()

        print("    [1/2] Fixed-time controller (30s/30s)...")
        t0 = time.time()
        fixed_results = run_experiment("fixed")
        print(f"          Done in {time.time() - t0:.1f}s  |  "
              f"Throughput: {fixed_results['throughput']}  |  "
              f"Avg wait: {fixed_results['avg_waiting_time']}s")

        print("    [2/2] Adaptive controller (demand-based)...")
        t0 = time.time()
        adaptive_results = run_experiment("adaptive")
        print(f"          Done in {time.time() - t0:.1f}s  |  "
              f"Throughput: {adaptive_results['throughput']}  |  "
              f"Avg wait: {adaptive_results['avg_waiting_time']}s")
        print()

        # Show comparison
        print_comparison_table(fixed_results, adaptive_results)

        # Generate HTML report
        generate_html_report(fixed_results, adaptive_results, REPORT_PATH)
        print()
        print(f"  HTML report saved to: {REPORT_PATH}")
        print()

    # -- Phase 2: Live SUMO-GUI demo ------------------------------------
    if no_gui:
        print("  Skipping SUMO-GUI demo (--nogui flag).")
    else:
        if not skip_eval:
            print()
            print("  " + "=" * 60)
            print("  Press Enter to start the SUMO-GUI demonstration...")
            print("  (The simulation window will open automatically)")
            print("  " + "=" * 60)
            try:
                input("  > ")
            except EOFError:
                pass

        run_live_demo(use_gui=True)

    # -- Final summary --------------------------------------------------
    print()
    print("=" * 64)
    print()
    print("  DEMONSTRATION COMPLETE")
    print()
    if fixed_results and adaptive_results:
        wt_s, _ = pct_change(fixed_results["avg_waiting_time"],
                              adaptive_results["avg_waiting_time"])
        qns_s, _ = pct_change(fixed_results["avg_queue_ns"],
                               adaptive_results["avg_queue_ns"])
        print(f"  Waiting time change : {wt_s}")
        print(f"  NS queue reduction  : {qns_s}")
        print(f"  Report              : {REPORT_PATH}")
    print()
    print("  Commands:")
    print("    python demo.py           Full demonstration")
    print("    python demo.py --nogui   Evaluation only (no GUI)")
    print("    python demo.py --noeval  Skip to SUMO-GUI directly")
    print("    python evaluate.py       Standalone evaluation")
    print("    python main.py --gui     Direct simulation + GUI")
    print()
    print("=" * 64)


if __name__ == "__main__":
    main()
