#!/usr/bin/env python3
# BOPT fleet test coordinator for ROS 2 Humble.
# Starts each robot workflow 30 s after the previous robot, allowing overlap.
# This is NOT a safety-rated collision-avoidance system.
import csv, os, signal, subprocess, sys, time
from datetime import datetime
from pathlib import Path

HOME = Path.home()
WS = HOME / "bopt_ws"
SDF = WS / "src/bopt_description/models/pallet/model.sdf"
WORLD = "fg_warehouse_world"
LOGDIR = WS / "logs/fleet_manager"
MAX_CYCLES = None  # Set to an integer for a finite test.
SPAWN_SETTLE = 3.0
REMOVE_SETTLE = 2.0
ROBOT_START_GAP = 35.0  # Between starting robot001, robot002, etc. in each phase.

ROBOTS = [
    dict(name="robot001", pallet="pallet_r001", pickup="Pickup2", drop="Drop8", x=26.5, y=26.36, z=0.05, yaw=0.0),
    dict(name="robot002", pallet="pallet_r002", pickup="Pickup4", drop="Drop6", x=26.5, y=29.82, z=0.05, yaw=0.0),
    dict(name="robot003", pallet="pallet_r003", pickup="Pickup6", drop="Drop4", x=26.5, y=33.30, z=0.05, yaw=0.0),
    dict(name="robot004", pallet="pallet_r004", pickup="Pickup8", drop="Drop2", x=26.5, y=36.60, z=0.05, yaw=0.0),
]
STOP = False
ACTIVE = {}  # robot name -> Popen object
START = time.monotonic()
stats = {r["name"]: dict(spawn_ok=0, spawn_fail=0, pickup_ok=0, pickup_fail=0,
                         drop_ok=0, drop_fail=0, despawn_ok=0, despawn_fail=0,
                         completed_cycles=0) for r in ROBOTS}
csvfile = None
csvwriter = None


def log(s):
    print(f"[{datetime.now().strftime('%F %T')}] {s}", flush=True)


def record(robot, cycle, phase, result, seconds=0, detail=""):
    csvwriter.writerow([datetime.now().isoformat(timespec="seconds"), robot, cycle,
                        phase, result, f"{seconds:.2f}", detail])
    csvfile.flush()


def cmd(args, timeout=30):
    log("$ " + " ".join(map(str, args)))
    try:
        r = subprocess.run(list(map(str, args)), text=True, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, timeout=timeout)
        if r.stdout.strip():
            log(r.stdout.strip()[-1500:])
        return r.returncode, r.stdout or ""
    except subprocess.TimeoutExpired as e:
        out = e.stdout or ""
        if isinstance(out, bytes): out = out.decode(errors="replace")
        log("Command timed out")
        return 124, out


def remove_pallet(robot, cycle, startup=False):
    rc, out = cmd(["ign", "service", "-s", f"/world/{WORLD}/remove",
        "--reqtype", "ignition.msgs.Entity", "--reptype", "ignition.msgs.Boolean",
        "--timeout", "5000", "--req", f'name: "{robot["pallet"]}", type: MODEL'], timeout=8)
    ok = rc == 0 and "data: true" in out.lower()
    if startup and not ok:
        log(f'[{robot["name"]}] No confirmed old pallet removal; assuming absent only for startup.')
        return True
    key = "despawn_ok" if ok else "despawn_fail"
    stats[robot["name"]][key] += 1
    record(robot["name"], cycle, "despawn", "success" if ok else "failed", detail=robot["pallet"])
    if ok: time.sleep(REMOVE_SETTLE)
    return ok


def spawn_pallet(r, cycle):
    if not SDF.is_file():
        log(f"FATAL: SDF not found: {SDF}")
        return False
    rc, out = cmd(["ros2", "run", "ros_gz_sim", "create", "-name", r["pallet"],
        "-file", SDF, "-x", r["x"], "-y", r["y"], "-z", r["z"], "-Y", r["yaw"]], timeout=35)
    ok = rc == 0
    stats[r["name"]]["spawn_ok" if ok else "spawn_fail"] += 1
    record(r["name"], cycle, "spawn", "create_command_ok" if ok else "failed", detail=out[-500:])
    if ok: time.sleep(SPAWN_SETTLE)
    return ok


def workflow_args(r, cycle, action):
    station = r["pickup"] if action == "Pickup" else r["drop"]
    env = os.environ.copy()
    env["ROS_NAMESPACE"] = "/" + r["name"]
    env["ROBOT_ID"] = r["name"]
    args = ["ros2", "run", "workflow_node", "workflow_node", str(cycle), station, action,
            "--ros-args", "-r", "__ns:=/" + r["name"]]
    return station, env, args


def stop_processes():
    """Interrupt all currently running workflow process groups."""
    for name, p in list(ACTIVE.items()):
        if p.poll() is None:
            log(f"Stopping workflow process for {name}")
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
    deadline = time.monotonic() + 5
    for name, p in list(ACTIVE.items()):
        if p.poll() is None:
            try: p.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGTERM)
                except ProcessLookupError: pass
    for name, p in list(ACTIVE.items()):
        if p.poll() is None:
            try: p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                try: os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError: pass


def run_phase(robots, cycle, action):
    """Launch workflows with a 30-second start gap, then collect all results."""
    global ACTIVE
    launched = []
    start_times = {}
    for index, r in enumerate(robots):
        if STOP:
            return False
        # If an already-launched workflow exits unsuccessfully while staggering starts,
        # pause dispatch rather than launching further robots.
        for prior_name, prior_p in list(ACTIVE.items()):
            if prior_p.poll() not in (None, 0):
                log(f"FATAL: {prior_name} exited with code {prior_p.returncode}; stopping new dispatch.")
                stop_processes()
                return False
        station, env, args = workflow_args(r, cycle, action)
        log(f'[{r["name"]}] START {action} at {station}, cycle {cycle}')
        t0 = time.monotonic()
        try:
            p = subprocess.Popen(args, env=env, start_new_session=True)
        except Exception as e:
            log(f'[{r["name"]}] Could not start workflow: {e}')
            stats[r["name"]][action.lower() + "_fail"] += 1
            record(r["name"], cycle, action.lower(), "launch_failed", 0, str(e))
            stop_processes()
            return False
        ACTIVE[r["name"]] = p
        launched.append((r, station, p, t0))
        start_times[r["name"]] = t0
        if index < len(robots) - 1:
            log(f"Waiting {ROBOT_START_GAP:.0f} s before starting the next robot's {action} workflow.")
            end = time.monotonic() + ROBOT_START_GAP
            while not STOP and time.monotonic() < end:
                # Check for a failed workflow while waiting through the stagger interval.
                failed = [(n, q) for n, q in ACTIVE.items() if q.poll() not in (None, 0)]
                if failed:
                    n, q = failed[0]
                    log(f"FATAL: {n} exited with code {q.returncode}; stopping new dispatch.")
                    stop_processes()
                    return False
                time.sleep(min(0.25, max(0.0, end - time.monotonic())))
    if STOP:
        stop_processes()
        return False

    all_ok = True
    for r, station, p, t0 in launched:
        try:
            rc = p.wait()
        except Exception as e:
            log(f'[{r["name"]}] Error waiting for workflow: {e}')
            rc = 1
        elapsed = time.monotonic() - t0
        ok = rc == 0 and not STOP
        key = action.lower() + ("_ok" if ok else "_fail")
        stats[r["name"]][key] += 1
        if action == "Drop" and ok:
            stats[r["name"]]["completed_cycles"] += 1
        record(r["name"], cycle, action.lower(), "process_exit_0" if ok else "failed",
               elapsed, f"station={station}; returncode={rc}")
        log(f'[{r["name"]}] END {action}: rc={rc}, {elapsed:.1f}s')
        ACTIVE.pop(r["name"], None)
        all_ok = all_ok and ok
    return all_ok


def shutdown(sig, frame):
    global STOP
    if STOP: return
    STOP = True
    log(f"Shutdown signal {sig}; no new tasks will be dispatched; interrupting active workflows.")
    stop_processes()
    log("Process termination is NOT a verified emergency stop. Check robots before restart.")


def summary(cycle):
    mins = max((time.monotonic() - START) / 60, 1e-9)
    log(f"SUMMARY after cycle {cycle}")
    for r in ROBOTS:
        log(r["name"] + ": " + ", ".join(f"{k}={v}" for k,v in stats[r["name"]].items()))
    drops = sum(s["drop_ok"] for s in stats.values())
    log(f"Drop workflow process successes={drops}; rate={drops/mins:.2f}/min elapsed={mins:.2f}min")
    log("Counts are process-exit results, NOT independent proof of physical pickup/drop success.")


def main():
    global csvfile, csvwriter
    if not SDF.is_file():
        log(f"FATAL: pallet SDF not found: {SDF}")
        return 2
    LOGDIR.mkdir(parents=True, exist_ok=True)
    logfile = LOGDIR / f"fleet_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    csvfile = logfile.open("w", newline="")
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(["timestamp","robot","cycle","phase","result","elapsed_seconds","detail"])
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    log("BOPT FLEET MANAGER V2: 30-second stagger between robot workflow starts in BOTH phases")
    log("Pickup order: robot001 -> robot002 -> robot003 -> robot004")
    log("Drop order: robot004 -> robot003 -> robot002 -> robot001")
    log("Robots' workflows can overlap after their staggered starts; verify traffic safety before use.")
    log("This does NOT implement route reservations or certified collision avoidance.")
    try:
        cycle = 1
        while not STOP and (MAX_CYCLES is None or cycle <= MAX_CYCLES):
            log(f"===== CYCLE {cycle}: SPAWN ALL PALLETS =====")
            for r in ROBOTS:
                if STOP: break
                if not spawn_pallet(r, cycle):
                    log("FATAL: spawn failed; stopping task dispatch.")
                    summary(cycle)
                    return 1
            if STOP: break
            log(f"===== CYCLE {cycle}: PICKUP PHASE (30-second start gaps) =====")
            if not run_phase(ROBOTS, cycle, "Pickup"):
                log("FATAL: pickup phase failed or was interrupted; fleet paused.")
                summary(cycle)
                return 1
            if STOP: break
            log(f"===== CYCLE {cycle}: DROP PHASE, REVERSE ORDER (30-second start gaps) =====")
            if not run_phase(list(reversed(ROBOTS)), cycle, "Drop"):
                log("FATAL: drop phase failed or was interrupted; fleet paused.")
                summary(cycle)
                return 1
            if STOP: break
            log(f"===== CYCLE {cycle}: DESPAWN ALL PALLETS =====")
            for r in ROBOTS:
                if STOP: break
                if not remove_pallet(r, cycle):
                    log(f'FATAL: could not confirm removal of {r["pallet"]}; fleet paused.')
                    summary(cycle)
                    return 1
            summary(cycle)
            cycle += 1
            time.sleep(2)
    finally:
        if csvfile: csvfile.close()
    log("Exited. Do not restart until robot state and Gazebo entities are checked.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
