#!/usr/bin/env bash
# Bounded SEPARATE localization run: pre-launched consumers -> ready recorders -> bag playback.
#
#   ./run_localization_sim.sh --run-dir /tmp/sim_loc --prior-map /tmp/sim_ref/frozen_map.pcd \
#       --consumer-mode predict --domain-id 137
#
# The localizer is a separate consumer of the mapping stack's live outputs; the prior map is
# frozen (hashed in the manifest) and the trajectory is the separate localization route.  The
# declared initial pose comes from that trajectory's definition (the robot's known placement)
# and is recorded verbatim by record_localization.py.
#
# ORDERING (this is why the runner no longer lets the launch own playback):
#   1. the launch starts with play_bag:=false, so lio_node, the REAL localizer_node, the REAL
#      robot_pose consumer (map_pose_publisher) and the simulation-only body->base_link static
#      TF are ALL up before anything is played;
#   2. the runner waits for that phase to be really ready - nodes, publishers, subscriptions and
#      the localizer's relocalize service as seen in the ROS graph, not a sleep - and records the
#      snapshot to ready_launch.json;
#   3. only then it starts record_localization.py and record_consumer.py and waits for the
#      consumer's own readiness handshake (consumer_ready.json) plus the recorders' subscriptions
#      (ready_recorders.json);
#   4. only then it starts `ros2 bag play` itself, with its PID recorded, so no scan is ever
#      published before a recorder is listening.  The old structure started playback and the
#      recorders at the same instant behind a fixed sleep, which raced the first scans.
#      --start-delay is kept, now measured AFTER the readiness handshake;
#   5. when playback ends the runner creates the shutdown gate, which ends the launch (its
#      bounded gate watchdog also ends it if the runner dies without signalling), waits for every
#      child it started and cleans up BY PID - never a blanket pkill.
#
# consumer_mode (stamp|current|predict) selects the consumer semantics configured in the launch;
# the bag is identical for all three, so pass --skip-bag to run another mode on the same bag.
# --fault starts sim_fault_inject.py against this very run (see that script); the default is none,
# i.e. no fault is injected and the run is a plain three-mode-capable localisation run.
#
# Concurrency contract: ROS_DOMAIN_ID defaults to 87, ROS_LOCALHOST_ONLY=1, PID-scoped cleanup.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${HERE}/.." && pwd)"
REPO_ROOT="$(cd "${SIM_DIR}/.." && pwd)"

RUN_DIR="/tmp/fastlio2_localization"
WORK_DIR=""
BAG_DIR=""
TRAJECTORY="${SIM_DIR}/synthetic_data/test_trajectory_localization.json"
MAPPING_TRAJECTORY="${SIM_DIR}/synthetic_data/test_trajectory.json"
PRIOR_MAP=""
DURATION=""
RATE="1.0"
DOMAIN_ID="${SIM_DOMAIN_ID:-87}"
START_DELAY="3.0"
SKIP_BAG="0"
LAUNCH_TIMEOUT_S="600"
RECORD_MARGIN_S="30"
CONSUMER_MODE="stamp"
READY_TIMEOUT_S="240"
FAULT="none"
FAULT_AT_S="30.0"
FAULT_HOLD_S="5.0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-dir) RUN_DIR="$2"; shift 2 ;;
        --work-dir) WORK_DIR="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --trajectory) TRAJECTORY="$2"; shift 2 ;;
        --prior-map) PRIOR_MAP="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --start-delay) START_DELAY="$2"; shift 2 ;;
        --consumer-mode) CONSUMER_MODE="$2"; shift 2 ;;
        --ready-timeout-s) READY_TIMEOUT_S="$2"; shift 2 ;;
        --fault) FAULT="$2"; shift 2 ;;
        --fault-at-s) FAULT_AT_S="$2"; shift 2 ;;
        --fault-hold-s) FAULT_HOLD_S="$2"; shift 2 ;;
        --skip-bag) SKIP_BAG="1"; shift ;;
        --launch-timeout-s) LAUNCH_TIMEOUT_S="$2"; shift 2 ;;
        -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

case "${CONSUMER_MODE}" in
    stamp|current|predict) ;;
    *) echo "[error] --consumer-mode must be stamp|current|predict (got '${CONSUMER_MODE}')" >&2
       exit 2 ;;
esac
case "${FAULT}" in
    none|stop_localizer|stop_odom|clock_pause|clock_jumpback) ;;
    *) echo "[error] --fault must be none|stop_localizer|stop_odom|clock_pause|clock_jumpback " \
            "(got '${FAULT}')" >&2; exit 2 ;;
esac

WORK_DIR="${WORK_DIR:-${RUN_DIR}/work}"
BAG_DIR="${BAG_DIR:-${RUN_DIR}/bag}"
MANIFEST="${RUN_DIR}/run_manifest.json"
mkdir -p "${RUN_DIR}" "${WORK_DIR}"
[[ -n "${PRIOR_MAP}" ]] || { echo "[error] --prior-map is required (the frozen PCD)" >&2; exit 2; }
[[ -f "${PRIOR_MAP}" ]] || { echo "[error] frozen prior map not found: ${PRIOR_MAP}" >&2; exit 2; }

# The ROS overlay is explicit: SIM_INSTALL_SETUP selects the install/setup.bash to run (default:
# this repository's own install, i.e. previous behaviour).  A build that lives elsewhere (e.g. a
# recovery workspace) is selected by exporting SIM_INSTALL_SETUP; nothing else is sourced, so no
# other install can silently take precedence.
SIM_INSTALL_SETUP="${SIM_INSTALL_SETUP:-${REPO_ROOT}/install/setup.bash}"
[[ -f "${SIM_INSTALL_SETUP}" ]] || { echo "[error] ROS overlay setup not found: ${SIM_INSTALL_SETUP} (export SIM_INSTALL_SETUP=<install/setup.bash>)" >&2; exit 2; }
export SIM_INSTALL_SETUP
set +u
source /opt/ros/humble/setup.bash
source "${SIM_INSTALL_SETUP}"
set -u
echo "ROS overlay: ${SIM_INSTALL_SETUP}"
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export ROS_LOCALHOST_ONLY=1

if [[ -z "${DURATION}" ]]; then
    DURATION="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['duration_s'])" "${TRAJECTORY}")"
fi
T0_EPOCH="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['t0_epoch_s'])" "${TRAJECTORY}")"
INITIAL_POSE="$(python3 - "${TRAJECTORY}" <<'PY'
import json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(sys.argv[1])), "."))
import traj_common as tc
traj = tc.load_trajectory(sys.argv[1])
gt = tc.build_ground_truth(traj, duration_s=1.0)
R = gt["rot"][0]
yaw = math.atan2(R[1, 0], R[0, 0])
x, y, z = traj["body"]["start_xyz"]
print("%.6f,%.6f,%.6f,0.0,0.0,%.6f" % (x, y, z, yaw))
PY
)"
echo "=== localization sim: run=${RUN_DIR} domain=${DOMAIN_ID} duration=${DURATION}s rate=${RATE} ==="
echo "consumer_mode=${CONSUMER_MODE} fault=${FAULT} bag=${BAG_DIR}"
echo "declared initial pose (trajectory start placement): ${INITIAL_POSE}"
echo "frozen prior map: ${PRIOR_MAP} sha256=$(sha256sum "${PRIOR_MAP}" | cut -d' ' -f1)"

# ---------------------------------------------------------------- 1. configs ----
python3 "${HERE}/sim_config.py" --work-dir "${WORK_DIR}" --json > "${WORK_DIR}/effective_config.json"
EFF_CONFIG="${WORK_DIR}/lio_sim.yaml"
MIN_RANGE="$(python3 -c "import json;print(json.load(open('${WORK_DIR}/effective_config.json'))['lidar_min_range'])")"
MAX_RANGE="$(python3 -c "import json;print(json.load(open('${WORK_DIR}/effective_config.json'))['lidar_max_range'])")"
LOC_CONFIG="${WORK_DIR}/loc_sim.yaml"
python3 "${HERE}/sim_localizer_config.py" --out "${LOC_CONFIG}" --pcd "${PRIOR_MAP}"

# ------------------------------------------------------------------- 2. bag ----
GEN=(python3 "${SIM_DIR}/synthetic_data/generate_test_bag.py" --out "${BAG_DIR}"
     --trajectory "${TRAJECTORY}" --duration "${DURATION}"
     --min-range "${MIN_RANGE}" --max-range "${MAX_RANGE}")
if [[ "${SKIP_BAG}" == "1" && -d "${BAG_DIR}" ]]; then
    echo "reusing existing bag ${BAG_DIR} (same bag for every consumer mode)"
else
    echo "--- generating localization bag ---"
    "${GEN[@]}" || { echo "[error] bag generation failed" >&2; exit 1; }
fi

# ------------------------------------------------------ 3. paths, PIDs, cleanup ----
PLAY_LOG="${RUN_DIR}/bag_play.log"
RECORD_TIMEOUT="$(python3 -c "print(int(float('${DURATION}') / float('${RATE}') + float('${RECORD_MARGIN_S}') + 20))")"
RECORD_SECONDS="$(python3 -c "print(float('${DURATION}') / float('${RATE}') + float('${RECORD_MARGIN_S}'))")"
PLAY_LIMIT="$(python3 -c "print(int(float('${DURATION}') / float('${RATE}') + float('${FAULT_HOLD_S}') + 60))")"
# The launch's own gate watchdog must outlive the WHOLE run: the readiness wait (which can take
# READY_TIMEOUT_S), playback, and the teardown margin.  Otherwise the launch would kill itself
# while the runner is still legitimately waiting for readiness (observed: 185 s gate vs 240 s
# readiness wait before this fix).
GATE_TIMEOUT="$(python3 -c "print(int(float('${READY_TIMEOUT_S}') + float('${DURATION}') / float('${RATE}') + float('${FAULT_HOLD_S}') + 120))")"
CONSUMER_READY="${RUN_DIR}/consumer_ready.json"
READY_LAUNCH="${RUN_DIR}/ready_launch.json"
READY_RECORDERS="${RUN_DIR}/ready_recorders.json"
PIDS_JSON="${RUN_DIR}/pids.json"
GATE_FILE="${RUN_DIR}/run_complete.gate"
BAG_PID_FILE="${RUN_DIR}/bag_play.pid"

rm -f "${RUN_DIR}/localization.tum" "${RUN_DIR}/localization_record.json" "${RUN_DIR}/tf_availability.json" \
      "${RUN_DIR}/consumer_queries.json" "${RUN_DIR}/consumer_status.jsonl" "${CONSUMER_READY}" \
      "${READY_LAUNCH}" "${READY_RECORDERS}" "${PIDS_JSON}" "${GATE_FILE}" "${BAG_PID_FILE}" \
      "${RUN_DIR}/fault.log"
: > "${RUN_DIR}/launch.log"
: > "${PLAY_LOG}"

LAUNCH_PID=""
REC_PID=""
CONS_PID=""
BAG_PID=""
FAULT_PID=""
LOC_NODE_PID=""
LIO_NODE_PID=""

cleanup() {
    local pids=() pid v alive
    for v in FAULT_PID BAG_PID REC_PID CONS_PID LAUNCH_PID; do
        pid="${!v:-}"
        [[ -n "${pid}" ]] && pids+=("${pid}")
    done
    if [[ -s "${BAG_PID_FILE}" ]]; then
        read -r pid < "${BAG_PID_FILE}" || true
        [[ -n "${pid:-}" ]] && pids+=("${pid}")
    fi
    [[ ${#pids[@]} -eq 0 ]] && return 0
    for pid in "${pids[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
    for _ in $(seq 1 20); do
        alive=0
        for pid in "${pids[@]}"; do kill -0 "${pid}" 2>/dev/null && alive=1; done
        [[ "${alive}" == "0" ]] && break
        sleep 0.5
    done
    for pid in "${pids[@]}"; do
        kill -0 "${pid}" 2>/dev/null && kill -KILL "${pid}" 2>/dev/null || true
    done
    return 0
}
trap cleanup EXIT

# Wait for PID $1, but SIGINT it after $2 seconds (the process is a child of THIS shell, so a
# plain `wait` in a command substitution could not observe it).  Result in WAITED_RC.
WAITED_RC=0
wait_with_deadline() {
    local pid="$1" limit="$2" watchdog
    WAITED_RC=0
    (
        sleep "${limit}" 2>/dev/null
        if kill -0 "${pid}" 2>/dev/null; then
            echo "[watchdog] pid ${pid} still alive after ${limit}s; SIGINT" >&2
            kill -INT "${pid}" 2>/dev/null || true
        fi
    ) &
    watchdog=$!
    wait "${pid}" 2>/dev/null
    WAITED_RC=$?
    kill "${watchdog}" 2>/dev/null || true
    wait "${watchdog}" 2>/dev/null || true
}

# Wait until $1 exists, up to $2 seconds; also fail fast if process $3 died meanwhile.
wait_for_file() {
    local path="$1" limit="$2" guard="${3:-}" waited=0
    while [[ ! -e "${path}" ]]; do
        if [[ -n "${guard}" ]] && ! kill -0 "${guard}" 2>/dev/null; then
            return 2
        fi
        waited=$((waited + 1))
        [[ "${waited}" -gt "$((limit * 2))" ]] && return 1
        sleep 0.5
    done
    return 0
}

# Readiness probe: the ROS graph itself (nodes, publishers, subscriptions, services) decides
# whether a phase is ready.  Writes the snapshot and the per-node PIDs to JSON.  $3 is an
# optional GUARD pid (the launch): the wait aborts as soon as it is gone, so a launch that died
# is reported as guard_gone instead of as an empty graph after a long timeout, and the CLOSEST
# snapshot seen (fewest unmet requirements) is the one written.
run_ready_probe() {
    python3 - "$1" "${PIDS_JSON}" "${READY_TIMEOUT_S}" "${DOMAIN_ID}" "$2" "${CONSUMER_MODE}" "${3:-}" <<'PY'
import json, os, sys, time

out_path, pids_path, timeout_s, domain, phase, consumer_mode, guard = sys.argv[1:8]
timeout_s = float(timeout_s)

REQUIREMENTS = {
    "launch": {
        "nodes": ["lio_node", "localizer_node", "map_pose_publisher"],
        "publishers": {"/fastlio2/body_cloud": 1, "/fastlio2/lio_odom": 1, "/robot_pose_map": 1,
                       "/robot_pose_map/status": 1, "/tf_static": 1},
        # The localizer only BROADCASTS TF; the consumer owns the transform listener - and tf2
        # runs that listener in a node OF ITS OWN named "transform_listener_impl_<id>" (measured
        # on this Humble install), so the /tf endpoints can NEVER be attributed to
        # "map_pose_publisher" by name.  The requirement is therefore "a tf2 listener
        # subscription exists on /tf and /tf_static" (any_prefix), while the consumer NODE is
        # required separately above; in this launch the consumer is the only transform listener.
        "subscribers": {
            "/fastlio2/body_cloud": {"nodes": ["localizer_node"]},
            "/fastlio2/lio_odom": {"nodes": ["localizer_node"]},
            "/tf": {"any_prefix": ["transform_listener_impl_"]},
            "/tf_static": {"any_prefix": ["transform_listener_impl_"]},
        },
        "services": ["/localizer/relocalize", "/localizer/relocalize_check"],
    },
    "recorders": {
        "nodes": ["sim_localization_recorder", "sim_consumer_recorder"],
        "publishers": {},
        # both recorders subscribe /tf and /clock DIRECTLY (rclpy, not tf2), so here the exact
        # node names are the right requirement
        "subscribers": {
            "/robot_pose_map": {"nodes": ["sim_consumer_recorder"]},
            "/robot_pose_map/status": {"nodes": ["sim_consumer_recorder"]},
            "/fastlio2/lio_odom": {"nodes": ["sim_localization_recorder"]},
            "/tf": {"nodes": ["sim_localization_recorder", "sim_consumer_recorder"]},
            "/clock": {"nodes": ["sim_localization_recorder", "sim_consumer_recorder"]},
        },
        "services": [],
    },
}
req = json.loads(json.dumps(REQUIREMENTS[phase]))        # deep copy: the mode tweak below
if phase == "launch" and consumer_mode != "stamp":
    # map_pose_publisher creates its odometry subscription ONLY in current/predict mode, so this
    # check also proves the consumer really came up in the mode the runner asked for.
    req["subscribers"]["/fastlio2/lio_odom"]["nodes"].append("map_pose_publisher")
NODE_EXECUTABLES = ("lio_node", "localizer_node", "map_pose_publisher")


def unmet(rule, names):
    """Requirements of one topic that the observed subscriber node names do not cover."""
    missing = [n for n in rule.get("nodes", []) if n not in names]
    for prefix in rule.get("any_prefix", []):
        if not any(n.startswith(prefix) for n in names):
            missing.append(prefix + "*")
    return missing




def find_pids():
    """PIDs of our nodes: cmdline basename match, restricted to THIS ROS_DOMAIN_ID."""
    found = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/environ" % entry, "rb") as fh:
                environ = fh.read().decode("utf-8", "replace")
            if ("ROS_DOMAIN_ID=%s\0" % domain) not in environ:
                continue
            with open("/proc/%s/cmdline" % entry, "rb") as fh:
                argv0 = fh.read().decode("utf-8", "replace").split("\0")[0]
        except OSError:
            continue
        base = os.path.basename(argv0)
        if base in NODE_EXECUTABLES:
            found[base] = int(entry)
    return found


import rclpy                                          # noqa: E402  (after the stdlib-only scan)
from rclpy.node import Node                           # noqa: E402

rclpy.init()
node = Node("sim_ready_probe")
started = time.monotonic()
best = None
best_unmet = None
guard_gone = False
try:
    while True:
        node_names = sorted({n for n, _ns in node.get_node_names_and_namespaces()})
        subs = {t: sorted({e.node_name for e in node.get_subscriptions_info_by_topic(t)})
                for t in req["subscribers"]}
        pubs = {t: len(node.get_publishers_info_by_topic(t)) for t in req["publishers"]}
        services = sorted({s for s, _t in node.get_service_names_and_types()})
        missing_nodes = [n for n in req["nodes"] if n not in node_names]
        missing_subs = {t: unmet(rule, subs[t]) for t, rule in req["subscribers"].items()
                        if unmet(rule, subs[t])}
        missing_pubs = {t: {"have": pubs[t], "need": need}
                        for t, need in req["publishers"].items() if pubs[t] < need}
        missing_services = [s for s in req["services"] if s not in services]
        snapshot = {
            "phase": phase, "domain_id": domain, "elapsed_s": round(time.monotonic() - started, 3),
            "nodes": node_names, "subscribers": subs, "publishers": pubs,
            "tf_listener_nodes": sorted({n for n in subs.get("/tf", [])
                                         if n.startswith("transform_listener_impl_")}),
            "services_present": [s for s in req["services"] if s in services],
            "missing_nodes": missing_nodes, "missing_subscribers": missing_subs,
            "missing_publishers": missing_pubs, "missing_services": missing_services,
            "pids": find_pids(),
            "timeout_s": timeout_s, "guard_pid": int(guard) if guard else None,
            "requirements": req,
        }
        unmet_count = (len(missing_nodes) + len(missing_subs) + len(missing_pubs)
                       + len(missing_services))
        if best is None or unmet_count < best_unmet:
            best, best_unmet = snapshot, unmet_count
        if unmet_count == 0:
            break
        if guard and not os.path.isdir("/proc/%s" % guard):
            guard_gone = True
            break
        if time.monotonic() - started > timeout_s:
            break
        time.sleep(0.5)
finally:
    node.destroy_node()
    rclpy.shutdown()

ok = best is not None and best_unmet == 0
snapshot = dict(best or {})
snapshot.update({
    "ok": ok, "guard_gone": guard_gone, "unmet_requirements": best_unmet,
    "settled_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "note": "the written snapshot is the CLOSEST one seen (fewest unmet requirements), not "
            "necessarily the last poll: if guard_gone is true the launch died while waiting",
})
if os.path.isfile(pids_path):
    try:
        with open(pids_path, "r") as fh:
            merged = json.load(fh)
    except ValueError:
        merged = {}
else:
    merged = {}
merged.update({"phase_%s" % phase: snapshot.get("pids") or {},
               "domain_id": domain,
               "updated_utc": snapshot["settled_utc"]})
for name, pid in (snapshot.get("pids") or {}).items():
    merged[name] = pid
with open(pids_path, "w") as fh:
    json.dump(merged, fh, indent=2, sort_keys=True)

with open(out_path, "w") as fh:
    json.dump(snapshot, fh, indent=2, sort_keys=True)
print("[ready:%s] ok=%s elapsed=%.1fs guard_gone=%s tf_listener=%s nodes=%s%s" % (
    phase, ok, snapshot.get("elapsed_s"), guard_gone, snapshot.get("tf_listener_nodes"),
    ",".join(snapshot.get("nodes") or []),
    "" if ok else " UNMET=%s" % json.dumps({k: snapshot.get(k) for k in
                                            ("missing_nodes", "missing_subscribers",
                                             "missing_publishers", "missing_services")})))
sys.exit(0 if ok else 1)
PY
}

# ------------------------------------------------------------------ 4. launch ----
LAUNCH=(ros2 launch "${SIM_DIR}/launch/sim_localization.launch.py"
        "bag_path:=${BAG_DIR}" "work_dir:=${WORK_DIR}" "config_file:=${EFF_CONFIG}"
        "rate:=${RATE}" "start_delay:=${START_DELAY}"
        "consumer_mode:=${CONSUMER_MODE}"
        "play_bag:=false" "shutdown_gate:=${GATE_FILE}" "shutdown_gate_timeout_s:=${GATE_TIMEOUT}"
        "localizer_config:=${LOC_CONFIG}" "prior_map:=${PRIOR_MAP}")
echo "--- launching consumers + static TF + localizer without playback (log ${RUN_DIR}/launch.log) ---"
timeout --signal=INT --kill-after=15 "${LAUNCH_TIMEOUT_S}" "${LAUNCH[@]}" >> "${RUN_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

if ! run_ready_probe "${READY_LAUNCH}" launch "${LAUNCH_PID}"; then
    echo "[error] the launch phase never became ready within ${READY_TIMEOUT_S}s (or the launch" \
         "died while waiting); see ${READY_LAUNCH} (unmet requirements + guard_gone) and" \
         "${RUN_DIR}/launch.log" >&2
    tail -n 40 "${RUN_DIR}/launch.log" >&2
    exit 1
fi
LOC_NODE_PID="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('localizer_node',''))" "${PIDS_JSON}" 2>/dev/null)"
LIO_NODE_PID="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('lio_node',''))" "${PIDS_JSON}" 2>/dev/null)"
echo "node PIDs (ours, domain ${DOMAIN_ID}): lio_node=${LIO_NODE_PID:-?} localizer_node=${LOC_NODE_PID:-?}"

# --------------------------------------------------------------- 5. recorders ----
echo "--- starting the recorders BEFORE playback (timeout ${RECORD_TIMEOUT}s, duration ${RECORD_SECONDS}s) ---"
timeout --signal=INT --kill-after=5 "${RECORD_TIMEOUT}" python3 "${HERE}/record_localization.py" \
    --out-dir "${RUN_DIR}" --duration "${RECORD_SECONDS}" \
    --idle-timeout 12 --prior-map "${PRIOR_MAP}" --initial-pose "${INITIAL_POSE}" \
    > "${RUN_DIR}/record.log" 2>&1 &
REC_PID=$!
timeout --signal=INT --kill-after=5 "${RECORD_TIMEOUT}" python3 "${HERE}/record_consumer.py" \
    --out-dir "${RUN_DIR}" --duration "${RECORD_SECONDS}" --pose-mode "${CONSUMER_MODE}" \
    > "${RUN_DIR}/consumer.log" 2>&1 &
CONS_PID=$!

if ! wait_for_file "${CONSUMER_READY}" 60 "${CONS_PID}"; then
    echo "[error] record_consumer.py never wrote ${CONSUMER_READY} (rc=${?}); see" \
         "${RUN_DIR}/consumer.log" >&2
    tail -n 20 "${RUN_DIR}/consumer.log" >&2
    exit 1
fi
if ! run_ready_probe "${READY_RECORDERS}" recorders "${LAUNCH_PID}"; then
    echo "[error] the recorders never appeared with their subscriptions; see" \
         "${READY_RECORDERS}" >&2
    tail -n 20 "${RUN_DIR}/consumer.log" >&2
    exit 1
fi
kill -0 "${REC_PID}" 2>/dev/null || { echo "[error] record_localization.py died before playback" >&2
    tail -n 20 "${RUN_DIR}/record.log" >&2; exit 1; }
kill -0 "${CONS_PID}" 2>/dev/null || { echo "[error] record_consumer.py died before playback" >&2
    tail -n 20 "${RUN_DIR}/consumer.log" >&2; exit 1; }

# ----------------------------------------------------------------- 6. playback ----
# start_delay is kept: it is now the settling time AFTER the readiness handshake, so playback can
# never start before every consumer and recorder is subscribed.
sleep "${START_DELAY}"
echo "--- runner-owned playback: ros2 bag play ${BAG_DIR} (log ${PLAY_LOG}) ---"
ros2 bag play "${BAG_DIR}" --clock --rate "${RATE}" --disable-keyboard-controls >> "${PLAY_LOG}" 2>&1 &
BAG_PID=$!
echo "${BAG_PID}" > "${BAG_PID_FILE}"

if [[ "${FAULT}" != "none" ]]; then
    echo "--- fault injection ARMED: ${FAULT} at bag t=${FAULT_AT_S}s (hold ${FAULT_HOLD_S}s) ---"
    python3 "${HERE}/sim_fault_inject.py" --fault "${FAULT}" \
        --target-clock-s "$(python3 -c "print(float('${T0_EPOCH}') + float('${FAULT_AT_S}'))")" \
        --hold-s "${FAULT_HOLD_S}" --pids-json "${PIDS_JSON}" --bag-pid-file "${BAG_PID_FILE}" \
        --bag-dir "${BAG_DIR}" --bag-t0-s "${T0_EPOCH}" --rate "${RATE}" --play-log "${PLAY_LOG}" \
        --log "${RUN_DIR}/fault.log" \
        >> "${RUN_DIR}/fault_stdout.log" 2>&1 &
    FAULT_PID=$!
fi

wait_with_deadline "${BAG_PID}" "${PLAY_LIMIT}"; PLAY_RC=${WAITED_RC}
BAG_PID=""
# a clock_jumpback fault restarts playback from an earlier offset: the restarted player is a child
# of the injector, so the injector (a child of this shell) is what ends that extended playback.
if [[ -n "${FAULT_PID}" ]] && kill -0 "${FAULT_PID}" 2>/dev/null; then
    echo "--- fault-restarted playback still running; waiting for the injector ---"
    wait_with_deadline "${FAULT_PID}" "${PLAY_LIMIT}"; PLAY_RC=${WAITED_RC}
fi
FAULT_PID=""
: > "${BAG_PID_FILE}"
echo "playback exit ${PLAY_RC}"

# ------------------------------------------------------------------ 7. teardown ----
: > "${GATE_FILE}"
echo "--- signalling the launch through ${GATE_FILE} ---"
wait_with_deadline "${LAUNCH_PID}" 60; LAUNCH_RC=${WAITED_RC}
LAUNCH_PID=""
echo "launch exit ${LAUNCH_RC} (gate watchdog in ${RUN_DIR}/launch.log)"

wait_with_deadline "${REC_PID}" "$((RECORD_MARGIN_S + 60))"; REC_RC=${WAITED_RC}
REC_PID=""
wait_with_deadline "${CONS_PID}" "$((RECORD_MARGIN_S + 60))"; CONS_RC=${WAITED_RC}
CONS_PID=""
echo "recorder exits: localization=${REC_RC} consumer=${CONS_RC}"
tail -n 4 "${RUN_DIR}/record.log"
tail -n 4 "${RUN_DIR}/consumer.log"

if [[ ! -s "${RUN_DIR}/localization.tum" ]]; then
    echo "[error] no localization poses recorded -- the run is INVALID (see ${RUN_DIR}/launch.log)" >&2
    tail -n 40 "${RUN_DIR}/launch.log" >&2
    exit 1
fi
if [[ ! -s "${RUN_DIR}/consumer_queries.json" ]]; then
    echo "[error] the consumer recorder wrote no census -- the run is INVALID" >&2
    exit 1
fi

# --------------------------------------------------------------- 8. manifest ----
python3 "${HERE}/run_manifest.py" --out "${MANIFEST}" --stage localization \
    --set "run_dir=${RUN_DIR}" --set "work_dir=${WORK_DIR}" --set "bag_dir=${BAG_DIR}" \
    --set "trajectory=${TRAJECTORY}" --set "mapping_trajectory=${MAPPING_TRAJECTORY}" \
    --set "duration_s=${DURATION}" --set "rate=${RATE}" --set "domain_id=${DOMAIN_ID}" \
    --set "consumer_mode=${CONSUMER_MODE}" \
    --set "consumer_publisher_params={\"current_pose_mode\": stamp=false/current=true/predict=true, \"predict_current_pose\": stamp=false/current=false/predict=true, \"use_sim_time\": true}" \
    --set "consumer_node=robot_pose/map_pose_publisher (the real consumer, one instance)" \
    --set "playback_owner=run_localization_sim.sh (ros2 bag play after the readiness handshake)" \
    --set "ready_timeout_s=${READY_TIMEOUT_S}" --set "start_delay_s=${START_DELAY}" \
    --set "record_seconds=${RECORD_SECONDS}" --set "record_margin_s=${RECORD_MARGIN_S}" \
    --set "fault=${FAULT}" --set "fault_at_s=${FAULT_AT_S}" --set "fault_hold_s=${FAULT_HOLD_S}" \
    --set "recorder_exit_codes={\"localization\": ${REC_RC}, \"consumer\": ${CONS_RC}}" \
    --set "declared_initial_pose=${INITIAL_POSE}" \
    --set "initial_pose_source=trajectory definition start placement (declared; GT-assisted)" \
    --hash "trajectory_json=${TRAJECTORY}" --hash "effective_config=${EFF_CONFIG}" \
    --hash "localizer_config=${LOC_CONFIG}" --hash "prior_map=${PRIOR_MAP}" \
    --hash "localization_tum=${RUN_DIR}/localization.tum" \
    --hash "localization_record=${RUN_DIR}/localization_record.json" \
    --hash "tf_availability=${RUN_DIR}/tf_availability.json" \
    --hash "consumer_queries=${RUN_DIR}/consumer_queries.json" \
    --hash "consumer_status=${RUN_DIR}/consumer_status.jsonl" \
    --hash "consumer_ready=${CONSUMER_READY}" \
    --hash "ready_launch=${READY_LAUNCH}" --hash "ready_recorders=${READY_RECORDERS}" \
    --hash "pids=${PIDS_JSON}" --hash "bag_play_log=${PLAY_LOG}" \
    --hash "fault_log=${RUN_DIR}/fault.log" \
    --add-cmd "$(printf '%q ' "${GEN[@]}")" \
    --add-cmd "$(printf '%q ' "${LAUNCH[@]}")" \
    --add-cmd "python3 ${HERE}/record_localization.py --out-dir ${RUN_DIR} --duration ${RECORD_SECONDS} --prior-map ${PRIOR_MAP} --initial-pose ${INITIAL_POSE}" \
    --add-cmd "python3 ${HERE}/record_consumer.py --out-dir ${RUN_DIR} --duration ${RECORD_SECONDS} --pose-mode ${CONSUMER_MODE}" \
    --add-cmd "ros2 bag play ${BAG_DIR} --clock --rate ${RATE} --disable-keyboard-controls" \
    --note "the localizer is a separate consumer; the frozen prior map is never updated" \
    --note "the consumer is the real robot_pose map_pose_publisher; only ONE instance is started (by the launch)" \
    --note "playback starts only after the launch phase and both recorders reported ready (ready_*.json, consumer_ready.json)"

echo "=== localization sim done: ${RUN_DIR} (consumer_mode=${CONSUMER_MODE}) ==="
exit 0
