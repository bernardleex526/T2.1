#!/usr/bin/env bash
# Bounded mapping simulation run: synthetic bag -> lio_node -> recorded map/odom + timing.
#
#   ./run_mapping_sim.sh --run-dir /tmp/sim_map [--extra-config ablation.yaml]
#
# Stages
#   1. compose the effective lio_node config (single source: the launch file's SIM_OVERRIDES)
#   2. generate the bag with the SAME range limits the node will apply (per-point time origin)
#   3. launch bag playback + lio_node (headless) and record map.pcd / map_eval.pcd / odom.tum
#      plus the latency evidence
#   4. write run_manifest.json with commands, hashes and wall times
#
# Concurrency contract: ROS_DOMAIN_ID defaults to 87, ROS_LOCALHOST_ONLY=1, every child PID is
# recorded and killed by PID only (no blanket pkill).  Every stage is wall-clock bounded.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${HERE}/.." && pwd)"
REPO_ROOT="$(cd "${SIM_DIR}/.." && pwd)"

RUN_DIR="/tmp/fastlio2_output"
WORK_DIR=""
BAG_DIR=""
TRAJECTORY="${SIM_DIR}/synthetic_data/test_trajectory.json"
DURATION=""
RATE="1.0"
EXTRA_CONFIG=""
FIT_UNTIL_S="30"
DOMAIN_ID="${SIM_DOMAIN_ID:-87}"
START_DELAY="3.0"
SKIP_BAG="0"
LAUNCH_TIMEOUT_S="600"
RECORD_MARGIN_S="30"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --run-dir) RUN_DIR="$2"; shift 2 ;;
        --work-dir) WORK_DIR="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --trajectory) TRAJECTORY="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --extra-config) EXTRA_CONFIG="$2"; shift 2 ;;
        --fit-until-s) FIT_UNTIL_S="$2"; shift 2 ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --start-delay) START_DELAY="$2"; shift 2 ;;
        --skip-bag) SKIP_BAG="1"; shift ;;
        --launch-timeout-s) LAUNCH_TIMEOUT_S="$2"; shift 2 ;;
        -h|--help) sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

WORK_DIR="${WORK_DIR:-${RUN_DIR}/work}"
BAG_DIR="${BAG_DIR:-${RUN_DIR}/bag}"
MANIFEST="${RUN_DIR}/run_manifest.json"
mkdir -p "${RUN_DIR}" "${WORK_DIR}"

# Explicit ROS overlay (agreed with RuntimeVerification): SIM_INSTALL_SETUP selects the
# install/setup.bash this runner uses; the default is this repository's own install, i.e.
# previous behaviour, and ONLY that file is sourced (never the repo install on top of it).
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
EVAL_START="$(python3 -c "print(float('${T0_EPOCH}') + float('${FIT_UNTIL_S}'))")"

echo "=== mapping sim: run=${RUN_DIR} domain=${DOMAIN_ID} duration=${DURATION}s rate=${RATE} ==="

# ---------------------------------------------------------------- 1. config ----
CONFIG_ARGS=(--work-dir "${WORK_DIR}")
[[ -n "${EXTRA_CONFIG}" ]] && CONFIG_ARGS+=(--extra-config "${EXTRA_CONFIG}")
python3 "${HERE}/sim_config.py" "${CONFIG_ARGS[@]}" --json > "${WORK_DIR}/effective_config.json"
EFF_CONFIG="${WORK_DIR}/lio_sim.yaml"
MIN_RANGE="$(python3 -c "import json;print(json.load(open('${WORK_DIR}/effective_config.json'))['lidar_min_range'])")"
MAX_RANGE="$(python3 -c "import json;print(json.load(open('${WORK_DIR}/effective_config.json'))['lidar_max_range'])")"
echo "effective config ${EFF_CONFIG} (ranges ${MIN_RANGE}..${MAX_RANGE})"

# ------------------------------------------------------------------- 2. bag ----
GEN=(python3 "${SIM_DIR}/synthetic_data/generate_test_bag.py" --out "${BAG_DIR}"
     --trajectory "${TRAJECTORY}" --duration "${DURATION}"
     --min-range "${MIN_RANGE}" --max-range "${MAX_RANGE}")
if [[ "${SKIP_BAG}" == "1" && -d "${BAG_DIR}" ]]; then
    echo "reusing existing bag ${BAG_DIR}"
else
    echo "--- generating bag ---"
    "${GEN[@]}" || { echo "[error] bag generation failed" >&2; exit 1; }
fi

# ---------------------------------------------------------------- 3. launch ----
LAUNCH=(ros2 launch "${SIM_DIR}/launch/sim_fastlio2.launch.py"
        "bag_path:=${BAG_DIR}" "work_dir:=${WORK_DIR}" "config_file:=${EFF_CONFIG}"
        "rviz:=false" "rate:=${RATE}" "start_delay:=${START_DELAY}")
[[ -n "${EXTRA_CONFIG}" ]] && LAUNCH+=("extra_config:=${EXTRA_CONFIG}")

rm -f "${RUN_DIR}/odom.tum" "${RUN_DIR}/map.pcd" "${RUN_DIR}/map_eval.pcd" \
      "${RUN_DIR}/record_run.json" "${RUN_DIR}/latency_evidence.json"
: > "${RUN_DIR}/launch.log"

cleanup() {
    [[ -n "${LAUNCH_PID:-}" ]] && kill "${LAUNCH_PID}" 2>/dev/null || true
    [[ -n "${REC_PID:-}" ]] && kill "${REC_PID}" 2>/dev/null || true
    [[ -n "${LAUNCH_PID:-}" ]] && wait "${LAUNCH_PID}" 2>/dev/null || true
    [[ -n "${REC_PID:-}" ]] && wait "${REC_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "--- launching node + playback (log ${RUN_DIR}/launch.log) ---"
timeout --signal=INT --kill-after=15 "${LAUNCH_TIMEOUT_S}" "${LAUNCH[@]}" >> "${RUN_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!
sleep "${START_DELAY}"
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[error] the launch died immediately; see ${RUN_DIR}/launch.log" >&2
    tail -n 30 "${RUN_DIR}/launch.log" >&2
    exit 1
fi

RECORD_TIMEOUT="$(python3 -c "print(int(float('${DURATION}') / float('${RATE}') + float('${RECORD_MARGIN_S}') + 20))")"
echo "--- recording (timeout ${RECORD_TIMEOUT}s) ---"
timeout --signal=INT --kill-after=15 "${RECORD_TIMEOUT}" python3 "${HERE}/record_run.py" \
    --out-dir "${RUN_DIR}" --duration "$(python3 -c "print(float('${DURATION}') / float('${RATE}') + float('${RECORD_MARGIN_S}'))")" \
    --idle-timeout 12 --voxel 0.05 --eval-start-epoch-s "${EVAL_START}" \
    > "${RUN_DIR}/record.log" 2>&1 &
REC_PID=$!
wait "${REC_PID}"; REC_RC=$?
REC_PID=""
kill "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
LAUNCH_PID=""
echo "recorder exit ${REC_RC}"
tail -n 4 "${RUN_DIR}/record.log"

if [[ ! -s "${RUN_DIR}/odom.tum" ]]; then
    echo "[error] no odometry recorded -- the run is INVALID (see ${RUN_DIR}/launch.log)" >&2
    tail -n 30 "${RUN_DIR}/launch.log" >&2
    exit 1
fi

# --------------------------------------------------------------- 4. manifest ----
# Verify the bag's time contract BEFORE quoting any latency: T2 pairs the observed input arrival
# with the output arrival directly, which is only physical if a complete sweep is published at its
# LAST point's readiness (the header stays the first point's measurement time).  Recorded as
# evidence, not assumed.
python3 "${HERE}/check_bag_availability.py" --bag "${BAG_DIR}" \
    --out "${RUN_DIR}/bag_availability.json" || \
    echo "[warn] bag availability check did not pass; T2's availability basis is unverified"

CMDS=()
CMDS+=("$(printf '%q ' "${GEN[@]}")")
CMDS+=("$(printf '%q ' "${LAUNCH[@]}")")
CMDS+=("python3 ${HERE}/record_run.py --out-dir ${RUN_DIR} --eval-start-epoch-s ${EVAL_START}")
CMDS+=("python3 ${HERE}/check_bag_availability.py --bag ${BAG_DIR} --out ${RUN_DIR}/bag_availability.json")
python3 "${HERE}/run_manifest.py" --out "${MANIFEST}" --stage mapping \
    --set "run_dir=${RUN_DIR}" --set "work_dir=${WORK_DIR}" --set "bag_dir=${BAG_DIR}" \
    --set "trajectory=${TRAJECTORY}" --set "duration_s=${DURATION}" --set "rate=${RATE}" \
    --set "fit_until_s=${FIT_UNTIL_S}" --set "eval_start_epoch_s=${EVAL_START}" \
    --set "domain_id=${DOMAIN_ID}" --set "extra_config=${EXTRA_CONFIG}" \
    --set "realtime_factor_planned=1.0" \
    --hash "trajectory_json=${TRAJECTORY}" --hash "effective_config=${EFF_CONFIG}" \
    --hash "map=${RUN_DIR}/map.pcd" --hash "map_eval=${RUN_DIR}/map_eval.pcd" \
    --hash "odom=${RUN_DIR}/odom.tum" --hash "record_run=${RUN_DIR}/record_run.json" \
    --hash "latency_evidence=${RUN_DIR}/latency_evidence.json" \
    --hash "bag_availability=${RUN_DIR}/bag_availability.json" \
    --add-cmd "${CMDS[0]}" --add-cmd "${CMDS[1]}" --add-cmd "${CMDS[2]}" --add-cmd "${CMDS[3]}"

echo "=== mapping sim done: ${RUN_DIR} ==="
exit 0
