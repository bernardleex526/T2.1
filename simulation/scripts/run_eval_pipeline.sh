#!/usr/bin/env bash
# End-to-end synthetic SLAM pipeline: scene reference -> mapping run -> T1/T2 ->
# frozen map -> SEPARATE localization run -> T3.
#
#   ./run_eval_pipeline.sh                                  # baseline only
#   ./run_eval_pipeline.sh --variants baseline,c2_1,c2_2,c2_combined
#   ./run_eval_pipeline.sh --consumer-mode predict          # consumer semantics for T3
#   ./run_eval_pipeline.sh --dry-run
#
# Every variant runs SEQUENTIALLY (never concurrently: T2 is a latency measurement) and every
# stage is wall-clock bounded.  Nothing here touches an external dataset: the reference
# geometry, the ROI, the ground truth and the prior map are all generated from the synthetic
# scene definition.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${HERE}/.." && pwd)"
REPO_ROOT="$(cd "${SIM_DIR}/.." && pwd)"

OUT_ROOT="${SIM_OUT_ROOT:-/tmp/sim_pipeline}"
REF_DIR=""
FIT_UNTIL_S="30"
RATE="1.0"
DOMAIN_ID="${SIM_DOMAIN_ID:-87}"
VARIANTS="baseline"
SKIP_MAPPING="0"
SKIP_LOCALIZATION="0"
DRY_RUN="0"
WITH_TRAJ_DIAG="0"
DURATION=""
LOC_DURATION=""
T3_COVERAGE="full_gt_span"
CONSUMER_MODE="stamp"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-root) OUT_ROOT="$2"; shift 2 ;;
        --ref-dir) REF_DIR="$2"; shift 2 ;;
        --fit-until-s) FIT_UNTIL_S="$2"; shift 2 ;;
        --rate) RATE="$2"; shift 2 ;;
        --domain-id) DOMAIN_ID="$2"; shift 2 ;;
        --variants) VARIANTS="$2"; shift 2 ;;
        --consumer-mode) CONSUMER_MODE="$2"; shift 2 ;;
        --skip-mapping) SKIP_MAPPING="1"; shift ;;
        --skip-localization) SKIP_LOCALIZATION="1"; shift ;;
        --with-trajectory-diagnostic) WITH_TRAJ_DIAG="1"; shift ;;
        --duration) DURATION="$2"; shift 2 ;;
        --loc-duration) LOC_DURATION="$2"; shift 2 ;;
        --t3-coverage-window) T3_COVERAGE="$2"; shift 2 ;;
        --dry-run) DRY_RUN="1"; shift ;;
        -h|--help) sed -n '2,14p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# the consumer semantics are declared once and threaded down to run_localization_sim.sh, which
# records them in the run manifest and selects the map_pose_publisher parameters in the launch
case "${CONSUMER_MODE}" in
    stamp|current|predict) ;;
    *) echo "[error] --consumer-mode must be stamp|current|predict (got '${CONSUMER_MODE}')" >&2
       exit 2 ;;
esac

REF_DIR="${REF_DIR:-${OUT_ROOT}/scene_ref}"
mkdir -p "${OUT_ROOT}"
# The reference geometry must be ray-cast over exactly the same window the evaluated map
# comes from, so the run duration is resolved here (default: the trajectory's own duration).
if [[ -z "${DURATION}" ]]; then
    DURATION="$(python3 -c "import json;print(json.load(open('${SIM_DIR}/synthetic_data/test_trajectory.json'))['duration_s'])")"
fi

variant_overlay() {
    case "$1" in
        baseline)   echo "" ;;
        c2_1)       echo "${SIM_DIR}/ablations/c2_1_acc_normalize.yaml" ;;
        c2_2)       echo "${SIM_DIR}/ablations/c2_2_first_batch.yaml" ;;
        c2_3)       echo "${SIM_DIR}/ablations/c2_3_sampling_phase.yaml" ;;
        c2_combined) echo "${SIM_DIR}/ablations/c2_combined.yaml" ;;
        *) echo "__unknown__" ;;
    esac
}

if [[ "${DRY_RUN}" == "1" ]]; then
    echo "=== dry run ==="
    for f in "${SIM_DIR}/synthetic_data/scene_reference.py" \
             "${SIM_DIR}/synthetic_data/generate_test_bag.py" \
             "${SIM_DIR}/synthetic_data/traj_common.py" \
             "${SIM_DIR}/synthetic_data/test_trajectory.json" \
             "${SIM_DIR}/synthetic_data/test_trajectory_localization.json" \
             "${SIM_DIR}/scripts/run_mapping_sim.sh" "${SIM_DIR}/scripts/run_localization_sim.sh" \
             "${SIM_DIR}/scripts/record_run.py" "${SIM_DIR}/scripts/record_localization.py" \
             "${SIM_DIR}/scripts/record_consumer.py" "${SIM_DIR}/scripts/sim_fault_inject.py" \
             "${SIM_DIR}/scripts/make_sim_transform.py" "${SIM_DIR}/scripts/sim_config.py" \
             "${SIM_DIR}/scripts/sim_localizer_config.py" "${SIM_DIR}/scripts/run_manifest.py" \
             "${SIM_DIR}/scripts/test_t1_accuracy.py" "${SIM_DIR}/scripts/test_t2_speed.py" \
             "${SIM_DIR}/scripts/test_t3_localization.py" \
             "${SIM_DIR}/launch/sim_fastlio2.launch.py" \
             "${SIM_DIR}/launch/sim_localization.launch.py"; do
        [[ -f "$f" ]] || { echo "MISSING $f" >&2; exit 1; }
    done
    python3 -m py_compile "${SIM_DIR}"/synthetic_data/*.py "${SIM_DIR}"/scripts/*.py \
        "${SIM_DIR}"/launch/*.py "${SIM_DIR}/sim_common.py" || exit 1
    echo "python syntax OK"
    for f in "${SIM_DIR}"/launch/rviz_config.rviz; do
        python3 -c "import yaml,sys;yaml.safe_load(open(sys.argv[1]))" "$f" || exit 1
    done
    echo "rviz config YAML OK"
    python3 "${SIM_DIR}/synthetic_data/generate_test_bag.py" --check | tail -n 8
    for v in ${VARIANTS//,/ }; do
        ov="$(variant_overlay "$v")"
        [[ "$ov" == "__unknown__" ]] && { echo "unknown variant $v (available: baseline, c2_1, c2_2, c2_3, c2_combined)" >&2; exit 2; }
        [[ -n "$ov" ]] && python3 -c "import yaml,sys;yaml.safe_load(open(sys.argv[1]))" "$ov" || true
        echo "variant $v -> overlay '${ov}'"
    done
    echo "would run: scene_reference -> run_mapping_sim -> make_sim_transform -> T1/T2 -> run_localization_sim -> T3"
    echo "consumer_mode: ${CONSUMER_MODE}"
    exit 0
fi

# Explicit ROS overlay (see run_localization_sim.sh): SIM_INSTALL_SETUP selects the
# install/setup.bash this pipeline and every runner it starts will use; the default is this
# repository's own install, i.e. previous behaviour.  Only that file is sourced.
SIM_INSTALL_SETUP="${SIM_INSTALL_SETUP:-${REPO_ROOT}/install/setup.bash}"
[[ -f "${SIM_INSTALL_SETUP}" ]] || { echo "[error] ROS overlay setup not found: ${SIM_INSTALL_SETUP} (export SIM_INSTALL_SETUP=<install/setup.bash>)" >&2; exit 2; }
export SIM_INSTALL_SETUP
set +u
source /opt/ros/humble/setup.bash
source "${SIM_INSTALL_SETUP}"
set -u
say() { printf '\n========== %s ==========\n' "$*"; }
say "ROS overlay: ${SIM_INSTALL_SETUP} (domain ${DOMAIN_ID}, consumer_mode ${CONSUMER_MODE})"

say "0. synthetic scene reference (frame: scene)"
python3 "${SIM_DIR}/synthetic_data/scene_reference.py" --out-dir "${REF_DIR}" \
    --duration "${DURATION}" --eval-start-s "${FIT_UNTIL_S}"

SUMMARY="${OUT_ROOT}/integration_report.json"
python3 - "${SUMMARY}" "${OUT_ROOT}" "${REF_DIR}" "${FIT_UNTIL_S}" "${DOMAIN_ID}" "${CONSUMER_MODE}" "${SIM_DIR}" "${REPO_ROOT}" <<'PY'
import json, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), "."))
sys.path.insert(0, sys.argv[7])
import sim_common as sc
path = sys.argv[1]
rep = sc.read_json(path) if os.path.isfile(path) else {}
head, dirty = sc.git_head(sys.argv[8])
rep.update({"tool": "run_eval_pipeline.sh", "created_utc": sc.utcnow(),
            "git_head": head, "git_dirty": dirty, "out_root": sys.argv[2],
            "scene_ref_dir": sys.argv[3], "fit_until_s": float(sys.argv[4]),
            "domain_id": sys.argv[5], "consumer_mode": sys.argv[6], "variants": {}})
sc.write_json(path, rep)
PY

for VARIANT in ${VARIANTS//,/ }; do
    OVERLAY="$(variant_overlay "${VARIANT}")"
    if [[ "${OVERLAY}" == "__unknown__" ]]; then
        echo "unknown variant ${VARIANT} (available: baseline, c2_1, c2_2, c2_3, c2_combined)" >&2; exit 2
    fi
    EXTRA_ARGS=()
    if [[ -n "${OVERLAY}" ]]; then
        EXTRA_ARGS=(--extra-config "${OVERLAY}")
    fi
    RUN_DIR="${OUT_ROOT}/${VARIANT}/mapping"
    LOC_DIR="${OUT_ROOT}/${VARIANT}/localization"
    mkdir -p "${RUN_DIR}" "${LOC_DIR}"

    if [[ "${SKIP_MAPPING}" != "1" ]]; then
        say "1. mapping run (${VARIANT}) -> ${RUN_DIR}"
        DUR_ARGS=(--duration "${DURATION}")
        SIM_DOMAIN_ID="${DOMAIN_ID}" bash "${HERE}/run_mapping_sim.sh" --run-dir "${RUN_DIR}" \
            --rate "${RATE}" --fit-until-s "${FIT_UNTIL_S}" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} \
            ${DUR_ARGS[@]+"${DUR_ARGS[@]}"} --domain-id "${DOMAIN_ID}"
        MR=$?
        if [[ ${MR} -ne 0 ]]; then
            echo "[pipeline] mapping run FAILED for ${VARIANT} (exit ${MR}); recording and continuing"
        fi
    fi

    T1=2; T2=2; T3=2
    if [[ -f "${RUN_DIR}/odom.tum" ]]; then
        say "2. frozen frame chain + frozen prior map (${VARIANT})"
        python3 "${HERE}/make_sim_transform.py" \
            --gt-tum "${REF_DIR}/gt_mapping.tum" --odom-tum "${RUN_DIR}/odom.tum" \
            --t0-epoch "$(python3 -c "import json;print(json.load(open('${SIM_DIR}/synthetic_data/test_trajectory.json'))['t0_epoch_s'])")" \
            --fit-until-s "${FIT_UNTIL_S}" \
            --apply-map "${RUN_DIR}/map.pcd" --out-map "${RUN_DIR}/frozen_map.pcd" \
            --out "${RUN_DIR}/frame_chain.json"
        if [[ "${WITH_TRAJ_DIAG}" == "1" ]]; then
            say "2b. trajectory diagnostic (SE(3)-aligned; NOT an acceptance metric)"
            python3 "${HERE}/check_trajectory.py" --odom "${RUN_DIR}/odom.tum" \
                --trajectory "${SIM_DIR}/synthetic_data/test_trajectory.json" \
                --duration "$(python3 -c "import json;print(json.load(open('${SIM_DIR}/synthetic_data/test_trajectory.json'))['duration_s'])")" \
                --out "${RUN_DIR}/trajectory_result.json" || true
        fi
    fi

    say "3. T1 / T2 (${VARIANT})"
    python3 "${HERE}/test_t1_accuracy.py" --run-dir "${RUN_DIR}" --ref-dir "${REF_DIR}" \
        --out "${RUN_DIR}/t1_result.json"; T1=$?
    python3 "${HERE}/test_t2_speed.py" --run-dir "${RUN_DIR}" --out "${RUN_DIR}/t2_result.json"; T2=$?

    T3=2
    if [[ "${SKIP_LOCALIZATION}" != "1" && -f "${RUN_DIR}/frozen_map.pcd" ]]; then
        say "4. separate localization run (${VARIANT}) -> ${LOC_DIR}"
        LDUR_ARGS=()
        [[ -n "${LOC_DURATION}" ]] && LDUR_ARGS=(--duration "${LOC_DURATION}")
        SIM_DOMAIN_ID="${DOMAIN_ID}" bash "${HERE}/run_localization_sim.sh" --run-dir "${LOC_DIR}" \
            --prior-map "${RUN_DIR}/frozen_map.pcd" --rate "${RATE}" \
            --consumer-mode "${CONSUMER_MODE}" \
            ${LDUR_ARGS[@]+"${LDUR_ARGS[@]}"} --domain-id "${DOMAIN_ID}"
        LR=$?
        if [[ ${LR} -ne 0 ]]; then
            echo "[pipeline] localization run FAILED for ${VARIANT} (exit ${LR})"
        fi
        say "5. T3 (${VARIANT})"
        python3 "${HERE}/test_t3_localization.py" --run-dir "${LOC_DIR}" --ref-dir "${REF_DIR}" \
            --coverage-window "${T3_COVERAGE}" --out "${LOC_DIR}/t3_result.json"; T3=$?
    fi

    say "6. aggregate (${VARIANT})"
    python3 - "${SUMMARY}" "${VARIANT}" "${RUN_DIR}" "${LOC_DIR}" "${T1}" "${T2}" "${T3}" "${SIM_DIR}" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[7])
import sim_common as sc
path, variant, run_dir, loc_dir, t1, t2, t3 = sys.argv[1:8]
rep = sc.read_json(path)
def load(p):
    return sc.read_json(p) if os.path.isfile(p) else None
entry = {
    "mapping_run_dir": run_dir, "localization_run_dir": loc_dir,
    "exit_codes": {"T1": int(t1), "T2": int(t2), "T3": int(t3)},
    "verdict": {0: "PASS", 1: "FAIL", 2: "BLOCKED"}.get(int(t1), "ERROR") + "/" +
               {0: "PASS", 1: "FAIL", 2: "BLOCKED"}.get(int(t2), "ERROR") + "/" +
               {0: "PASS", 1: "FAIL", 2: "BLOCKED"}.get(int(t3), "ERROR"),
    "t1": load(os.path.join(run_dir, "t1_verdict.json")),
    "t2": load(os.path.join(run_dir, "t2_verdict.json")),
    "t3": load(os.path.join(loc_dir, "t3_verdict.json")),
    "hashes": {name: sc.sha256_file(p) for name, p in (
        ("map", os.path.join(run_dir, "map.pcd")),
        ("map_eval", os.path.join(run_dir, "map_eval.pcd")),
        ("odom", os.path.join(run_dir, "odom.tum")),
        ("frame_chain", os.path.join(run_dir, "frame_chain.json")),
        ("frozen_map", os.path.join(run_dir, "frozen_map.pcd")),
        ("localization", os.path.join(loc_dir, "localization.tum")),
    ) if os.path.isfile(p)},
    "logs": {name: p for name, p in (
        ("mapping_launch", os.path.join(run_dir, "launch.log")),
        ("mapping_record", os.path.join(run_dir, "record.log")),
        ("localization_launch", os.path.join(loc_dir, "launch.log")),
        ("localization_record", os.path.join(loc_dir, "record.log")),
    )},
}
rep.setdefault("variants", {})[variant] = entry
sc.write_json(path, rep)
print("[pipeline] %s -> %s" % (variant, entry["verdict"]))
PY
done

say "done: ${SUMMARY}"
python3 - "${SUMMARY}" <<'PY'
import json, sys
rep = json.load(open(sys.argv[1]))
for name, e in (rep.get("variants") or {}).items():
    print("%-14s %s" % (name, e["verdict"]))
PY
exit 0
