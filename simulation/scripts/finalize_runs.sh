#!/usr/bin/env bash
# Re-run the three metric tests for every variant of a completed pipeline run, using the
# ALREADY RECORDED artifacts (no simulation, no bag regeneration), then rebuild the
# integration report and persist the evidence into the repository.
#
#   ./finalize_runs.sh --out-root /tmp/sim_pipeline --dest fork/artifacts/simulation_final
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${HERE}/.." && pwd)"
REPO_ROOT="$(cd "${SIM_DIR}/.." && pwd)"
OUT_ROOT="/tmp/sim_pipeline"
DEST=""
REPORT=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-root) OUT_ROOT="$2"; shift 2 ;;
        --dest) DEST="$2"; shift 2 ;;
        --report) REPORT="$2"; shift 2 ;;
        -h|--help) sed -n '2,8p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

for V in $(python3 -c "
import json,sys
d=json.load(open('${OUT_ROOT}/integration_report.json'))
print(' '.join(d.get('variants',{}).keys()))"); do
    RUN="${OUT_ROOT}/${V}/mapping"
    LOC="${OUT_ROOT}/${V}/localization"
    echo "--- ${V} ---"
    python3 "${HERE}/test_t1_accuracy.py" --run-dir "${RUN}" --ref-dir "${OUT_ROOT}/scene_ref" \
        --out "${RUN}/t1_result.json" > "${RUN}/t1_test.log" 2>&1; T1=$?
    python3 "${HERE}/test_t2_speed.py" --run-dir "${RUN}" \
        --out "${RUN}/t2_result.json" > "${RUN}/t2_test.log" 2>&1; T2=$?
    T3=2
    if [[ -f "${LOC}/localization.tum" ]]; then
        python3 "${HERE}/test_t3_localization.py" --run-dir "${LOC}" \
            --ref-dir "${OUT_ROOT}/scene_ref" --out "${LOC}/t3_result.json" \
            > "${LOC}/t3_test.log" 2>&1; T3=$?
    fi
    python3 - "${OUT_ROOT}/integration_report.json" "${V}" "${RUN}" "${LOC}" "${T1}" "${T2}" "${T3}" "${SIM_DIR}" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[7])
import sim_common as sc
path, variant, run_dir, loc_dir, t1, t2, t3 = sys.argv[1:8]
rep = sc.read_json(path)
load = lambda p: sc.read_json(p) if os.path.isfile(p) else None
e = rep.setdefault("variants", {}).setdefault(variant, {})
e["exit_codes"] = {"T1": int(t1), "T2": int(t2), "T3": int(t3)}
name = {0: "PASS", 1: "FAIL", 2: "BLOCKED"}
e["verdict"] = "/".join(name.get(int(x), "ERROR") for x in (t1, t2, t3))
e["t1"] = load(os.path.join(run_dir, "t1_verdict.json"))
e["t2"] = load(os.path.join(run_dir, "t2_verdict.json"))
e["t3"] = load(os.path.join(loc_dir, "t3_verdict.json"))
sc.write_json(path, rep)
print("[finalize] %s -> %s" % (variant, e["verdict"]))
PY
    echo "T1=${T1} T2=${T2} T3=${T3}"
done

python3 "${HERE}/make_report.py" --out-root "${OUT_ROOT}" --evidence-dir "${DEST}" \
    --out "${REPORT:-${REPO_ROOT}/.omp/reports/simulation_integration.md}"
if [[ -n "${DEST}" ]]; then
    bash "${HERE}/persist_evidence.sh" --out-root "${OUT_ROOT}" --dest "${DEST}"
fi
echo "=== finalize done ==="
