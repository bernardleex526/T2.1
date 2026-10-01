#!/usr/bin/env bash
# Run the three metric tests (T1 map accuracy, T2 mapping speed, T3 localisation).
# Each test: 0 = PASS, 1 = FAIL, 2 = BLOCKED.
# NOTE: no `set -e` -- a failing test must not abort the remaining ones.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${HERE}"
PY="${PYTHON:-python3}"

RUN_DIR="${SIM_RUN_DIR:-/tmp/fastlio2_output}"
REF_DIR="${SIM_REF_DIR:-/tmp/fastlio2_scene_ref}"
LOC_RUN_DIR="${SIM_LOC_RUN_DIR:-/tmp/fastlio2_localization}"

run_test() {
  local label="$1" script="$2"; shift 2
  printf '\n[%s]\n' "${label}"
  "${PY}" "${script}" "$@"
  local code=$?
  case ${code} in
    0) echo "→ ${label} PASS" ;;
    1) echo "→ ${label} FAIL" ;;
    2) echo "→ ${label} BLOCKED（不可测，见上方原因）" ;;
    *) echo "→ ${label} ERROR (exit ${code})" ;;
  esac
  return ${code}
}

echo "=== 运行三项指标测试 (run=${RUN_DIR}, ref=${REF_DIR}, loc=${LOC_RUN_DIR}) ==="
run_test "1/3 T1: 建图精度 ≤5cm"   test_t1_accuracy.py --run-dir "${RUN_DIR}" --ref-dir "${REF_DIR}"; T1=$?
run_test "2/3 T2: 建图速度 ≥0.5m/s" test_t2_speed.py --run-dir "${RUN_DIR}";                 T2=$?
run_test "3/3 T3: 定位精度 ≤5cm"   test_t3_localization.py --run-dir "${LOC_RUN_DIR}" --ref-dir "${REF_DIR}"; T3=$?

printf '\n=== 测试总结 ===\n'
for pair in "T1:${T1}" "T2:${T2}" "T3:${T3}"; do
  name="${pair%%:*}"; code="${pair##*:}"
  case ${code} in
    0) echo "✅ ${name} PASS" ;;
    1) echo "❌ ${name} FAIL" ;;
    2) echo "⚠️  ${name} BLOCKED" ;;
    *) echo "❌ ${name} ERROR(${code})" ;;
  esac
done

status=0
for code in "${T1}" "${T2}" "${T3}"; do
  [ "${code}" -ne 0 ] && status=1
done
exit "${status}"
