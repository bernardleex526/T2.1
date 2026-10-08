#!/usr/bin/env bash
# Offline recomputation of the corrected (v2.2) map metric over EXISTING artifacts.
# Owner: EvalContract.  Consumes NO replay budget: nothing here runs an online
# algorithm, a bag, or the front-end.  It reads the old diagnosis artifacts and
# the survey reference map, and writes ONLY into the --out-dir given by the caller.
#
#   bash recompute_commands.sh --out-dir PATH      # PATH is REQUIRED
#
# [step 6 / R14 revision] The output directory used to be hard-coded to
# evaluation/results/, which meant a re-run overwrote frozen evidence.  It is now a
# REQUIRED argument:
#   * no default exists, so every recorded legacy invocation ("bash recompute_commands.sh",
#     e.g. in verification_plan.json / evaluator_verification.json) fails closed with
#     exit 2 instead of silently rewriting the archive;
#   * --out-dir is canonicalized (realpath -m, i.e. WITHOUT requiring it to exist) and then
#     refused (exit 3) when it EQUALS, is UNDER, or is an ANCESTOR of any frozen path in
#     FROZEN_PATHS (the frozen root, the frozen localizer_evidence/out/ and the legacy
#     evaluation/results/).  Containment is a pure string test on canonical absolute paths,
#     so a symlink alias, a `..` segment or a relative spelling cannot slip past it;
#   * any of the four derived artifacts already present in --out-dir is never overwritten
#     (exit 3), and there is no --force.
#
# Ordering (no filesystem mutation before the guards):
#   parse args -> required check -> canonicalize -> frozen containment guard
#   -> existing-artifact guard -> mkdir -p -> writes
#
# Steps:
#   1. derive the DIAGNOSTIC-only support region from the reference extent
#      (diagnostic_only=true -> the tool can never emit a PASS from it)
#   2. recompute the corrected metric for the two archived replays
#   3. strict cross-check of the arithmetic for one of them
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
usage: bash recompute_commands.sh --out-dir PATH

Required:
  --out-dir PATH   NEW directory receiving every derived artifact
                   (roi_diagnostic_*.json, *_map_accuracy_v2.json, *_cross_check_strict.json,
                   recompute_run.json).  The path is canonicalized with realpath -m and REFUSED
                   (exit 3) when it equals, lies under, or is an ancestor of any frozen path
                   (the frozen root, the frozen localizer_evidence/out/, the legacy
                   evaluation/results/).  Refused (exit 3) as well if any derived artifact of
                   that name already exists there.

Optional environment:
  VOXEL            evaluation sampling voxel in metres (default 0.25, always recorded)
USAGE
}

EVAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FROZEN_ROOT="/home/lee/t21_wp2/fork/artifacts/04b73f5_evidence_audit_20260929_5V6fKb"
LEGACY_OUT="${EVAL_DIR}/results"
OLD="/home/lee/t21_wp2/fork/artifacts/04b73f5_diagnosis_20260929_JWUGeL"
REF="/home/lee/t21_wp2/datasets/gt/mcd/tuhh_night_09_seeded.pcd"
VOXEL="${VOXEL:-0.25}"          # evaluation sampling voxel (recorded in every result)

# Every path whose bytes are frozen for this round.  A NEW --out-dir must lie outside all of
# them: equality, descendant and ancestor are all refused.
FROZEN_PATHS=(
  "${FROZEN_ROOT}"
  "${FROZEN_ROOT}/localizer_evidence/out"
  "${LEGACY_OUT}"
)

frozen_match() {
  # $1 = candidate path, ALREADY canonicalized to an absolute path (realpath -m).
  # Pure string containment, no [ -e ]: prints the frozen path that matched and returns 0,
  # else returns 1.  Equal / descendant / ancestor are all covered by the two prefix tests.
  # (The frozen paths contain no glob metacharacters, so `case` sees plain literals.)
  local cand="${1%/}/" orig fp
  for orig in "${FROZEN_PATHS[@]}"; do
    fp="$(realpath -m -- "${orig}")"
    fp="${fp%/}/"
    case "${cand}" in "${fp}"*) printf '%s' "${orig}"; return 0 ;; esac
    case "${fp}" in "${cand}"*) printf '%s' "${orig}"; return 0 ;; esac
  done
  return 1
}

OUT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --out-dir)   [ $# -ge 2 ] || { echo "error: --out-dir needs PATH" >&2; usage; exit 2; }
                 OUT="$2"; shift 2 ;;
    --out-dir=*) OUT="${1#*=}"; shift ;;
    -h|--help)   usage; exit 0 ;;
    *)           echo "error: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

if [ -z "${OUT}" ]; then
  echo "error: --out-dir PATH is REQUIRED (the frozen evaluation/results/ is never written)" >&2
  usage
  exit 2
fi

# canonicalize WITHOUT creating anything, so every guard can run before any mutation
OUT="$(realpath -m -- "${OUT}")"

if _frozen_hit="$(frozen_match "${OUT}")"; then
  echo "error: refusing to write into the frozen namespace: --out-dir ${OUT} equals, is under, or is an ancestor of the frozen path ${_frozen_hit} (frozen root: ${FROZEN_ROOT})" >&2
  echo "       choose a NEW --out-dir outside the frozen tree" >&2
  exit 3
fi

ARTIFACTS="roi_diagnostic_mcd_tuhh_night_09.json
replay_1_upstream_map_accuracy_v2.json
replay_2_fork_map_accuracy_v2.json
replay_2_fork_cross_check_strict.json"
for f in ${ARTIFACTS}; do
  if [ -e "${OUT}/${f}" ]; then
    echo "error: ${OUT}/${f} already exists; refusing to overwrite (use a NEW --out-dir)" >&2
    exit 3
  fi
done

# first filesystem mutation in this script: everything above is read-only
mkdir -p "${OUT}"

sha256_of() { sha256sum "$1" | cut -d' ' -f1; }

cat > "${OUT}/recompute_run.json" <<EOF
{
 "schema": "recompute_run/v1",
 "produced_by": "evaluation/recompute_commands.sh",
 "out_dir": "${OUT}",
 "frozen_out_dir_refused": "${LEGACY_OUT}",
 "stat_voxel_m": ${VOXEL},
 "replay_budget_consumed": 0,
 "inputs": {
  "diagnosis_root": "${OLD}",
  "replay_1_upstream_map_pcd": {"path": "${OLD}/replay_1_upstream/map/map.pcd", "sha256": "$(sha256_of "${OLD}/replay_1_upstream/map/map.pcd")"},
  "replay_2_fork_map_pcd": {"path": "${OLD}/replay_2_fork/map/map.pcd", "sha256": "$(sha256_of "${OLD}/replay_2_fork/map/map.pcd")"},
  "reference_map": {"path": "${REF}", "sha256": "$(sha256_of "${REF}")"}
 },
 "note": "provenance of this recomputation only; the ROI built in step 1 is diagnostic-only, so no accuracy PASS/FAIL can be produced while the pre-defined ROI and the sourced coordinate chain are missing"
}
EOF

echo "== 1/3 diagnostic ROI from the reference extent"
python3 "${EVAL_DIR}/make_diagnostic_roi.py" \
  --ref-map "${REF}" \
  --out "${OUT}/roi_diagnostic_mcd_tuhh_night_09.json"

for R in replay_1_upstream replay_2_fork; do
  echo "== 2/3 corrected map metric: ${R} (voxel ${VOXEL} m)"
  python3 "${EVAL_DIR}/map_accuracy_eval.py" \
    --est-map "${OLD}/${R}/map/map.pcd" \
    --ref-map "${REF}" \
    --roi "${OUT}/roi_diagnostic_mcd_tuhh_night_09.json" \
    --stat-voxel "${VOXEL}" \
    --run-name "${R}" --sequence mcd_tuhh_night_09 \
    --out "${OUT}/${R}_map_accuracy_v2.json"
done

echo "== 3/3 strict cross-check (same estimand, independent arithmetic): replay_2_fork"
python3 "${EVAL_DIR}/cross_check_map_eval.py" --mode strict \
  --est-map "${OLD}/replay_2_fork/map/map.pcd" \
  --ref-map "${REF}" \
  --roi "${OUT}/roi_diagnostic_mcd_tuhh_night_09.json" \
  --main-result "${OUT}/replay_2_fork_map_accuracy_v2.json" \
  --stat-voxel "${VOXEL}" \
  --out "${OUT}/replay_2_fork_cross_check_strict.json"

echo "== done.  results in ${OUT}"
echo "   NOTE: no accuracy PASS/FAIL is produced while the pre-defined ROI and the"
echo "   sourced coordinate chain are missing (see verification_plan.json blockers)."
