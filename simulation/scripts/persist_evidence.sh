#!/usr/bin/env bash
# Persist the pipeline's evidence inside the repository (Main requirement: final evidence must
# live in fork/artifacts, not /tmp).  Copies manifests, logs, verdicts, result JSONs, the scene
# reference (PCDs included) and the frame chains; SKIPS the multi-hundred-MB bags, whose hashes
# are already recorded in the manifests.
#
#   ./persist_evidence.sh --out-root /tmp/sim_pipeline [--dest fork/artifacts/simulation_<utc>]
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${HERE}/../.." && pwd)"
OUT_ROOT="/tmp/sim_pipeline"
DEST=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-root) OUT_ROOT="$2"; shift 2 ;;
        --dest) DEST="$2"; shift 2 ;;
        -h|--help) sed -n '2,9p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

[[ -d "${OUT_ROOT}" ]] || { echo "[error] no run root: ${OUT_ROOT}" >&2; exit 2; }
if [[ -z "${DEST}" ]]; then
    DEST="${REPO_ROOT}/artifacts/simulation_$(date -u +%Y%m%dT%H%M%SZ)"
fi
mkdir -p "${DEST}"

# Everything except the bags and the per-message point clouds.
rsync -a --exclude '*/bag/' --exclude 'bag_*.db3' --exclude 'metadata.yaml' \
      "${OUT_ROOT}/" "${DEST}/"

python3 - "${DEST}" "${OUT_ROOT}" "${SIM_DIR}" <<'PY'
import json, os, sys
sys.path.insert(0, sys.argv[3])
import sim_common as sc
dest, src = sys.argv[1], sys.argv[2]
index = {"source_run_root": os.path.abspath(src), "persisted_utc": sc.utcnow(),
         "excluded": ["*/bag/ (rosbag2 sqlite + metadata; hashes are in run_manifest.json)"],
         "files": {}}
for dp, _, fs in os.walk(dest):
    for f in sorted(fs):
        p = os.path.join(dp, f)
        rel = os.path.relpath(p, dest)
        index["files"][rel] = {"bytes": os.path.getsize(p), "sha256": sc.sha256_file(p)}
sc.write_json(os.path.join(dest, "EVIDENCE_INDEX.json"), index)
print("[persist_evidence] %d files indexed under %s" % (len(index["files"]), dest))
PY
echo "[persist_evidence] persisted to ${DEST}"
