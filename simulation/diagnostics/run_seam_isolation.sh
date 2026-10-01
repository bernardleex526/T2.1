#!/usr/bin/env bash
# Controlled isolation: the PRE-FIX bag, truncated just before the seam glitch (t<85.9 s),
# evaluated against the PRE-FIX reference with a chain fitted on its own pre-eval odometry.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=137 ROS_LOCALHOST_ONLY=1
REF="${ROOT}/artifacts/simulation_20260930/scene_ref"
RUN=/tmp/diag/run_bag859
bash simulation/scripts/run_mapping_sim.sh --run-dir "$RUN" --bag-dir /tmp/diag/bag859 \
     --skip-bag --duration 85.9 --fit-until-s 30 --domain-id 137
python3 simulation/scripts/make_sim_transform.py --gt-tum "$REF/gt_mapping.tum" \
     --odom-tum "$RUN/odom.tum" --t0-epoch 1700000000 --fit-until-s 30 \
     --apply-map "$RUN/map.pcd" --out-map "$RUN/frozen_map.pcd" \
     --out "$RUN/frame_chain.json"
python3 simulation/scripts/test_t1_accuracy.py --run-dir "$RUN" --ref-dir "$REF" \
     --out "$RUN/t1_result.json"
echo "=== controlled run exit: T1=$? ==="
