#!/usr/bin/env bash
# ==============================================================================
# orin_nx_bringup.sh - Jetson Orin NX 16GB Platform Optimization & Sensor Bringup
# WP2 T2.1 Hardware Readiness Script
# ==============================================================================
set -euo pipefail

echo "=================================================================="
echo "  T2.1 Orin NX 16GB Platform Initialization & Performance Lock   "
echo "=================================================================="

# 1. Jetson Power Model (MAXN 25W/MAX mode for Orin NX)
if command -v nvpmodel >/dev/null 2>&1; then
    echo "[1/5] Setting nvpmodel to MAXN mode (-m 0)..."
    sudo nvpmodel -m 0 || true
    echo "Current mode: $(nvpmodel -q | grep -o 'NVPM.*' || true)"
else
    echo "[1/5] nvpmodel not found (non-Jetson or simulation host). Skipping."
fi

# 2. Lock Clocks to Maximum
if command -v jetson_clocks >/dev/null 2>&1; then
    echo "[2/5] Locking hardware clocks to maximum frequencies..."
    sudo jetson_clocks || true
    jetson_clocks --show || true
else
    echo "[2/5] jetson_clocks not found. Skipping."
fi

# 3. Kernel Network Buffer Optimization (prevent LiDAR UDP packet drop)
echo "[3/5] Optimizing kernel socket buffers for LiDAR point cloud streams..."
sudo sysctl -w net.core.rmem_max=67108864 >/dev/null 2>&1 || true
sudo sysctl -w net.core.rmem_default=33554432 >/dev/null 2>&1 || true
sudo sysctl -w net.core.netdev_max_backlog=10000 >/dev/null 2>&1 || true
echo "  net.core.rmem_max set to 64 MB"

# 4. CPU Affinity and RT Scheduling Recommendation
echo "[4/5] Process Scheduling and CPU Core Pinning Strategy:"
echo "  - CPU 0-1: OS kernel, DDS communication, ROS 2 middleware"
echo "  - CPU 2-3: fastlio2 frontend (IESKF + ikd-Tree) -> taskset -c 2,3 chrt -f 50"
echo "  - CPU 4-5: pgo / localizer backend -> taskset -c 4,5 chrt -f 40"
echo "  - CPU 6-7: sensor drivers, camera/rs_to_fastlio, diagnostics"

# 5. PTP / gPTP Hardware Synchronization Status
echo "[5/5] Checking IEEE 1588 PTP Hardware Clock Status..."
if ip link show eth0 >/dev/null 2>&1; then
    if ethtool -T eth0 2>/dev/null | grep -q "hardware-transmit"; then
        echo "  eth0 supports PTP hardware timestamping: READY"
    else
        echo "  eth0 PTP hardware timestamping not detected: use software chrony fallback"
    fi
else
    echo "  Interface eth0 not found."
fi

echo "=================================================================="
echo "  Orin NX Performance Configuration Complete. Ready for Bringup. "
echo "=================================================================="
