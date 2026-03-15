#!/usr/bin/env bash
# ============================================================
# gNNS Drone — Jetson Nano mission launcher
# ============================================================
# Run the full mission from the Jetson Nano with one command.
# Usage (from anywhere):
#   ./scripts/jetson_nano/run_mission.sh
#   ./scripts/jetson_nano/run_mission.sh --demo
#   ./scripts/jetson_nano/run_mission.sh --config /path/to/mavlink_config.yaml
# ============================================================

set -e

# Resolve project root (directory containing gnns_drone/ and config/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$PROJECT_ROOT"

# Optional: detect Jetson (aarch64)
ARCH="$(uname -m)"
if [ "$ARCH" = "aarch64" ]; then
  echo "[Jetson Nano] Running on aarch64 (MAVLink port from config/mavlink_config.yaml)."
else
  echo "[Info] Not aarch64 (current: $ARCH). Running anyway (e.g. for testing)."
fi

# Ensure Python can find the package (in case not installed in venv)
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH:-}"

# Run mission; pass all arguments (e.g. --demo, --config, --interactive)
exec python3 -m gnns_drone "$@"
