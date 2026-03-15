#!/usr/bin/env bash
# ============================================================
# gNNS Drone — Jetson Nano one-time setup
# ============================================================
# Run once after cloning the repo on the Jetson Nano.
# Ensures Python deps, serial port access, and optional venv.
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=== gNNS Drone — Jetson Nano setup ==="
echo "Project root: $PROJECT_ROOT"
cd "$PROJECT_ROOT"

# 1. Serial port (TELEM2 on Nano = /dev/ttyTHS1)
echo ""
echo "--- Serial port ---"
if [ -e /dev/ttyTHS1 ]; then
  echo "  /dev/ttyTHS1 found (Jetson hardware UART)."
  echo "  To allow non-root access, add udev rule:"
  echo "  sudo cp $SCRIPT_DIR/99-gnns-telem.rules /etc/udev/rules.d/"
  echo "  sudo udevadm control --reload-rules && sudo udevadm trigger"
  echo "  Then unplug/replug the serial connection or reboot."
else
  echo "  /dev/ttyTHS1 not found. If using USB serial, use /dev/ttyUSB0 and set in config/mavlink_config.yaml"
fi

# 2. Python dependencies
echo ""
echo "--- Python dependencies ---"
if [ -f "requirements.txt" ]; then
  echo "  Installing from requirements.txt..."
  pip3 install --user -r requirements.txt || true
  echo "  Done. If you use a venv, activate it and run: pip install -r requirements.txt"
else
  echo "  requirements.txt not found; skip pip install."
fi

# 3. Optional: create venv
echo ""
echo "--- Optional virtual environment ---"
echo "  To use a venv:"
echo "    cd $PROJECT_ROOT"
echo "    python3 -m venv venv"
echo "    source venv/bin/activate"
echo "    pip install -r requirements.txt"
echo "  Then run: ./scripts/jetson_nano/run_mission.sh"

# 4. Run mission
echo ""
echo "=== Setup complete ==="
echo "  Run mission:  ./scripts/jetson_nano/run_mission.sh"
echo "  Demo (SITL): ./scripts/jetson_nano/run_mission.sh --sitl --demo"
echo ""
