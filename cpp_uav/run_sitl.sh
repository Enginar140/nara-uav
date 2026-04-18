#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

EXE="build/bin/uav_system"
CONFIG="config/sitl.yaml"

if [ ! -f "$EXE" ]; then
    echo "Executable bulunamadi, once build ediliyor..."
    ./build.sh Release
fi

echo ""
echo "========================================"
echo "  UAV v10 - SITL Mode"
echo "========================================"
echo "  Config: $CONFIG"
echo "  Pixhawk: udpin:127.0.0.1:14550"
echo "  GCS:     tcpin:127.0.0.1:14552"
echo ""
echo "  ArduPilot SITL baslatin:"
echo "    sim_vehicle.py -v ArduPlane --map --console"
echo ""
echo "  Ctrl+C ile durdurun."
echo "========================================"
echo ""

exec "$EXE" "$CONFIG"
