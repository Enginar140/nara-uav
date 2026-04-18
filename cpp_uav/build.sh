#!/bin/bash
set -e

BUILD_TYPE="${1:-Release}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo ""
echo "========================================"
echo "  UAV v10 - Build ($BUILD_TYPE)"
echo "========================================"
echo ""

if [ ! -d build ]; then
    echo "[1/2] CMake configure..."
    cmake -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
else
    echo "[1/2] Build dizini mevcut, configure atlaniyor..."
    echo "       (Temiz build icin: ./clean.sh)"
fi

echo "[2/2] Building..."
cmake --build build --config "$BUILD_TYPE" --parallel "$(nproc)"

echo ""
echo "========================================"
echo "  Build basarili!"
echo "  Cikti: build/bin/uav_system"
echo "========================================"
echo ""
