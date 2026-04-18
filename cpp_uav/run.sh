#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

CONFIG="$1"
EXE="build/bin/uav_system"

if [ ! -f "$EXE" ]; then
    echo "Executable bulunamadi: $EXE"
    echo "Once build edin: ./build.sh"
    exit 1
fi

echo ""
echo "========================================"
echo "  UAV v10 - Calistiriliyor"
echo "========================================"

if [ -z "$CONFIG" ]; then
    echo "  Config: default (otomatik arama)"
    echo ""
    exec "$EXE"
else
    if [ ! -f "$CONFIG" ]; then
        echo "  HATA: Config dosyasi bulunamadi: $CONFIG"
        exit 1
    fi
    echo "  Config: $CONFIG"
    echo ""
    exec "$EXE" "$CONFIG"
fi
