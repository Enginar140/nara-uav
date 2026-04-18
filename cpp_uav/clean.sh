#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "Cleaning build directory..."
if [ -d build ]; then
    rm -rf build
    echo "Build dizini silindi."
else
    echo "Zaten temiz."
fi
echo ""
