#!/bin/bash
# X5 (地瓜派 aarch64) 构建脚本
# 用法: bash build_x5.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build_x5"

ORBBEC_SDK_ROOT="/root/OrbbecSDK_C_C++_v1.10.27_20250925_0549823_linux_arm64_release/OrbbecSDK_v1.10.27/SDK"

echo "========================================"
echo "  Glass Cleaning Drone Vision - X5 Build"
echo "========================================"
echo "Build dir : $BUILD_DIR"
echo "Orbbec SDK: $ORBBEC_SDK_ROOT"
echo ""

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

cmake "$SCRIPT_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DORBBEC_SDK_ROOT="$ORBBEC_SDK_ROOT" \
    -DSIMULATION_MODE=OFF \
    -DUSE_ORBBEC_SDK=ON \
    -DENABLE_UART=ON \
    -DENABLE_DEBUG_VISUALIZER=OFF

echo ""
echo "Building..."
cmake --build . --parallel $(nproc)

echo ""
echo "========================================"
echo "  Build SUCCESS"
echo "  Binary: $BUILD_DIR/glass_cleaning_drone_vision"
echo "========================================"
