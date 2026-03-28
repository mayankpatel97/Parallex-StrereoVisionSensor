#!/bin/bash
# build.sh — builds the stereo vision system on Raspberry Pi CM4
# Run: chmod +x build.sh && ./build.sh

set -e

echo "=== Stereo Vision System — CM4 Build Script ==="
echo ""

# ── Install system dependencies ──────────────────────────────────────────────
echo "[1/4] Installing dependencies..."
sudo apt update -qq
sudo apt install -y \
    cmake \
    g++ \
    libopencv-dev \
    libopencv-contrib-dev \
    pkg-config \
    libcamera-dev \
    libcamera-tools \
    python3-libcamera

echo "      Done."

# ── Create build directory ───────────────────────────────────────────────────
echo "[2/4] Configuring CMake..."
mkdir -p build
cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$HOME/.local"

echo "      Done."

# ── Compile ──────────────────────────────────────────────────────────────────
echo "[3/4] Compiling (using all 4 cores)..."
make -j4
echo "      Done."

# ── Install to ~/.local/bin ──────────────────────────────────────────────────
echo "[4/4] Installing binaries..."
make install
echo "      Installed to $HOME/.local/bin"

cd ..

# ── Print usage ──────────────────────────────────────────────────────────────
echo ""
echo "=== Build complete ==="
echo ""
echo "Binaries:"
echo "  ./build/calibrate       — stereo calibration tool"
echo "  ./build/stereo_vision   — real-time depth pipeline"
echo "  ./build/depth_query     — click-to-measure utility"
echo ""
echo "Quick start:"
echo "  1. Calibrate:  ./build/calibrate --images ./calib"
echo "  2. Run:        ./build/stereo_vision --720p"
echo "  3. Measure:    ./build/depth_query --720p"
echo ""
echo "Runtime flags for stereo_vision:"
echo "  --480p          640×480 (fastest, ~35 fps)"
echo "  --720p          1280×720 (default, ~18 fps)"
echo "  --1080p         1920×1080 (slowest, ~9 fps)"
echo "  --show-rect     display rectified pair with epipolar lines"
echo "  --wls           enable WLS disparity filter (smoother, ~30% slower)"
echo "  --save-depth    save depth frames as 16-bit PNG"
echo "  --calib FILE    use a custom calibration file"
echo ""
echo "Keyboard shortcuts (while running):"
echo "  q / ESC         quit"
echo "  s               save snapshot (left + right + depth)"
echo "  r               toggle rectified view"
