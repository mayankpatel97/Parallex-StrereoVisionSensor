#pragma once

#include <opencv2/core.hpp>
#include <string>

// ── Calibration data ──────────────────────────────────────────────────────────

struct StereoCalib {
    cv::Mat K1, D1;          // Left  camera intrinsics + distortion
    cv::Mat K2, D2;          // Right camera intrinsics + distortion
    cv::Mat R1, R2;          // Rectification rotation matrices
    cv::Mat P1, P2;          // Projection matrices after rectification
    cv::Mat Q;               // Disparity-to-depth mapping matrix

    // Pre-computed undistort + rectify maps (built once from above)
    cv::Mat map1x, map1y;
    cv::Mat map2x, map2y;
};

// ── SGBM tuning parameters ────────────────────────────────────────────────────

struct SGBMConfig {
    int minDisparity     = 0;
    int numDisparities   = 96;   // Must be divisible by 16
    int blockSize        = 7;    // Odd number, 5–15
    int disp12MaxDiff    = 1;
    int preFilterCap     = 63;
    int uniquenessRatio  = 10;
    int speckleWindowSize = 100;
    int speckleRange     = 32;
};

// ── Application config ────────────────────────────────────────────────────────

struct AppConfig {
    std::string calibFile    = "stereo_calib.yml";
    int   width              = 1280;
    int   height             = 720;
    bool  showRectified      = false;
    bool  useWLS             = false;
    bool  saveDepth          = false;
};
