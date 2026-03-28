/**
 * stereo_vision.cpp
 * Real-time stereo depth pipeline for Raspberry Pi Compute Module 4
 * Dual CSI cameras via libcamera, stereo matching via OpenCV C++ SGBM
 *
 * Build:  mkdir build && cd build && cmake .. && make -j4
 * Run:    ./stereo_vision [options]
 */

#include "stereo_vision.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc.hpp>      // WLS filter (optional)

#include <libcamera/libcamera.h>

#include <iostream>
#include <chrono>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <filesystem>

using namespace cv;
using namespace std::chrono;
namespace fs = std::filesystem;

// ── Globals ──────────────────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

void signalHandler(int) { g_running = false; }

// ── FrameBuffer – lock-free double buffer ─────────────────────────────────────

struct FramePair {
    Mat left, right;
    steady_clock::time_point timestamp;
};

class DoubleBuffer {
public:
    void write(Mat& l, Mat& r) {
        std::lock_guard<std::mutex> lk(mtx_);
        back_.left  = l.clone();
        back_.right = r.clone();
        back_.timestamp = steady_clock::now();
        ready_ = true;
        cv_.notify_one();
    }

    bool read(FramePair& out, std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!cv_.wait_for(lk, timeout, [this]{ return ready_; }))
            return false;
        std::swap(front_, back_);
        ready_ = false;
        out = front_;
        return true;
    }

private:
    FramePair front_, back_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool ready_{false};
};

static DoubleBuffer g_frameBuf;

// ── Calibration helpers ───────────────────────────────────────────────────────

bool loadCalibration(const std::string& path, StereoCalib& cal, Size imgSize) {
    if (!fs::exists(path)) {
        std::cerr << "[CALIB] File not found: " << path << "\n";
        return false;
    }
    FileStorage fs(path, FileStorage::READ);
    if (!fs.isOpened()) { std::cerr << "[CALIB] Cannot open: " << path << "\n"; return false; }

    fs["K1"] >> cal.K1; fs["D1"] >> cal.D1;
    fs["K2"] >> cal.K2; fs["D2"] >> cal.D2;
    fs["R1"] >> cal.R1; fs["R2"] >> cal.R2;
    fs["P1"] >> cal.P1; fs["P2"] >> cal.P2;
    fs["Q"]  >> cal.Q;

    // Build undistort+rectify maps (computed once, reused every frame)
    initUndistortRectifyMap(cal.K1, cal.D1, cal.R1, cal.P1, imgSize, CV_32FC1, cal.map1x, cal.map1y);
    initUndistortRectifyMap(cal.K2, cal.D2, cal.R2, cal.P2, imgSize, CV_32FC1, cal.map2x, cal.map2y);

    std::cout << "[CALIB] Loaded from " << path << "\n";
    return true;
}

// ── Stereo matcher factory ───────────────────────────────────────────────────

Ptr<StereoSGBM> buildSGBM(const SGBMConfig& cfg) {
    int P1 = 8  * 3 * cfg.blockSize * cfg.blockSize;
    int P2 = 32 * 3 * cfg.blockSize * cfg.blockSize;
    return StereoSGBM::create(
        cfg.minDisparity,
        cfg.numDisparities,
        cfg.blockSize,
        P1, P2,
        cfg.disp12MaxDiff,
        cfg.preFilterCap,
        cfg.uniquenessRatio,
        cfg.speckleWindowSize,
        cfg.speckleRange,
        StereoSGBM::MODE_SGBM_3WAY
    );
}

// ── libcamera capture thread ──────────────────────────────────────────────────
// NOTE: libcamera API is async/event-driven. We use a simplified polling
// wrapper here — production use should implement a full CameraManager loop.

class DualCameraCapture {
public:
    DualCameraCapture(int w, int h) : width_(w), height_(h) {}

    bool open() {
        // In practice on CM4: use v4l2 or libcamera C++ API.
        // We expose a clean interface here; the implementation hooks into
        // the system's camera driver. For the CM4 + libcamera, you would:
        //   1. libcamera::CameraManager::start()
        //   2. Acquire cam0 (index 0) and cam1 (index 1)
        //   3. Configure StreamConfiguration for both
        //   4. Start capture loops with request queuing
        //
        // For direct v4l2 (simpler, works with most Pi cameras):
        //   cap0_.open("/dev/video0", CAP_V4L2);
        //   cap1_.open("/dev/video2", CAP_V4L2);  // cam1 is typically video2

        cap0_.open(0, CAP_V4L2);
        cap1_.open(2, CAP_V4L2);   // cam1 = /dev/video2 on CM4

        if (!cap0_.isOpened() || !cap1_.isOpened()) {
            std::cerr << "[CAM] Failed to open cameras. "
                      << "Ensure both cameras are enabled in /boot/config.txt\n";
            return false;
        }

        for (auto* cap : {&cap0_, &cap1_}) {
            cap->set(CAP_PROP_FRAME_WIDTH,  width_);
            cap->set(CAP_PROP_FRAME_HEIGHT, height_);
            cap->set(CAP_PROP_FPS, 30);
            cap->set(CAP_PROP_BUFFERSIZE, 2);   // reduce latency
        }

        std::cout << "[CAM] Opened both cameras @ " << width_ << "×" << height_ << "\n";
        return true;
    }

    void captureLoop() {
        Mat l, r;
        while (g_running) {
            cap0_ >> l;
            cap1_ >> r;
            if (l.empty() || r.empty()) continue;
            g_frameBuf.write(l, r);
        }
    }

    void close() { cap0_.release(); cap1_.release(); }

private:
    VideoCapture cap0_, cap1_;
    int width_, height_;
};

// ── FPS counter ───────────────────────────────────────────────────────────────

class FpsCounter {
public:
    void tick() {
        auto now = steady_clock::now();
        frames_++;
        double elapsed = duration<double>(now - last_).count();
        if (elapsed >= 1.0) {
            fps_   = frames_ / elapsed;
            frames_ = 0;
            last_  = now;
        }
    }
    double fps() const { return fps_; }

private:
    int frames_{0};
    double fps_{0.0};
    steady_clock::time_point last_ = steady_clock::now();
};

// ── main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // ── Config ──
    AppConfig cfg;
    cfg.calibFile      = "stereo_calib.yml";
    cfg.width          = 1280;
    cfg.height         = 720;
    cfg.showRectified  = false;
    cfg.useWLS         = false;   // set true if opencv-contrib installed
    cfg.saveDepth      = false;

    // Simple CLI parsing
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if      (arg == "--480p")         { cfg.width = 640;  cfg.height = 480; }
        else if (arg == "--720p")         { cfg.width = 1280; cfg.height = 720; }
        else if (arg == "--1080p")        { cfg.width = 1920; cfg.height = 1080; }
        else if (arg == "--show-rect")    { cfg.showRectified = true; }
        else if (arg == "--wls")          { cfg.useWLS = true; }
        else if (arg == "--save-depth")   { cfg.saveDepth = true; }
        else if (arg == "--calib" && i+1 < argc) { cfg.calibFile = argv[++i]; }
    }

    Size imgSize(cfg.width, cfg.height);
    std::cout << "[INIT] Resolution: " << cfg.width << "×" << cfg.height << "\n";

    // ── Load calibration ──
    StereoCalib cal;
    if (!loadCalibration(cfg.calibFile, cal, imgSize)) {
        std::cerr << "[INIT] Run calibrate.cpp first to generate " << cfg.calibFile << "\n";
        return 1;
    }

    // ── Build stereo matcher ──
    SGBMConfig sgbmCfg;
    sgbmCfg.numDisparities   = 96;   // must be divisible by 16
    sgbmCfg.blockSize        = 7;
    sgbmCfg.minDisparity     = 0;
    sgbmCfg.disp12MaxDiff    = 1;
    sgbmCfg.preFilterCap     = 63;
    sgbmCfg.uniquenessRatio  = 10;
    sgbmCfg.speckleWindowSize = 100;
    sgbmCfg.speckleRange     = 32;

    Ptr<StereoSGBM> stereo = buildSGBM(sgbmCfg);

    // Optional WLS filter for smoother edges (requires opencv-contrib)
    Ptr<ximgproc::DisparityWLSFilter> wls;
    Ptr<StereoMatcher> rightMatcher;
    if (cfg.useWLS) {
        wls          = ximgproc::createDisparityWLSFilter(stereo);
        rightMatcher = ximgproc::createRightMatcher(stereo);
        wls->setLambda(8000.0);
        wls->setSigmaColor(1.5);
        std::cout << "[INIT] WLS filter enabled\n";
    }

    // ── Open cameras ──
    DualCameraCapture cameras(cfg.width, cfg.height);
    if (!cameras.open()) return 1;

    // Capture on a dedicated thread — runs independently of compute
    std::thread captureThread([&cameras]{ cameras.captureLoop(); });

    // ── Main compute loop ──
    std::cout << "[RUN]  Pipeline running. Press 'q' to quit.\n";

    Mat left_raw, right_raw;
    Mat left_rect, right_rect;
    Mat left_gray, right_gray;
    Mat disparity, disparity_right;
    Mat disparity_filtered;
    Mat depth_colored, overlay;

    FpsCounter fps;
    FramePair  frame;
    int saveIdx = 0;

    while (g_running) {
        if (!g_frameBuf.read(frame)) continue;

        left_raw  = frame.left;
        right_raw = frame.right;

        // 1. Rectify
        remap(left_raw,  left_rect,  cal.map1x, cal.map1y, INTER_LINEAR);
        remap(right_raw, right_rect, cal.map2x, cal.map2y, INTER_LINEAR);

        // 2. Greyscale for matcher
        cvtColor(left_rect,  left_gray,  COLOR_BGR2GRAY);
        cvtColor(right_rect, right_gray, COLOR_BGR2GRAY);

        // 3. Compute disparity
        stereo->compute(left_gray, right_gray, disparity);

        // 4. Optional WLS filter
        if (cfg.useWLS && wls) {
            rightMatcher->compute(right_gray, left_gray, disparity_right);
            wls->filter(disparity, left_rect, disparity_filtered, disparity_right);
        } else {
            disparity_filtered = disparity;
        }

        // 5. Normalise + colourmap for display
        Mat disp_norm;
        normalize(disparity_filtered, disp_norm, 0, 255, NORM_MINMAX, CV_8U);
        applyColorMap(disp_norm, depth_colored, COLORMAP_INFERNO);

        // 6. FPS overlay
        fps.tick();
        std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps.fps()));
        putText(depth_colored, fpsText, Point(12, 30),
                FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,255,255), 2);

        // 7. Display
        imshow("Depth map (C++)", depth_colored);
        if (cfg.showRectified) {
            Mat side_by_side;
            hconcat(left_rect, right_rect, side_by_side);
            // Draw epipolar lines (should be horizontal after rectification)
            for (int y = 0; y < side_by_side.rows; y += 40)
                line(side_by_side, Point(0,y), Point(side_by_side.cols,y),
                     Scalar(0,255,0), 1);
            imshow("Rectified (epipolar lines = horizontal)", side_by_side);
        }

        // 8. Optionally save depth frames
        if (cfg.saveDepth) {
            // Save float disparity as 16-bit PNG (divide by 16 to get real disparity)
            Mat disp16;
            disparity_filtered.convertTo(disp16, CV_16U);
            imwrite("depth_" + std::to_string(saveIdx++) + ".png", disp16);
        }

        char key = (char)waitKey(1);
        if (key == 'q' || key == 27) g_running = false;
        if (key == 's') {
            imwrite("snapshot_left.jpg",  left_rect);
            imwrite("snapshot_right.jpg", right_rect);
            imwrite("snapshot_depth.jpg", depth_colored);
            std::cout << "[SNAP] Saved rectified pair + depth map\n";
        }
        if (key == 'r') {
            cfg.showRectified = !cfg.showRectified;
            if (!cfg.showRectified) destroyWindow("Rectified (epipolar lines = horizontal)");
        }
    }

    cameras.close();
    captureThread.join();
    destroyAllWindows();
    std::cout << "[DONE] Pipeline stopped.\n";
    return 0;
}
