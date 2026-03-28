/**
 * depth_query.cpp
 * Utility: click anywhere on the depth window to get the real-world
 * distance in millimetres at that pixel. Useful for debugging and tuning.
 *
 * Usage: ./depth_query [--calib stereo_calib.yml] [--480p|--720p]
 */

#include "stereo_vision.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <atomic>
#include <csignal>

using namespace cv;

static std::atomic<bool> g_running{true};
void signalHandler(int) { g_running = false; }

struct MouseCtx {
    Mat*  depth3D;       // CV_32FC3 from reprojectImageTo3D
    Mat*  display;
    Point click{-1,-1};
};

void onMouse(int event, int x, int y, int, void* userdata) {
    if (event != EVENT_LBUTTONDOWN) return;
    auto* ctx = reinterpret_cast<MouseCtx*>(userdata);
    ctx->click = Point(x, y);
    if (!ctx->depth3D || ctx->depth3D->empty()) return;

    Vec3f pt = ctx->depth3D->at<Vec3f>(y, x);
    float Zmm = pt[2];

    if (std::isinf(Zmm) || Zmm < 0 || Zmm > 20000) {
        std::cout << "[DEPTH] (" << x << "," << y << ") = invalid / no disparity\n";
    } else {
        std::cout << "[DEPTH] (" << x << "," << y << ") = "
                  << static_cast<int>(Zmm) << " mm  ("
                  << static_cast<int>(Zmm/10) << " cm)\n";
    }

    // Draw crosshair on display image
    if (ctx->display && !ctx->display->empty()) {
        circle(*ctx->display, Point(x,y), 6, Scalar(0,255,0), 2);
        std::string label = std::isinf(Zmm) ? "?" :
            std::to_string(static_cast<int>(Zmm)) + "mm";
        putText(*ctx->display, label, Point(x+8,y-8),
                FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0), 2);
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);

    std::string calibFile = "stereo_calib.yml";
    int w = 1280, h = 720;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--480p")                  { w=640;  h=480; }
        else if (a == "--720p")                  { w=1280; h=720; }
        else if (a == "--calib" && i+1 < argc)   { calibFile = argv[++i]; }
    }

    StereoCalib cal;
    FileStorage fs(calibFile, FileStorage::READ);
    if (!fs.isOpened()) { std::cerr << "Cannot open " << calibFile << "\n"; return 1; }
    fs["K1"]>>cal.K1; fs["D1"]>>cal.D1;
    fs["K2"]>>cal.K2; fs["D2"]>>cal.D2;
    fs["R1"]>>cal.R1; fs["R2"]>>cal.R2;
    fs["P1"]>>cal.P1; fs["P2"]>>cal.P2;
    fs["Q"] >>cal.Q;

    Size imgSize(w, h);
    initUndistortRectifyMap(cal.K1,cal.D1,cal.R1,cal.P1,imgSize,CV_32FC1,cal.map1x,cal.map1y);
    initUndistortRectifyMap(cal.K2,cal.D2,cal.R2,cal.P2,imgSize,CV_32FC1,cal.map2x,cal.map2y);

    VideoCapture cap0(0, CAP_V4L2), cap1(2, CAP_V4L2);
    for (auto* c : {&cap0, &cap1}) {
        c->set(CAP_PROP_FRAME_WIDTH, w);
        c->set(CAP_PROP_FRAME_HEIGHT, h);
    }

    auto stereo = StereoSGBM::create(0, 96, 7,
        8*3*49, 32*3*49, 1, 63, 10, 100, 32,
        StereoSGBM::MODE_SGBM_3WAY);

    Mat depth3D, displayImg;
    MouseCtx ctx{ &depth3D, &displayImg };

    namedWindow("Depth query (click to measure)", WINDOW_NORMAL);
    setMouseCallback("Depth query (click to measure)", onMouse, &ctx);

    std::cout << "[QUERY] Click on the depth image to measure distance.\n";

    while (g_running) {
        Mat l, r;
        cap0 >> l; cap1 >> r;
        if (l.empty() || r.empty()) continue;

        Mat lr, rr, lg, rg, disp, norm, colored;
        remap(l, lr, cal.map1x, cal.map1y, INTER_LINEAR);
        remap(r, rr, cal.map2x, cal.map2y, INTER_LINEAR);
        cvtColor(lr, lg, COLOR_BGR2GRAY);
        cvtColor(rr, rg, COLOR_BGR2GRAY);
        stereo->compute(lg, rg, disp);

        // Compute 3D for clicking
        Mat disp32;
        disp.convertTo(disp32, CV_32F, 1.0/16.0);
        reprojectImageTo3D(disp32, depth3D, cal.Q);

        normalize(disp, norm, 0, 255, NORM_MINMAX, CV_8U);
        applyColorMap(norm, colored, COLORMAP_INFERNO);
        displayImg = colored.clone();

        imshow("Depth query (click to measure)", displayImg);
        if (waitKey(1) == 'q') break;
    }

    cap0.release(); cap1.release();
    return 0;
}
