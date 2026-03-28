/**
 * calibrate.cpp
 * Stereo camera calibration — captures chessboard pairs and computes
 * the stereo calibration parameters, saving them to stereo_calib.yml
 *
 * Usage:
 *   ./calibrate [--cols 9] [--rows 6] [--square 25.0] [--images ./calib]
 *
 * If --images is a directory: reads left_NN.jpg + right_NN.jpg pairs.
 * If --images is omitted:     captures live from both cameras interactively.
 */

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <iomanip>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────

struct Config {
    int    cols        = 9;       // inner corners (columns)
    int    rows        = 6;       // inner corners (rows)
    float  squareMM    = 25.0f;   // physical square size in mm
    string imageDir    = "";      // empty = live capture mode
    string outputFile  = "stereo_calib.yml";
    int    camWidth    = 1280;
    int    camHeight   = 720;
};

// ─────────────────────────────────────────────────────────────────────────────

void drawChessboard(Mat& img, const Size& boardSize,
                    const vector<Point2f>& corners, bool found) {
    drawChessboardCorners(img, boardSize, corners, found);
}

bool findBoard(const Mat& gray, const Size& boardSize,
               vector<Point2f>& corners) {
    bool found = findChessboardCorners(gray, boardSize, corners,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK);
    if (found) {
        cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
            TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));
    }
    return found;
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    Config cfg;

    for (int i = 1; i < argc; ++i) {
        string a = argv[i];
        if      (a == "--cols"   && i+1<argc) cfg.cols       = stoi(argv[++i]);
        else if (a == "--rows"   && i+1<argc) cfg.rows       = stoi(argv[++i]);
        else if (a == "--square" && i+1<argc) cfg.squareMM   = stof(argv[++i]);
        else if (a == "--images" && i+1<argc) cfg.imageDir   = argv[++i];
        else if (a == "--output" && i+1<argc) cfg.outputFile = argv[++i];
        else if (a == "--width"  && i+1<argc) cfg.camWidth   = stoi(argv[++i]);
        else if (a == "--height" && i+1<argc) cfg.camHeight  = stoi(argv[++i]);
    }

    Size boardSize(cfg.cols, cfg.rows);

    // Build object points (same for every frame)
    vector<Point3f> objp;
    for (int r = 0; r < cfg.rows; ++r)
        for (int c = 0; c < cfg.cols; ++c)
            objp.emplace_back(c * cfg.squareMM, r * cfg.squareMM, 0.0f);

    vector<vector<Point3f>> objPoints;
    vector<vector<Point2f>> imgPointsL, imgPointsR;
    Size imgSize;

    // ── Load or capture image pairs ──
    if (!cfg.imageDir.empty()) {
        // --- Load from directory ---
        cout << "[CALIB] Loading images from: " << cfg.imageDir << "\n";

        vector<string> leftFiles, rightFiles;
        for (auto& entry : fs::directory_iterator(cfg.imageDir)) {
            string name = entry.path().filename().string();
            if (name.find("left_") == 0)  leftFiles.push_back(entry.path().string());
            if (name.find("right_") == 0) rightFiles.push_back(entry.path().string());
        }
        sort(leftFiles.begin(), leftFiles.end());
        sort(rightFiles.begin(), rightFiles.end());

        if (leftFiles.size() != rightFiles.size() || leftFiles.empty()) {
            cerr << "[CALIB] Need matching left_NN.jpg / right_NN.jpg pairs in: "
                 << cfg.imageDir << "\n";
            return 1;
        }

        for (size_t i = 0; i < leftFiles.size(); ++i) {
            Mat imgL = imread(leftFiles[i]);
            Mat imgR = imread(rightFiles[i]);
            if (imgL.empty() || imgR.empty()) continue;

            Mat gL, gR;
            cvtColor(imgL, gL, COLOR_BGR2GRAY);
            cvtColor(imgR, gR, COLOR_BGR2GRAY);
            imgSize = gL.size();

            vector<Point2f> cL, cR;
            bool okL = findBoard(gL, boardSize, cL);
            bool okR = findBoard(gR, boardSize, cR);

            if (okL && okR) {
                objPoints.push_back(objp);
                imgPointsL.push_back(cL);
                imgPointsR.push_back(cR);
                cout << "  [OK] " << leftFiles[i] << "\n";
            } else {
                cout << "  [--] Skipped (board not found in both)\n";
            }
        }

    } else {
        // --- Live capture mode ---
        cout << "[CALIB] Live capture mode. Press SPACE to capture, 'q' when done.\n";

        VideoCapture cap0(0, CAP_V4L2), cap1(2, CAP_V4L2);
        if (!cap0.isOpened() || !cap1.isOpened()) {
            cerr << "[CALIB] Cannot open cameras.\n"; return 1;
        }
        for (auto* c : {&cap0, &cap1}) {
            c->set(CAP_PROP_FRAME_WIDTH,  cfg.camWidth);
            c->set(CAP_PROP_FRAME_HEIGHT, cfg.camHeight);
        }

        fs::create_directories("calib");
        int idx = 0;

        while (true) {
            Mat l, r;
            cap0 >> l; cap1 >> r;
            if (l.empty() || r.empty()) continue;

            Mat gL, gR;
            cvtColor(l, gL, COLOR_BGR2GRAY);
            cvtColor(r, gR, COLOR_BGR2GRAY);
            imgSize = gL.size();

            vector<Point2f> cL, cR;
            bool okL = findBoard(gL, boardSize, cL);
            bool okR = findBoard(gR, boardSize, cR);

            Mat disp;
            Mat lViz = l.clone(), rViz = r.clone();
            if (okL) drawChessboard(lViz, boardSize, cL, okL);
            if (okR) drawChessboard(rViz, boardSize, cR, okR);
            hconcat(lViz, rViz, disp);

            string status = (okL && okR) ? "READY - press SPACE to capture" : "searching...";
            putText(disp, status + "  [" + to_string(idx) + " pairs]",
                    Point(10,30), FONT_HERSHEY_SIMPLEX, 0.7,
                    (okL&&okR) ? Scalar(0,200,0) : Scalar(0,100,200), 2);

            imshow("Calibration capture", disp);
            char key = (char)waitKey(1);

            if (key == 'q') break;
            if ((key == ' ') && okL && okR) {
                objPoints.push_back(objp);
                imgPointsL.push_back(cL);
                imgPointsR.push_back(cR);
                imwrite("calib/left_"  + to_string(idx) + ".jpg", l);
                imwrite("calib/right_" + to_string(idx) + ".jpg", r);
                cout << "  [OK] Captured pair " << idx++ << "\n";
            }
        }
        destroyAllWindows();
        cap0.release(); cap1.release();
    }

    if (objPoints.size() < 10) {
        cerr << "[CALIB] Need at least 10 valid pairs, got " << objPoints.size() << "\n";
        return 1;
    }
    cout << "[CALIB] Using " << objPoints.size() << " valid pairs.\n";

    // ── Run stereo calibration ──
    Mat K1, D1, K2, D2, R, T, E, F;

    double rms = stereoCalibrate(
        objPoints, imgPointsL, imgPointsR,
        K1, D1, K2, D2, imgSize,
        R, T, E, F,
        CALIB_FIX_INTRINSIC,
        TermCriteria(TermCriteria::COUNT | TermCriteria::EPS, 200, 1e-7)
    );

    cout << fixed << setprecision(4);
    cout << "[CALIB] RMS reprojection error: " << rms << " px";
    if (rms < 0.5)       cout << "  (excellent)\n";
    else if (rms < 1.0)  cout << "  (good)\n";
    else if (rms < 2.0)  cout << "  (acceptable)\n";
    else                 cout << "  (poor — recalibrate)\n";

    // ── Stereo rectification ──
    Mat R1, R2, P1, P2, Q;
    Rect validROI1, validROI2;
    stereoRectify(K1, D1, K2, D2, imgSize, R, T,
                  R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1.0, imgSize,
                  &validROI1, &validROI2);

    // ── Save to YAML ──
    FileStorage fs_out(cfg.outputFile, FileStorage::WRITE);
    fs_out << "imageSize_width"  << imgSize.width
           << "imageSize_height" << imgSize.height
           << "K1" << K1 << "D1" << D1
           << "K2" << K2 << "D2" << D2
           << "R"  << R  << "T"  << T
           << "R1" << R1 << "R2" << R2
           << "P1" << P1 << "P2" << P2
           << "Q"  << Q
           << "rms" << rms;
    fs_out.release();

    cout << "[CALIB] Saved to: " << cfg.outputFile << "\n";
    return 0;
}
