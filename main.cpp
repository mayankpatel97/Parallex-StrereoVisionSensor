#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{
    // Open two cameras
    VideoCapture capL(0);
    VideoCapture capR(1);

    if (!capL.isOpened() || !capR.isOpened())
    {
        cerr << "Error: Cannot open cameras" << endl;
        return -1;
    }

    // Set resolution (important for performance)
    capL.set(CAP_PROP_FRAME_WIDTH, 640);
    capL.set(CAP_PROP_FRAME_HEIGHT, 480);
    capR.set(CAP_PROP_FRAME_WIDTH, 640);
    capR.set(CAP_PROP_FRAME_HEIGHT, 480);

    // Stereo matcher (SGBM)
    Ptr<StereoSGBM> stereo = StereoSGBM::create(
        0,      // minDisparity
        96,     // numDisparities (must be multiple of 16)
        5       // blockSize
    );

    stereo->setP1(8 * 3 * 5 * 5);
    stereo->setP2(32 * 3 * 5 * 5);
    stereo->setMode(StereoSGBM::MODE_SGBM);

    Mat frameL, frameR, grayL, grayR;
    Mat disparity, disparity8;

    while (true)
    {
        capL >> frameL;
        capR >> frameR;

        if (frameL.empty() || frameR.empty())
            break;

        // Convert to grayscale
        cvtColor(frameL, grayL, COLOR_BGR2GRAY);
        cvtColor(frameR, grayR, COLOR_BGR2GRAY);

        // Compute disparity
        stereo->compute(grayL, grayR, disparity);

        // Normalize for display
        normalize(disparity, disparity8, 0, 255, NORM_MINMAX, CV_8U);

        imshow("Left", frameL);
        imshow("Right", frameR);
        imshow("Disparity", disparity8);

        if (waitKey(1) == 27) // ESC to exit
            break;
    }

    return 0;
}