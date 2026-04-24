# stereo_depth.py

import cv2

# Load calibration
fs = cv2.FileStorage("stereo.yml", cv2.FILE_STORAGE_READ)

mtxL = fs.getNode("mtxL").mat()
distL = fs.getNode("distL").mat()
mtxR = fs.getNode("mtxR").mat()
distR = fs.getNode("distR").mat()
R = fs.getNode("R").mat()
T = fs.getNode("T").mat()

w, h = 640, 480

R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(mtxL, distL, mtxR, distR, (w,h), R, T)

mapLx, mapLy = cv2.initUndistortRectifyMap(mtxL, distL, R1, P1, (w,h), cv2.CV_32FC1)
mapRx, mapRy = cv2.initUndistortRectifyMap(mtxR, distR, R2, P2, (w,h), cv2.CV_32FC1)

stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=64,
    blockSize=5
)

# Camera pipelines
left_pipeline = "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@1/ov5647@36 ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! appsink"
right_pipeline = "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@0/ov5647@36 ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! appsink"

capL = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
capR = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if not retL or not retR:
        print("Camera error")
        break

    rectL = cv2.remap(frameL, mapLx, mapLy, cv2.INTER_LINEAR)
    rectR = cv2.remap(frameR, mapRx, mapRy, cv2.INTER_LINEAR)

    grayL = cv2.cvtColor(rectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(rectR, cv2.COLOR_BGR2GRAY)

    disparity = stereo.compute(grayL, grayR)

    cv2.imshow("Disparity", disparity / 64.0)

    if cv2.waitKey(1) == 27:
        break

capL.release()
capR.release()
cv2.destroyAllWindows()