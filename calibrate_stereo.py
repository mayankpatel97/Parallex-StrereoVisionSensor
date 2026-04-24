# calibrate_stereo.py

import cv2
import numpy as np
import glob

CHECKERBOARD = (9, 6)
square_size = 0.025  # meters (25mm)

objp = np.zeros((CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0],0:CHECKERBOARD[1]].T.reshape(-1,2)
objp *= square_size

objpoints = []
imgpointsL = []
imgpointsR = []

imagesL = sorted(glob.glob('images/left_*.png'))
imagesR = sorted(glob.glob('images/right_*.png'))

for imgL, imgR in zip(imagesL, imagesR):
    imgL = cv2.imread(imgL)
    imgR = cv2.imread(imgR)

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, CHECKERBOARD)
    retR, cornersR = cv2.findChessboardCorners(grayR, CHECKERBOARD)

    if retL and retR:
        objpoints.append(objp)
        imgpointsL.append(cornersL)
        imgpointsR.append(cornersR)

ret, mtxL, distL, mtxR, distR, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpointsL, imgpointsR,
    None, None, None, None,
    grayL.shape[::-1]
)

print("RMS Error:", ret)

# Save parameters
fs = cv2.FileStorage("stereo.yml", cv2.FILE_STORAGE_WRITE)
fs.write("mtxL", mtxL)
fs.write("distL", distL)
fs.write("mtxR", mtxR)
fs.write("distR", distR)
fs.write("R", R)
fs.write("T", T)
fs.release()

print("Calibration saved to stereo.yml")