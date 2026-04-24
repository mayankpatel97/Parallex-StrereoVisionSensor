# capture_stereo.py

import cv2
import os

os.makedirs("images", exist_ok=True)

left_pipeline = "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@1/ov5647@36 ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! appsink"
right_pipeline = "libcamerasrc camera-name=/base/soc/i2c0mux/i2c@0/ov5647@36 ! video/x-raw,width=640,height=480,format=BGR ! videoconvert ! appsink"

capL = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
capR = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

count = 0

print("Press 's' to save images, ESC to exit")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if not retL or not retR:
        print("Camera error")
        break

    combined = cv2.hconcat([frameL, frameR])
    cv2.imshow("Stereo Capture", combined)

    key = cv2.waitKey(1)

    if key == ord('s'):
        cv2.imwrite(f"images/left_{count}.png", frameL)
        cv2.imwrite(f"images/right_{count}.png", frameR)
        print(f"Saved pair {count}")
        count += 1

    elif key == 27:
        break

capL.release()
capR.release()
cv2.destroyAllWindows()