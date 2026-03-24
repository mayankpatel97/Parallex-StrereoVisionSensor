# 🚀 Stereo Vision System using Raspberry Pi CM4

A real-time stereo vision system designed for **depth estimation and obstacle avoidance**, optimized for embedded platforms like the **Compute Module 4 (CM4)**.

---

# 📌 Overview

This project implements a **dual-camera stereo vision pipeline** that computes depth from two synchronized images. It is suitable for:

* Drone obstacle avoidance
* Robotics navigation
* 3D perception systems

---

# 🧠 System Architecture

```
[Left Camera] ─┐
               ├──> [CM4] → [Stereo Processing] → [Depth Map]
[Right Camera] ┘                         ↓
                                    [Obstacle Detection]
                                            ↓
                                    [Flight Controller]
```

---

# 📷 Hardware Setup

## Components

* Raspberry Pi Compute Module 4 (CM4)
* 2 × CSI Cameras (e.g., Camera Module 3)
* CM4 IO Board / Custom carrier board
* Rigid stereo mount frame

---

## 📸 Stereo Camera Setup

![Image](https://www.waveshare.com/img/devkit/CM4-DUAL-CAMERA-BASE/CM4-DUAL-CAMERA-BASE-3_460.jpg)

![Image](https://www.fabtolab.com/image/cache/catalog/Accessory/IO%20devices/Optical%20devices/Cameras%20for%20SBCs/b0265r_1_-500x500.png)

![Image](https://www.researchgate.net/publication/2704594/figure/fig5/AS%3A668229852745737%401536329820462/A-general-view-of-a-stereo-camera-set-up-where-the-baseline-b-is-parallel-to-the-ground.png)

![Image](https://us1.discourse-cdn.com/flex020/uploads/opencv/original/2X/f/fce35ae64dab57111d6e02731600e39135f0d742.jpeg)

---

## 📏 Baseline (Distance Between Cameras)

| Use Case     | Recommended Baseline |
| ------------ | -------------------- |
| Indoor Robot | 6–8 cm               |
| Drone        | 8–12 cm              |
| Long Range   | 15–25 cm             |

👉 Recommended: **8–10 cm**

---

## ⚠️ Mounting Guidelines

* Cameras must be **perfectly parallel**
* Same height and orientation
* Use **rigid frame (no vibration)**

---

# ⚙️ Software Stack

* OS: Linux (Raspberry Pi OS / Yocto)
* Camera Interface: `libcamera`
* Processing: `OpenCV (C++)`
* Optional: MAVLink for drone integration

---

# 🧮 Stereo Vision Pipeline

---

## 1️⃣ Image Capture

* Capture frames from both cameras
* Ensure synchronization (critical)

---

## 2️⃣ Camera Calibration

Use OpenCV calibration tools:

```
calibrateCamera()
stereoCalibrate()
```

Outputs:

* Intrinsic parameters (K1, K2)
* Extrinsic parameters (R, T)

---

## 3️⃣ Rectification

Align images horizontally:

```
stereoRectify()
initUndistortRectifyMap()
```

---

## 4️⃣ Disparity Computation

Core depth equation:

$$
Depth = \frac{f \cdot B}{Disparity}
$$

Where:

* **f** = focal length
* **B** = baseline

---

## 5️⃣ Depth Map Generation

![Image](https://user-images.githubusercontent.com/29349268/118022259-c1052580-b38e-11eb-9574-16c3e73d6273.png)

![Image](https://www.researchgate.net/profile/Ali-Amiri-2/publication/333233531/figure/fig1/AS%3A760844371628032%401558410842685/Using-stereo-only-c-leads-to-the-noisy-depth-map-Using-LiDAR-only-d-results-in.jpg)

![Image](https://www.mathworks.com/help/examples/visionhdl/win64/xxSGBM_result2.png)

![Image](https://www.mathworks.com/help/examples/visionhdl/win64/xxSGBM_DisparityLevels.png)

---

### Recommended Algorithm

| Algorithm  | Speed  | Quality |
| ---------- | ------ | ------- |
| StereoBM   | Fast   | Low     |
| StereoSGBM | Medium | Good ✅  |

👉 Use: **StereoSGBM**

---

# 🚧 Obstacle Detection

Basic logic:

```cpp
if (depth < threshold_distance)
{
    obstacle_detected = true;
}
```

---

## Zone-based Detection (Recommended)

Divide image into:

* Left
* Center
* Right

Detect closest obstacle per zone.

---

# 🔌 Integration with Flight Controller

### Options:

* UART communication
* MAVLink (`DISTANCE_SENSOR`)
* Onboard decision making

---

# ⚡ Performance Optimization

* Reduce resolution → 640×480
* Use ROI instead of full frame
* Multi-threading
* Enable NEON optimizations

---

# ⚠️ Challenges & Solutions

| Problem           | Cause            | Solution             |
| ----------------- | ---------------- | -------------------- |
| Bad depth         | Unsynced cameras | Use hardware sync    |
| No depth on walls | Low texture      | Add lidar/ultrasonic |
| Noise in depth    | Poor calibration | Recalibrate          |
| High CPU load     | CM4 limits       | Optimize resolution  |

---

# 🚀 Future Improvements

* Visual-Inertial Odometry (VIO)
* SLAM (ORB-SLAM3)
* AI-based depth estimation
* Upgrade to Jetson platform

---

# 🧪 Development Phases

### Phase 1

* Dual camera capture
* Disparity map generation

### Phase 2

* Obstacle detection
* Flight controller integration

### Phase 3

* SLAM / Visual odometry

---

# 📂 Suggested Project Structure

```
stereo-vision/
│
├── src/
│   ├── capture.cpp
│   ├── calibration.cpp
│   ├── stereo.cpp
│   └── obstacle.cpp
│
├── config/
│   └── calibration.yaml
│
├── scripts/
│   └── calibrate.py
│
├── docs/
│   └── images/
│
└── README.md
```

---

# 🏁 Conclusion

This system provides a **low-cost, scalable stereo vision solution** suitable for drones and robotics. Starting with CM4 allows rapid prototyping, with clear upgrade paths to more powerful platforms.

---

# 👨‍💻 Author

Built for embedded systems, robotics, and drone applications.

---
