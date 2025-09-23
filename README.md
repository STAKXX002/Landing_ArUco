# ArUco-Based Autonomous Landing – DroneKit + OpenCV

This project enables a drone to autonomously detect and land on an ArUco marker using a Raspberry Pi camera (or USB webcam), OpenCV, and DroneKit.

---

## Folder Structure

```
aruco_landing_project/
├── scripts/
│   └── aruco_landing.py            # Main landing logic
├── opencv/
│   ├── lib_aruco_pose.py           # ArUco marker detection and pose estimation
│   ├── cameraMatrix_raspi.txt      # Camera calibration matrix
│   └── cameraDistortion_raspi.txt  # Camera distortion coefficients
```

---

## System Requirements

- Raspberry Pi or companion computer (e.g., Jetson Nano)
- Camera module (Pi Camera or USB webcam)
- Flight controller (e.g., Pixhawk running ArduPilot or PX4)
- Python 3.7–3.9 (recommended)
- Linux OS (Raspberry Pi OS or Ubuntu recommended)

---

## Installation

### Step 1: Install Dependencies

```bash
sudo apt update
sudo apt install python3-pip python3-opencv -y
pip3 install dronekit pymavlink opencv-contrib-python numpy
```

### Step 2: Enable the Camera

For Raspberry Pi:

```bash
sudo modprobe bcm2835-v4l2
```

To make this persistent, add it to `/etc/rc.local` or a custom `systemd` service.

---

## Setup and Configuration

### 1. Hardware Connection

Connect your flight controller to the Pi using:

- `/dev/ttyAMA0` (GPIO UART, commonly used), or
- `/dev/ttyUSB0` (USB FTDI cable)

Update the `--connect` argument when running the script if needed.

---

### 2. ArUco Marker

- Marker ID used: `72`
- Physical marker size: `10 cm`
- Place the marker flat on the ground, facing up.

You can generate ArUco markers using this online tool:  
https://chev.me/arucogen/

---

### 3. Running the Script

From inside your project directory:

```bash
python3 scripts/aruco_landing.py
```

The drone must already be flying in `GUIDED` mode.  
This script will detect the ArUco marker and guide the drone to align over it and descend gradually. Once it reaches a specified altitude and is well-aligned, it will trigger `LAND` mode.

---

## Configuration Parameters

You can change the behavior using command-line arguments:

| Argument             | Description                                  | Default Value         |
|----------------------|----------------------------------------------|------------------------|
| `--connect`          | MAVLink connection string                    | `/dev/ttyAMA0`         |
| `--baud`             | Baud rate for serial connection              | `57600`                |
| `--aruco-id`         | ID of the ArUco marker to track              | `72`                   |
| `--marker-size`      | Physical marker size in centimeters          | `10`                   |
| `--land-alt-cm`      | Trigger LAND mode below this altitude (cm)   | `50`                   |
| `--freq-hz`          | Command update frequency in Hz               | `1`                    |
| `--angle-thresh-deg` | Max error in degrees to allow descent        | `20`                   |
| `--calib-path`       | Path to calibration files                    | `opencv`               |
| `--show-video`       | Enable video feed during operation           | `False`                |

---

## Camera Calibration

Accurate pose estimation depends on correct camera calibration.

Make sure the following files match your camera:

- `opencv/cameraMatrix_raspi.txt`
- `opencv/cameraDistortion_raspi.txt`

If not, perform camera calibration using OpenCV tools and update these files.

---

## Troubleshooting

| Problem                        | Cause or Fix                                               |
|----------------------------------|----------------------------------------------------------|
| `ModuleNotFoundError: cv2.aruco` | Install `opencv-contrib-python`                          |
| Marker not detected              | Wrong ID or incorrect calibration                        |
| Drone doesn't descend            | Check angle threshold or marker alignment                |
| Connection errors                | Check port (e.g., `/dev/ttyAMA0`) and drone power        |
| Camera not found                 | Run `sudo modprobe bcm2835-v4l2` on Raspberry Pi         |

---

## Testing Without Drone (SITL)

You can use PX4 or ArduPilot SITL for testing:

```bash
python3 scripts/aruco_landing.py --connect 127.0.0.1:14550
```

---

## License

Copyright (c) 2025 Kamal Kumar Behera

All rights reserved.

This project is private and not licensed for copying, modification, or redistribution.  
You cannot use it in any form without explicit authorization from the author.
