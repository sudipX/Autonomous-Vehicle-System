# AutoTrack : Autonomous Self-Driving Car System

AutoTrack is a Raspberry Pi-based autonomous vehicle that uses computer vision and machine learning to navigate in real time. It detects lane markings, reads traffic signs, and avoids obstacles, all processed on a Raspberry Pi 4B and executed through an Arduino-controlled motor system.

A full project report with system design, methodology, and results is available as a separate PDF document.

---

## How It Works

The car captures a live feed from a USB webcam. Each frame is processed to detect lane lines using OpenCV's Hough Line Transform. Based on the lane positions, the system decides whether to steer left, right, or go straight. Simultaneously, a fine-tuned YOLOv11 model scans for traffic signs (Stop, Speed Limit 40, Speed Limit 70) and issues the appropriate commands. Ultrasonic sensor data from the Arduino is read in a background thread, if an obstacle is detected, the vehicle halts immediately.

All decisions are sent to the Arduino as short text commands over UART serial, which then drives the motors accordingly.

---

## Modules

**`main_controller.py`** is the entry point. It ties everything together, opens the camera and serial connection, runs the perception loop, and handles graceful shutdown.

**`lane_detection.py`** handles the full lane detection pipeline: edge detection, ROI masking, Hough line detection, slope-based lane classification, and stateful directional decision logic. It can also be run standalone for testing.

**`sign_detection.py`** wraps the YOLOv11 model for traffic sign inference. It filters low-confidence detections, maps class labels to actions, and includes a debounce timer for stop signs to prevent repeated triggers.

**`serial_comm.py`** manages UART communication with the Arduino. It is thread-safe, auto-detects the Arduino port, validates commands, and falls back to mock mode if `pyserial` is not installed.

---

## Hardware

- Raspberry Pi 4B
- Arduino Uno
- USB Webcam
- HC-SR04 Ultrasonic Sensor
- L298N Motor Driver

---

## Dependencies

```bash
pip install opencv-python numpy pyserial ultralytics
```

Place the fine-tuned YOLOv11 weights at `models/yolov11_traffic_signs.pt` before running sign detection. If the model file is missing, the system will continue operating using lane detection only.

---

## Running the System

```bash
python main_controller.py
python main_controller.py --port /dev/ttyUSB0
python main_controller.py --no-preview   # headless mode
```

Press `q` to quit when running with preview windows.

---

## Serial Commands

The Raspberry Pi sends these commands to the Arduino over serial: `left`, `right`, `straight`, `stop`, `speed_40`, `speed_70`.

The Arduino sends back plain-text lines. Any line containing `"obstacle detected"` triggers an immediate stop.

---

## Project Structure

```
autotrack/
├── modules
    ├── main_controller.py
    ├── lane_detection.py
    ├── sign_detection.py
    ├── serial_comm.py
├── report.pdf
```
