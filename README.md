# CV_Drone

An autonomous drone control system integrating computer vision with MAVLink-based flight control on Raspberry Pi.

## Overview

CV_Drone combines real-time computer vision using YOLO models with autonomous flight control via DroneKit and the MAVLink protocol. The system runs on a Raspberry Pi mounted on a drone, providing object detection, live video streaming, telemetry monitoring, and thermal management.

## Features

### Computer Vision
- Real-time object detection using YOLOv8n and YOLO-E 11s segmentation models
- Live video processing at 800x800 resolution with FPS monitoring
- Flask-based MJPEG streaming for remote viewing
- Inference optimization for Raspberry Pi hardware

### Autonomous Flight
- DroneKit Python API for MAVLink communication with ArduPilot/PX4 flight controllers
- SITL (Software-in-the-loop) simulation support for testing
- Programmed autonomous maneuvers including takeoff, navigation, and landing
- Body-frame velocity control for precise movement

### Telemetry & Monitoring
- Data logging for GPS, attitude, velocity, battery, and sensors
- Configurable message streaming rates up to 10Hz
- CSV export for post-flight analysis
- EKF (Extended Kalman Filter) status monitoring

### System Management
- Automatic fan control with temperature-based hysteresis
- GPIO hardware interfacing via gpiozero
- ARM architecture optimization

## Architecture

```
┌─────────────────────────────────────────────────┐
│             Raspberry Pi                         │
│  ┌──────────────┐         ┌──────────────┐     │
│  │  PiCamera2   │────────▶│  YOLO Model  │     │
│  │  (800x800)   │         │  Inference   │     │
│  └──────────────┘         └──────────────┘     │
│         │                        │              │
│         ▼                        ▼              │
│  ┌──────────────────────────────────────┐      │
│  │     Flask Web Server / CV Demo       │      │
│  │   (Real-time streaming + overlay)    │      │
│  └──────────────────────────────────────┘      │
│                                                  │
│  ┌──────────────────────────────────────┐      │
│  │      DroneKit / MAVLink Layer        │      │
│  │  (Telemetry + Flight Commands)       │      │
│  └──────────────────────────────────────┘      │
│         │                                       │
└─────────┼───────────────────────────────────────┘
          │ UDP/Serial
          ▼
┌─────────────────────────┐
│   Flight Controller     │
│   (ArduPilot/PX4)       │
│   - Real hardware       │
│   - SITL simulation     │
└─────────────────────────┘
```

## Technologies

### Core
- Python 3.x
- DroneKit - Python API for MAVLink vehicle control
- Ultralytics YOLO - Object detection framework
- OpenCV - Computer vision library
- Flask - Web framework for video streaming

### Hardware
- PiCamera2 - Raspberry Pi camera interface
- gpiozero - GPIO control library
- MAVLink - Micro Air Vehicle communication protocol

### Development
- SITL (Software In The Loop) - Flight simulation
- ArduPilot/PX4 - Open-source autopilot firmware

## Hardware Requirements

### Required
- Raspberry Pi 4/5 (4GB+ RAM recommended)
- Raspberry Pi Camera Module (v2 or HQ camera)
- Flight Controller with MAVLink support (Pixhawk, Cube, etc.)
- Drone frame with motors, ESCs, and power system

### Optional
- Cooling fan with transistor/relay
- GPS module (typically included with flight controller)
- Telemetry radio for long-range communication

## Installation

### 1. System Setup (Raspberry Pi)

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-opencv libcamera-dev
```

### 2. Python Dependencies

```bash
# Install DroneKit and MAVLink
pip3 install dronekit pymavlink

# Install Computer Vision dependencies
pip3 install ultralytics picamera2 opencv-python

# Install web server
pip3 install flask

# Install hardware control
pip3 install gpiozero RPi.GPIO
```

### 3. YOLO Models

Download pre-trained models:
```bash
# YOLOv8 nano for general object detection
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# YOLO-E 11s segmentation model (if needed)
# Model file: yoloe-11s-seg.pt
```

### 4. Hardware Connections

Flight Controller to Raspberry Pi:
- USB: `/dev/ttyUSB0` or `/dev/ttyACM0`
- UART (GPIO): `/dev/ttyAMA0` or `/dev/serial0`
- Network: UDP connection for SITL or wireless telemetry

Camera: Connect via CSI ribbon cable

Fan (optional): GPIO 14 via transistor/relay

## Usage

### Computer Vision Demo (Local Display)

Real-time object detection with local display:
```bash
python3 cv_demo.py
```
Press 'q' to quit. Shows FPS overlay and bounding boxes.

### Web Streaming Server

Stream video with YOLO inference to a web browser:
```bash
python3 app.py
```
Access at `http://<raspberry-pi-ip>:8000` or view stream at `http://<raspberry-pi-ip>:8000/video`

### Autonomous Flight Demo

```bash
# Connect to real drone
python3 auto_demo.py
```

The script connects to the flight controller via UDP, arms and takes off to 10 meters, flies forward at 1 m/s for 5 seconds, then lands autonomously.

WARNING: Test in SITL simulation first. Ensure adequate space and safety measures before flying real hardware.

### Telemetry Monitoring

Monitor drone state and log telemetry data:
```bash
# Real flight controller over USB
python3 autonomous_telemetry.py --conn /dev/ttyUSB0 --baud 115200 --csv telem.csv

# SITL simulation
python3 autonomous_telemetry.py --conn udp:0.0.0.0:14550 --csv telem.csv

# Custom update rate
python3 autonomous_telemetry.py --conn /dev/ttyUSB0 --hz 20 --csv flight_data.csv
```

Output includes GPS fix, satellite count, position, attitude (roll/pitch/yaw), velocity, groundspeed, heading, battery voltage/current, rangefinder distance, and EKF status.

### SITL Simulation Testing

```bash
# Terminal 1: Start SITL
dronekit-sitl copter --home=<LAT>,<LON>,<ALT>,<HEADING>

# Terminal 2: Run basic connection test
python3 drone.py

# Or run telemetry monitor
python3 autonomous_telemetry.py --conn udp:0.0.0.0:14550
```

### Thermal Management

Run fan control daemon:
```bash
python3 fan_control.py
```
Turns fan on at 45°C and off at 40°C. Logs state changes and periodic status.

## Project Structure

```
CV_Drone/
├── app.py                      # Flask web server for YOLO video streaming
├── cv_demo.py                  # Local CV demo with YOLO detection
├── auto_demo.py                # Autonomous flight demonstration
├── autonomous_telemetry.py     # Comprehensive telemetry monitoring & control
├── drone.py                    # Basic DroneKit SITL test
├── drone_kit_test.py           # DroneKit connection test
├── fan_control.py              # Thermal management for Raspberry Pi
├── yolov8n.pt                  # YOLOv8 nano model weights
├── yoloe-11s-seg.pt           # YOLO-E segmentation model weights
└── telem.csv                   # Telemetry data logs
```

## Technical Implementation

### Software Architecture
- Modular design separating computer vision, flight control, and hardware management
- Error handling with timeouts and graceful degradation
- Command-line configuration via argparse
- CSV-based data persistence for telemetry

### Robotics & Control
- MAVLink protocol implementation for drone communication
- State machine management for flight modes
- EKF sensor fusion monitoring
- High-rate data streaming and processing

### Computer Vision
- Optimized YOLO model deployment on embedded hardware
- Real-time frame capture, inference, and display pipeline
- FPS monitoring and performance tuning

### Hardware Integration
- Direct GPIO control for peripherals
- CSI camera interface via PiCamera2
- Serial/UART communication with flight hardware

## Future Work

- Vision-based navigation using object detection for autonomous waypoints
- Collision avoidance integrating obstacle detection with flight control
- Complex mission planning with CV-triggered events
- Multi-drone coordination with visual markers
- Follow-me mode using person detection
- Gimbal control for camera stabilization
- Remote monitoring dashboard
- ROS 2 integration

## Performance

### YOLO Inference (Raspberry Pi 4)
- YOLOv8n: ~5-8 FPS at 320x320 input
- YOLO-E 11s-seg: ~3-5 FPS at 800x800 input
- Latency: 125-200ms per frame

### Telemetry Rates
- Position/Attitude: Up to 10 Hz
- GPS/Health: 2 Hz
- CSV logging: Real-time with minimal overhead

## Safety

Important considerations for UAV operation:

1. Always test in SITL simulation before deploying to real hardware
2. Maintain visual line of sight during flight operations
3. Follow local regulations for UAV operation
4. Use a safety pilot with manual override capability
5. Test in open areas away from people and obstacles
6. Monitor battery levels - autonomous landing triggers at low voltage
7. Verify GPS lock before arming (6+ satellites recommended)
8. Implement failsafes such as RTL (Return to Launch) on signal loss

## Author

Felix Lu

## License

This project is provided as-is for educational and demonstration purposes.

## Acknowledgments

- DroneKit-Python for MAVLink abstraction
- Ultralytics for YOLO implementation
- ArduPilot open-source autopilot
- Raspberry Pi Foundation
