# 🚁 CV_Drone - Computer Vision Enabled Autonomous Drone System

A sophisticated autonomous drone control system integrating computer vision, real-time object detection, and MAVLink-based flight control for Raspberry Pi-powered UAVs.

## 🎯 Project Overview

CV_Drone is a comprehensive autonomous drone platform that combines:
- **Real-time Computer Vision** using YOLO models for object detection and segmentation
- **Autonomous Flight Control** via DroneKit and MAVLink protocol
- **Live Video Streaming** with embedded AI inference
- **Telemetry Monitoring** with high-rate sensor data collection
- **Hardware Management** including thermal control systems

This project demonstrates full-stack robotics development from low-level hardware interfacing to high-level autonomous navigation.

## ✨ Key Features

### 🤖 Computer Vision
- **YOLO-based Object Detection**: Real-time object detection using YOLOv8n and YOLO-E 11s segmentation models
- **Live Video Processing**: 800x800 resolution with FPS monitoring and performance metrics
- **Web Streaming**: Flask-based MJPEG streaming for remote monitoring
- **Optimized Inference**: Tuned for Raspberry Pi hardware with configurable confidence thresholds

### ✈️ Autonomous Flight
- **DroneKit Integration**: Python API for MAVLink communication with ArduPilot/PX4
- **SITL Support**: Software-in-the-loop simulation for safe development and testing
- **Autonomous Maneuvers**: Pre-programmed takeoff, navigation, and landing sequences
- **Body-Frame Velocity Control**: Precise movement commands in drone's reference frame

### 📊 Telemetry & Monitoring
- **Comprehensive Data Logging**: GPS, attitude, velocity, battery, and sensor data
- **High-Rate Streaming**: Configurable message rates up to 10Hz
- **CSV Export**: Timestamped telemetry data for post-flight analysis
- **EKF Status Monitoring**: Extended Kalman Filter health checks

### 🔧 System Management
- **Thermal Control**: Automatic fan control with hysteresis to prevent thermal throttling
- **GPIO Management**: Hardware interfacing via gpiozero
- **Raspberry Pi Optimization**: Tailored for ARM architecture

## 🏗️ Architecture

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

## 🛠️ Technologies Used

### Core Technologies
- **Python 3.x**: Primary programming language
- **DroneKit**: Python API for MAVLink vehicle control
- **Ultralytics YOLO**: State-of-the-art object detection
- **OpenCV**: Computer vision and image processing
- **Flask**: Web framework for video streaming

### Hardware Integration
- **PiCamera2**: Raspberry Pi camera interface
- **gpiozero**: GPIO control for peripherals
- **MAVLink**: Micro Air Vehicle communication protocol

### Development & Testing
- **SITL (Software In The Loop)**: Safe simulation environment
- **ArduPilot/PX4**: Open-source flight controller firmware

## 📋 Hardware Requirements

### Required
- **Raspberry Pi 4/5** (4GB+ RAM recommended)
- **Raspberry Pi Camera Module** (v2 or HQ camera)
- **Flight Controller** with MAVLink support (Pixhawk, Cube, etc.)
- **Drone Frame** with motors, ESCs, and power system

### Optional
- **Cooling Fan** with transistor/relay for thermal management
- **GPS Module** (typically included with flight controller)
- **Telemetry Radio** for long-range communication

## 🚀 Installation

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
# YOLOv8 nano (general object detection)
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# YOLO-E 11s segmentation (if needed)
# Model file: yoloe-11s-seg.pt
```

### 4. Hardware Connections

**Flight Controller to Raspberry Pi:**
- USB: `/dev/ttyUSB0` or `/dev/ttyACM0`
- UART (GPIO): `/dev/ttyAMA0` or `/dev/serial0`
- Network: UDP connection for SITL or wireless telemetry

**Camera:** Connect via CSI ribbon cable

**Fan (optional):** GPIO 14 via transistor/relay

## 💻 Usage

### Computer Vision Demo (Local Display)

Run real-time object detection with local display:
```bash
python3 cv_demo.py
```
- Press 'q' to quit
- Shows FPS overlay and bounding boxes

### Web Streaming Server

Stream video with YOLO inference to web browser:
```bash
python3 app.py
```
- Access at: `http://<raspberry-pi-ip>:8000`
- View stream at: `http://<raspberry-pi-ip>:8000/video`

### Autonomous Flight Demo

```bash
# Connect to real drone
python3 auto_demo.py
```

This script:
1. Connects to flight controller via UDP
2. Arms and takes off to 10 meters
3. Flies forward at 1 m/s for 5 seconds
4. Lands autonomously

⚠️ **Safety Warning**: Test in SITL first! Ensure adequate space and safety measures.

### Telemetry Monitoring

Monitor drone state and log data:
```bash
# Real flight controller over USB
python3 autonomous_telemetry.py --conn /dev/ttyUSB0 --baud 115200 --csv telem.csv

# SITL simulation
python3 autonomous_telemetry.py --conn udp:0.0.0.0:14550 --csv telem.csv

# Custom update rate
python3 autonomous_telemetry.py --conn /dev/ttyUSB0 --hz 20 --csv flight_data.csv
```

Output includes:
- GPS fix, satellite count, position
- Attitude (roll, pitch, yaw)
- Velocity, groundspeed, heading
- Battery voltage and current
- Rangefinder distance
- EKF status

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
- Turns fan ON at 45°C
- Turns fan OFF at 40°C
- Logs state changes and periodic heartbeat

## 📁 Project Structure

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

## 🎓 Key Concepts Demonstrated

### Software Engineering
- **Modular Architecture**: Separated concerns (CV, flight control, hardware)
- **Error Handling**: Timeouts, exception handling, graceful degradation
- **Configuration Management**: Argparse for CLI configuration
- **Data Persistence**: CSV logging for telemetry data

### Robotics & Control Systems
- **MAVLink Protocol**: Industry-standard drone communication
- **State Machines**: Mode management and flight state transitions
- **Sensor Fusion**: EKF health monitoring
- **Real-time Systems**: High-rate data streaming and processing

### Computer Vision
- **Deep Learning Inference**: Optimized YOLO deployment
- **Real-time Processing**: Frame capture, inference, and display pipeline
- **Performance Optimization**: FPS monitoring and model selection

### Hardware Integration
- **GPIO Control**: Direct hardware interfacing
- **Camera Interface**: CSI camera integration via PiCamera2
- **Serial Communication**: UART/USB connections to flight hardware

## 🔮 Future Improvements

- [ ] **Vision-based Navigation**: Use object detection for autonomous waypoint navigation
- [ ] **Collision Avoidance**: Integrate obstacle detection with flight control
- [ ] **Mission Planning**: Implement complex flight plans with CV-triggered events
- [ ] **Multi-drone Coordination**: Swarm behavior with visual markers
- [ ] **Advanced Tracking**: Follow-me mode using person detection
- [ ] **Gimbal Control**: Camera stabilization and pointing
- [ ] **Cloud Integration**: Remote monitoring dashboard
- [ ] **ROS 2 Bridge**: Integration with Robot Operating System

## 📊 Performance

### YOLO Inference (Raspberry Pi 4)
- **YOLOv8n**: ~5-8 FPS at 320x320 input
- **YOLO-E 11s-seg**: ~3-5 FPS at 800x800 input
- **Latency**: 125-200ms per frame

### Telemetry Rates
- **Position/Attitude**: Up to 10 Hz
- **GPS/Health**: 2 Hz
- **CSV Logging**: Real-time with minimal overhead

## 🔒 Safety Considerations

⚠️ **Important Safety Notes:**

1. **Always test in SITL first** before deploying to real hardware
2. **Maintain visual line of sight** during flight operations
3. **Follow local regulations** for UAV operation
4. **Use a safety pilot** with manual override capability
5. **Test in open areas** away from people and obstacles
6. **Monitor battery levels** - autonomous landing triggers at low voltage
7. **Verify GPS lock** before arming (6+ satellites recommended)
8. **Implement failsafes** - RTL (Return to Launch) on signal loss

## 👨‍💻 Author

**Felix Lu**

This project showcases skills in:
- Embedded systems programming (Raspberry Pi)
- Real-time computer vision and deep learning
- Autonomous vehicle control systems
- Hardware-software integration
- Python development for robotics applications

## 📄 License

This project is provided as-is for educational and demonstration purposes.

## 🙏 Acknowledgments

- **DroneKit-Python**: Excellent MAVLink abstraction layer
- **Ultralytics**: State-of-the-art YOLO implementation
- **ArduPilot**: Robust open-source autopilot
- **Raspberry Pi Foundation**: Accessible embedded computing platform

---

**⚡ Built with passion for robotics and autonomous systems**
