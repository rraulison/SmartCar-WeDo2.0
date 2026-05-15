# SmartCar - LEGO WeDo 2.0 Vision Control

This project implements a computer vision-based control system for a LEGO WeDo 2.0 SmartCar. It uses MediaPipe for hand/pose tracking and Bleak for Bluetooth Low Energy (BLE) communication with the WeDo 2.0 Hub.

## Inspiration
This project is inspired by the **LEGO WeDo 2.0 Tank** created by **Yoshihito Isogawa**. 
You can watch the original build here: [LEGO WeDo 2.0 Tank (YouTube)](https://www.youtube.com/watch?v=FXIxTJCT5ew)


## Features
- **Computer Vision Control**: Control motor speed by moving your hands in front of the camera.
- **Continuous Control**: Smooth speed transitions and power budgeting.
- **Current Protection**: Monitoring for low voltage and high current alerts from the hub.
- **Motor Dithering**: Synthesizes intermediate speeds to overcome motor stalling.
- **Self-Test**: Initial diagnostic to verify motor functionality.

## Prerequisites
- Python 3.8+
- Webcam
- LEGO WeDo 2.0 Smart Hub

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/SmartCar-WeDo2.0.git
   cd SmartCar-WeDo2.0
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

Run the main script:
```bash
python main.py
```

### Command Line Arguments
- `--dither`: Choose between `sigma`, `spread`, or `discrete` (default: `sigma`).
- `--period`: BLE update interval in seconds (default: 0.06).
- `--no-self-test`: Disable the initial motor self-test.
- `--hand-confidence`: Minimum confidence for hand detection (default: 0.65).

## License
MIT
