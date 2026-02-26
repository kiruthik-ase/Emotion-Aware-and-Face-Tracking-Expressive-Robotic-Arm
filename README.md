<div align="center">

<h1>🤖 Emotion-Reactive Desk Assistant Robot</h1>

<p><strong>A real-time AI-powered robotic arm that sees your face, reads your emotions, and physically reacts — tracking you with its body and expressing feelings through movement.</strong></p>

<br/>

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Face%20Detection-FF6B6B?style=for-the-badge)
![DeepFace](https://img.shields.io/badge/DeepFace-Emotion%20AI-9B59B6?style=for-the-badge)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3317?style=for-the-badge&logo=opencv&logoColor=white)

<br/>

> *"A robot that doesn't just move — it understands how you feel."*

</div>

---

## 🌟 What Is This?

This project is a **Semester 4 Robotics Project** — a 3-servo robotic arm controlled by a **computer vision + emotion AI pipeline**. It connects a live webcam feed to an Arduino Uno over USB serial.

Here's what happens in real time:
- 👁️ The camera sees your face
- 🧠 AI detects your emotion (happy, sad, angry, surprised, neutral, bored, excited)
- 🤖 The robot arm **physically tracks your face** left/right and up/down
- 💪 The **elbow joint expresses your emotion** with a specific pose
- 📟 An OLED display on the robot shows your current emotion label

---

## 🎥 Demo

```
Webcam → YOLO Face Detect → DeepFace Emotion → Filter + State Machine → Arduino Serial → 3 Servos Move
```

| Emotion    | Elbow Angle | Behaviour                         |
|------------|-------------|-----------------------------------|
| 😊 HAPPY   | 130°        | Arm raised high — celebratory     |
| 😢 SAD     | 60°         | Arm drooped down — dejected       |
| 😡 ANGRY   | 40°         | Arm pulled tight — tense          |
| 😲 SURPRISE| 150°        | Arm fully extended — shocked      |
| 😐 NEUTRAL | 90°         | Arm at rest — relaxed             |
| 😴 BORED   | 80°         | Arm slightly drooped — disengaged |
| 🤩 EXCITED | 130°        | Same as happy — energetic         |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Python (PC Side)                         │
│                                                                  │
│  Webcam ──► YOLOv8n-face ──► DeepFace ──► EmotionFilter        │
│                  │                              │                │
│            [face x,y pos]               DerivedEmotionEngine    │
│                  │                   (EXCITED / BORED logic)    │
│                  │                              │                │
│                  └──────────► EmotionStateMachine               │
│                                  (voting window)                 │
│                                       │                          │
│                              TrackerController                   │
│                         ┌────────────┴───────────┐              │
│                    [B:xx S:xx E:xx]          [EMO:xxx]          │
└─────────────────────────────────────────────────────────────────┘
                                │ USB Serial (115200 baud)
┌─────────────────────────────────────────────────────────────────┐
│                      Arduino Uno (Hardware)                      │
│                                                                  │
│   Parse "B:90 S:20 E:130"         Parse "EMO:HAPPY"            │
│          │                                 │                     │
│   moveSmooth() on 3 servos         Display on OLED              │
│   (fast → slow deceleration)                                    │
│                                                                  │
│   D9 → Base Servo (left/right)                                  │
│   D10 → Shoulder Servo (up/down)                                │
│   D11 → Elbow Servo (emotion pose)                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
robo-sem4/
│
├── README.md
│
└── software/
    │
    ├── main_track.py              ← 🚀 MAIN ENTRY POINT — runs everything
    │
    ├── models/
    │   └── yolov8n-face.pt        ← Pre-trained YOLO face detection model
    │
    ├── vision/                    ← 👁️  Computer Vision Layer
    │   ├── emotion_detector.py    ← DeepFace wrapper for emotion inference
    │   ├── face_detector.py       ← Standalone YOLO face detection test
    │   └── test.py                ← Integration test (without tracking)
    │
    ├── perception/                ← 🧹 Signal Filtering Layer
    │   └── filters.py             ← EmotionFilter (confidence + dominance gating)
    │
    ├── emotion_engine/            ← 🧠 Emotion Intelligence Layer
    │   ├── derived.py             ← Custom EXCITED + BORED state detection
    │   └── state_machine.py       ← Voting-based stable emotion state machine
    │
    └── control/                   ← 🦾 Robot Control Layer
        ├── tracker_controller.py  ← Main controller (tracking + emotion → angles)
        └── emotion_to_servo.py    ← Manual pose test script for Arduino
```

---

## 🧩 Module Deep Dive

### 👁️ Vision Layer

#### `vision/emotion_detector.py` — EmotionDetector
Wraps **DeepFace** to classify emotions from a face crop. Since YOLO already isolated the face, DeepFace is told to skip its own detection (`detector_backend="skip"`) — this makes it significantly faster.

```python
detector = EmotionDetector()
emotions = detector.predict(face_crop)
# → {"happy": 0.82, "neutral": 0.10, "sad": 0.04, ...}
```

---

### 🧹 Perception Layer

#### `perception/filters.py` — EmotionFilter
Raw emotion scores are noisy and jittery. This filter applies **3 gates** before accepting a reading:

| Gate | Rule |
|------|------|
| **Confidence Gate** | Top emotion must score ≥ 55% |
| **Neutral Suppression** | If neutral wins by < 15%, prefer the runner-up |
| **Dominance Gate** | Top emotion must beat 2nd by ≥ 15% margin |

If any gate fails → the frame is thrown out (returns `None`).

---

### 🧠 Emotion Engine

#### `emotion_engine/derived.py` — DerivedEmotionEngine
DeepFace only has 7 basic emotions. We added **2 custom derived states**:

| Derived Emotion | Trigger Condition |
|-----------------|-------------------|
| **EXCITED** | `happy > 60%` AND `surprise > 20%` simultaneously |
| **BORED** | Person is `neutral > 90%` for **5+ continuous seconds** |

Derived emotions take **priority** over base emotions.

#### `emotion_engine/state_machine.py` — EmotionStateMachine
Even after filtering, an emotion can flicker. The state machine uses **majority voting** on a rolling window:

```
Window: [HAPPY, HAPPY, NEUTRAL, HAPPY, HAPPY, HAPPY, HAPPY, HAPPY, HAPPY, HAPPY]
Threshold: 7 / 12 votes → HAPPY wins → robot switches to HAPPY pose
```
This makes the robot feel **calm, deliberate, and confident** in its reactions.

---

### 🦾 Control Layer

#### `control/tracker_controller.py` — TrackerController
The heart of the robot. Handles:

**1. Face Tracking (Base + Shoulder)**
```
error_x = face_center_x - screen_center_x
base_angle    = 90  +  error_x * 0.20   (rotate toward face)
shoulder_angle  = 20  -  error_y * 0.16   (tilt toward face)
```
A **10-pixel deadzone** prevents micro-jitter when your face is nearly centered.

**2. Emotion → Elbow Angle lookup table**

**3. Serial Protocol**
```
B:90 S:20 E:130\n   ← Servo command
EMO:HAPPY\n         ← OLED display command
```
Duplicate commands are suppressed — the Arduino isn't spammed every frame.

---

## ⚡ Arduino Firmware

The Arduino runs a smooth motion algorithm:

```cpp
void moveSmooth(int &cur, int target, Servo &s) {
    while (cur != target) {
        int dist = abs(target - cur);
        int step = (dist > SWITCH_ZONE) ? STEP_FAST : STEP_SLOW;
        // Fast when far → Slow as it approaches
        cur += (target > cur) ? step : -step;
        s.write(cur);
        delay(10);
    }
}
```

- **Fast phase**: 4° steps when far from target
- **Slow phase**: 1° steps when within 15° of target
- Creates a **natural deceleration** feel — not robotic and jerky

**Serial command format:**
```
B:90 S:20 E:120\n    → moves all 3 servos
EMO:HAPPY\n          → updates OLED emotion display
```

---

## 🔌 Hardware Setup

### Components

| Component | Details |
|-----------|---------|
| Microcontroller | Arduino Uno |
| Servos (×3) | Standard hobby servos (SG90 or MG996R) |
| Camera | USB Webcam or built-in laptop camera |
| Display (optional) | I2C OLED (128×64) |
| Power | External 5V supply for servos (don't power from Arduino 5V!) |

### Servo Wiring

| Servo | Arduino Pin | Movement | Angle Range |
|-------|------------|----------|-------------|
| Base  | D9  | Left ↔ Right (tracks face) | 60° – 120° |
| Shoulder | D10 | Up ↔ Down (tracks face) | 0° – 60° |
| Elbow | D11 | Emotion expression | 30° – 150° |

### Serial Connection
```
PC (Python) ──── USB ──── Arduino Uno (COM port)
Baud Rate: 115200
```

---

## 🚀 Getting Started

### Prerequisites

- Python 3.10+
- Arduino IDE
- Arduino Uno connected via USB

### 1. Clone the Repository

```bash
git clone https://github.com/yourusername/robo-sem4.git
cd robo-sem4
```

### 2. Create a Virtual Environment

```bash
python -m venv venv

# Windows
venv\Scripts\activate

# macOS/Linux
source venv/bin/activate
```

### 3. Install Python Dependencies

```bash
pip install ultralytics deepface opencv-python torch pyserial
```

### 4. Upload Arduino Firmware

1. Open **Arduino IDE**
2. Load the sketch from the `notes` file (copy the `#include <Servo.h>` code block)
3. Select your board: `Tools → Board → Arduino Uno`
4. Select your port: `Tools → Port → COMx`
5. Click **Upload**

### 5. Configure the Serial Port

In `software/main_track.py`, update the COM port to match your Arduino:
```python
controller = TrackerController(port="COM8")  # ← Change this to your port
```

### 6. Run the Project

```bash
cd software
python main_track.py
```

A window titled **"Desk Assistant Robot"** will open. Point the camera at your face and watch the arm follow you and react to your emotions!

---

## 🎮 Controls

| Key | Action |
|-----|--------|
| `Q` | Quit the application cleanly (robot returns to neutral) |
| Close window | Same as pressing Q |

---

## 🧪 Testing Individual Components

Test each layer independently before running the full system:

```bash
# Test ONLY face detection (no emotion, no arm)
python software/vision/face_detector.py

# Test emotion + arm WITHOUT face tracking
python software/vision/test.py

# Manually send poses to Arduino (quick hardware test)
python software/control/emotion_to_servo.py
```

---

## 🛠️ Configuration & Tuning

All key parameters are in `main_track.py` and `tracker_controller.py`:

```python
# How aggressively the arm follows your face
TRACK_GAIN_X = 0.20   # left/right sensitivity
TRACK_GAIN_Y = 0.16   # up/down sensitivity

# Ignore face movements smaller than this (prevents jitter)
DEADZONE = 10          # pixels

# Emotion filter thresholds
EmotionFilter(
    window_size=12,        # frames to consider
    min_confidence=0.55,   # minimum score to count
    dominance_margin=0.15  # must beat runner-up by this much
)

# How long neutral = BORED
DerivedEmotionEngine(bored_time_sec=5.0)

# State machine stability
EmotionStateMachine(window_size=12, threshold=7)
# → emotion needs 7/12 votes to take effect
```

---

## 🧪 Tech Stack

| Technology | Role |
|------------|------|
| **YOLOv8n-face** | Real-time face detection (Ultralytics) |
| **DeepFace** | Emotion classification from face crops |
| **OpenCV** | Webcam capture, image processing, display |
| **PyTorch** | GPU acceleration (CUDA if available) |
| **PySerial** | USB serial communication to Arduino |
| **Arduino + Servo.h** | Hardware servo motor control |

---

## 📊 Performance

- **Face Detection**: ~30+ FPS on modern hardware (YOLO is very fast)
- **Emotion Detection**: ~5–10 FPS (DeepFace is heavier — runs every frame but is the bottleneck)
- **Serial Latency**: ~2ms (115200 baud, negligible)
- **Motion Smoothing**: Handled purely by Arduino firmware (no Python overhead)
- **GPU**: CUDA is used automatically if an NVIDIA GPU is present

---

## 📐 Design Decisions

> **Why YOLO for faces instead of letting DeepFace detect?**
> YOLO is significantly faster and more accurate for real-time detection. We use `detector_backend="skip"` in DeepFace so it only classifies emotion on the already-cropped face — no redundant detection.

> **Why 3 layers of emotion stability (filter → derived → state machine)?**
> Raw emotion readings are extremely noisy frame-to-frame. Without filtering, the robot would look like it's having a seizure. Each layer adds a different kind of stability — confidence gating, custom logic, and temporal voting.

> **Why separate base/shoulder from elbow?**
> Clean separation of concerns: 2 joints chase your physical position; 1 joint expresses emotional state. This makes the robot feel like it has a *body* and a *soul*.

---
</div>
