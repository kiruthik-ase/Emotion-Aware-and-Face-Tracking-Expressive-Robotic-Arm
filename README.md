<div align="center">

<h1>Emotion Aware and Face Tracking Expressive Robotic Arm</h1>

<p><em>A real-time AI-powered robotic arm that sees your face, tracks it, reads your emotion, and physically reacts.</em></p>

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Face%20Detection-FF6B6B?style=for-the-badge)
![DeepFace](https://img.shields.io/badge/DeepFace-Emotion%20AI-9B59B6?style=for-the-badge)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3317?style=for-the-badge&logo=opencv&logoColor=white)

</div>

---

## What Makes This Special

- 🎯 **Real-time face tracking** — the arm physically follows your face using two servo joints
- 🧠 **Emotion-aware movement** — detects 7 emotions and expresses them through a dedicated elbow servo
- 🔗 **End-to-end pipeline** — from webcam frame to servo pulse in milliseconds
- 🛡️ **Noise-resilient AI** — 3-layer emotion stability pipeline (confidence filtering → derived states → voting state machine) prevents jitter and false triggers
- 💡 **Custom derived emotions** — goes beyond DeepFace's 7 defaults; detects **EXCITED** and **BORED** using compound logic
- 📟 **OLED feedback** — Arduino displays the detected emotion as text in real time

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Face Detection | **YOLOv8n-face** (Ultralytics) — fast, accurate, GPU-accelerated |
| Emotion Recognition | **DeepFace** — skips re-detection since YOLO already crops the face |
| Vision Pipeline | **OpenCV** — webcam capture, display, drawing |
| Serial Comms | **PySerial** — USB serial at 115200 baud to Arduino Uno |
| Hardware | **Arduino Uno + 3× SG90 Servos** — smooth motion firmware on-board |
| Acceleration | **PyTorch / CUDA** — automatically uses GPU if available |

---

## Architecture

```
Webcam → YOLOv8 (face box) → DeepFace (raw emotions)
              │                        │
        [x, y position]        EmotionFilter → DerivedEmotionEngine → StateMachine
              │                                                              │
              └──────────────── TrackerController ─────────────────────────-┘
                                       │
                          USB Serial → Arduino Uno
                                       │
                          Base ── Shoulder ── Elbow Servos
```

---

## Emotion → Motion Mapping

| Emotion | Elbow Pose | Feel |
|---------|-----------|------|
| 😊 HAPPY / 🤩 EXCITED | 130° | Raised — celebratory |
| 😲 SURPRISE | 150° | Fully extended — shocked |
| 😐 NEUTRAL | 90° | Rest position |
| 😴 BORED | 80° | Slightly drooped |
| 😢 SAD | 60° | Drooped down |
| 😡 ANGRY | 40° | Pulled tight |

---

## 📁 Project Structure

```
software/
├── main_track.py              ← Entry point — runs the full pipeline
├── models/yolov8n-face.pt     ← Pre-trained face detection model
├── vision/
│   └── emotion_detector.py    ← DeepFace emotion inference wrapper
├── perception/
│   └── filters.py             ← Confidence + dominance gating filter
├── emotion_engine/
│   ├── derived.py             ← EXCITED / BORED compound emotion logic
│   └── state_machine.py       ← Majority-voting emotion stabiliser
└── control/
    └── tracker_controller.py  ← Face tracking + emotion → servo angles
```

---

## Getting Started

```bash
git clone https://github.com/yourusername/Emotion-Aware-and-Face-Tracking-Expressive-Robotic-Arm.git
```

```bash
# 1. Install dependencies
pip install ultralytics deepface opencv-python torch pyserial

# 2. Set your Arduino COM port in main_track.py
controller = TrackerController(port="COM8")

# 3. Upload the Arduino sketch (Servo.h based, see notes file)

# 4. Run
cd software
python main_track.py
```

> Press **`Q`** or close the window to exit cleanly. The arm returns to neutral automatically.

---

## 📊 Performance

- **~30 FPS** face detection (YOLO, GPU)
- **~5–10 FPS** emotion inference (DeepFace — the bottleneck)
- **~2ms** serial latency
- Smooth servo motion handled entirely on the Arduino (no Python overhead)

---


