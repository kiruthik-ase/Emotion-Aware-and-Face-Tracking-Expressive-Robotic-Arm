<div align="center">

<h1>Emotion-Aware Face-Tracking Expressive Robotic Arm</h1>

<p><em>A real-time AI-powered 3-DOF robotic arm that detects your face, tracks it, reads your emotion, and physically expresses a response — complete with an OLED face display.</em></p>

![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Face%20Detection-FF6B6B?style=for-the-badge)
![DeepFace](https://img.shields.io/badge/DeepFace-Emotion%20AI-9B59B6?style=for-the-badge)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-5C3317?style=for-the-badge&logo=opencv&logoColor=white)

**[Live Project Website](https://kiruthik-ase.github.io/Emotion-Aware-and-Face-Tracking-Expressive-Robotic-Arm/)** · **Group 3** · 22AIE214 — AI in Robotics · Amrita School of Engineering

</div>

---

## Highlights

- **Real-time face tracking** — base and shoulder servos physically follow your face using proportional control
- **Emotion-aware poses** — detects 7+ emotions and expresses each one through unique arm configurations
- **3-layer noise-resilient pipeline** — confidence gating → derived emotion engine → majority-vote state machine prevents jitter
- **Compound derived emotions** — detects EXCITED and BORED beyond DeepFace's 7 base emotions
- **EMA-smoothed output** — exponential moving average (α=0.30) ensures smooth, natural servo motion
- **Demo mode** — press `D` to cycle through all emotion poses automatically; `N` to skip
- **Idle breathing** — gentle scanning motion when no face is detected, so the robot looks alive
- **Safe shutdown** — Ctrl+C, Q key, or window close all guarantee return to neutral (3-attempt retry)
- **OLED feedback** — Arduino displays pixel-art emotion faces in real-time

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Face Detection | **YOLOv8n-face** (Ultralytics) — GPU-accelerated, real-time |
| Emotion Recognition | **DeepFace** — emotion analysis on pre-cropped face regions |
| Vision Pipeline | **OpenCV** — webcam capture, display, frame processing |
| Serial Comms | **PySerial** — USB @ 115200 baud to Arduino Uno |
| Hardware | **Arduino Uno + 3× SG90 Servos + SSD1306 OLED** (128×64, I²C) |
| Acceleration | **PyTorch / CUDA** — auto GPU if available |

---

## Architecture

```
Webcam → YOLOv8n-face (face box + position)
              │
         ┌────┴────┐
    [face_x, face_y]  [cropped face image]
         │                    │
         │              DeepFace (raw emotions)
         │                    │
         │           EmotionFilter (confidence gate)
         │                    │
         │           DerivedEmotionEngine (EXCITED/BORED)
         │                    │
         │           EmotionStateMachine (majority vote)
         │                    │
         └─── TrackerController ──┘
                    │
            USB Serial → Arduino Uno
                    │
         Base ─ Shoulder ─ Elbow Servos
                    │
              SSD1306 OLED
```

---

## Emotion → Servo Pose Mapping

The arm **reacts** to the detected emotion (doesn't mirror it):

| Emotion | Base | Shoulder | Elbow | Expression |
|---------|------|----------|-------|------------|
| 😊 HAPPY | 100° | 20° | 120° | Cheerful, arm raised & extended |
| 🤩 EXCITED | 100° | 10° | 140° | High energy, big extension |
| 😲 SURPRISE | 90° | 5° | 150° | Fully extended, shocked |
| 😐 NEUTRAL | 90° | 0° | 90° | Relaxed rest position |
| 😴 BORED | 95° | 10° | 80° | Slightly drooped, restless |
| 😢 SAD | 80° | 50° | 60° | Droopy, arm low & folded |
| 😡 ANGRY | 110° | 35° | 40° | Tense, folded tight |

---

## Control Architecture

| Servo | Control Method | Details |
|-------|---------------|---------|
| **Base** (D9, 60°–120°) | Face tracking + emotion offset | Gain: 0.25 · horizontal error, 8px deadzone |
| **Shoulder** (D10, 0°–60°) | 80% emotion pose + 20% tracking | Gain: 0.25 · vertical error, EMA α=0.30 |
| **Elbow** (D11, 30°–150°) | 100% emotion-driven | Pure pose expression, EMA smoothed |

**Stability features:**
- EMA smoothing (α=0.30) on all outputs
- ≥2° change threshold to send commands
- Rate-limited to ~12 commands/sec (80ms interval)
- Serial buffer reset before each write
- 2-second heartbeat send prevents freeze

---

## Project Structure

```
software/
├── main_track.py                ← Entry point (face tracking + emotion + arm control)
├── models/yolov8n-face.pt       ← Pre-trained face detection model
├── vision/
│   └── emotion_detector.py      ← DeepFace emotion inference wrapper
├── perception/
│   └── filters.py               ← Confidence gating + dominance filter
├── emotion_engine/
│   ├── derived.py               ← EXCITED / BORED compound emotion logic
│   └── state_machine.py         ← Majority-vote emotion stabiliser (12-window, 7-threshold)
└── control/
    ├── tracker_controller.py    ← Always-tracking + emotion pose controller (EMA + rate limiting)
    └── emotion_to_servo.py      ← Standalone servo test utility

docs/                            ← GitHub Pages website
├── index.html                   ← Full project documentation with math, architecture, results
├── css/style.css                ← Dark theme styling
└── js/
    ├── main.js                  ← Sidebar navigation + scroll handling
    └── simulation3d.js          ← Interactive 3D arm simulation (Three.js, FK/IK modes)
```

---

## Getting Started

```bash
# Clone
git clone https://github.com/kiruthik-ase/Emotion-Aware-and-Face-Tracking-Expressive-Robotic-Arm.git
cd Emotion-Aware-and-Face-Tracking-Expressive-Robotic-Arm
```

```bash
# Install dependencies
pip install -r requirements.txt
```

```bash
# Set your Arduino COM port in main_track.py if not COM4
# Upload the Arduino sketch (Servo.h + OLED, see notes)

# Run
cd software
python main_track.py
```

**Controls:**
- **Q** or close window → clean exit (arm returns to neutral)
- **Ctrl+C** → clean exit (signal handler)
- **D** → toggle demo mode (cycles all emotion poses)
- **N** → next emotion in demo mode

---

## Performance

| Metric | Value |
|--------|-------|
| Face Detection | ~30 FPS (YOLOv8, GPU) |
| Emotion Inference | ~5–10 FPS (DeepFace — bottleneck) |
| Serial Latency | ~2 ms |
| Servo Command Rate | ~12/sec (rate-limited) |
| Emotion Switch Time | 1–2 sec (7/12 majority vote) |
| Motion Profile | Dual-zone: 4°/step (far) + 1°/step (near) |

---

## Hardware

| Component | Specification |
|-----------|--------------|
| Controller | Arduino Uno (ATmega328P) |
| Base Servo (D9) | SG90, 60°–120° |
| Shoulder Servo (D10) | SG90, 0°–60° |
| Elbow Servo (D11) | SG90, 30°–150° |
| Display | SSD1306 OLED, 128×64px, I²C @ 0x3C |
| Arm Geometry | L1=9cm (upper), L2=8cm (forearm), d1=5cm (base height) |
| Serial | USB @ 115200 baud, protocol: `B:{base} S:{shoulder} E:{elbow}\n` |

---

## Contributors

| Name | Roll Number |
|------|------------|
| Kiruthikpranav | CB.SC.U4AIE24023 |
| Ramkumar K R | CB.SC.U4AIE24042 |
| Dheeraj | CB.SC.U4AIE24050 |
| Vikramendraa | CB.SC.U4AIE24060 |

**Group 3** · Amrita School of Artificial Intelligence · Course 22AIE214 — AI in Robotics


