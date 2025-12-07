# 🔫 AutoLocker — AI-Powered Aim-Assisted Sentry Turret

An autonomous sentry turret capable of **real-time human detection, tracking, and foam-dart firing** using **computer vision and embedded actuation**.  
AutoLocker combines a live camera feed, deep learning–based person recognition, and a servo-controlled pan-tilt system to lock onto a target and engage automatically.

> ⚠️ This project is strictly for academic and research purposes and is designed only for safe foam projectiles. It must not be used in harmful or dangerous applications.

---

## 🚀 Key Features
- Real-time human/face detection using OpenCV and deep learning  
- Automated target tracking with **PID-stabilized pan-tilt control**
- Autonomous firing logic after target lock
- Live video feed with bounding box + lock indicator
- Safety mode with cooldown and distance restriction

---


---

## 📷 Demo
📌 *Add your demo video link here once uploaded*  
`https://drive.google.com/...`

---

## 🔧 Hardware Requirements
| Component | Function |
|----------|----------|
| Arduino Uno/Laptop | Computer vision & control |
| USB Camera | Live video feed |
| 2× Servo Motors (MG995) | Pan & tilt motion |
| High-speed DC Motor | Flywheel for dart firing |
| Linear Actuator | Dart-push mechanism |
| Power Supply (5-9V) | Motors and servos |
| 3D-printed Turret Mount | Mechanical frame |

---

## 💻 Software Stack
| Component | Technology |
|----------|------------|
| Language | Python |
| CV Framework | OpenCV |
| Embedded Control | pySerial / pigpio / Arduino |
| Optional Model | Haar Cascade / MobileNet / YOLO |
| 3D Printing | UltiMaker Cura |

---

