# 🚓 TurtleBot4 Multi-Robot Surveillance System

**Autonomous illegal parking detection system using TurtleBot4 robots**, developed as part of the **Doosan Robotics Bootcamp 2025**.

---

## 📌 Overview

This project demonstrates a **multi-robot surveillance system** using two TurtleBot4 robots. The robots autonomously patrol an area, detect parked vehicles, recognize license plates, and verify them against a database. If a violation is detected, the robot triggers a warning signal (🔊 *beep-boop alert*).

---

## 🎯 Use Case Scenario

1. A vehicle is detected by the surveillance system (YOLO-based vision).
2. A TurtleBot4 robot is dispatched to the location.
3. The robot captures the license plate and performs **OCR-based recognition**.
4. It checks the plate number against a local database of authorized vehicles.
5. If the plate is not found → 🚨 **illegal parking alert** is triggered.
6. A speaker plays an alarm sound to notify nearby personnel.
7. The system is deployed in two zones, each covered by an independent TurtleBot4 robot (multi-robot operation).

---

## 🎥 Demo Video

[▶️ Watch the full demo on YouTube](https://your_demo_video_link_here)

---

## 🖼️ License Plate Recognition

<div align="center">
  <img src="https://your_license_plate_recognition_image_link_here.jpg" width="60%">
  <p><i>Example: Successfully detected and matched license plate</i></p>
</div>

---

## 🛠️ Tech Stack

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/TurtleBot4-robot-brightgreen?logo=raspberrypi" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/YOLOv8-CV-red?logo=opencv" />
  <img src="https://img.shields.io/badge/Tesseract-OCR-blueviolet?logo=google" />
  <img src="https://img.shields.io/badge/SQLite-database-lightgrey?logo=sqlite" />
</p>

---

## 🔁 System Architecture

* `robot0`: Patrols Zone 1, checks vehicle presence, detects license plate, and raises alert.
* `robot1`: Operates in Zone 2, performs same tasks independently.
* **Shared Services**:

  * License plate OCR module
  * Vehicle number database service

---

## 🚀 Quick Start

```bash
# Clone the project
git clone https://github.com/your-username/turtlebot4_illegal_parking.git

# Build and source
colcon build --symlink-install
source install/setup.bash

# Launch the multi-robot system
ros2 launch illegal_parking multi_robot.launch.py
```

---

## 👥 Contributors

| Name             | Role                              | Affiliation              |
| ---------------- | --------------------------------- | ------------------------ |
| Junmo Han (한준모)  | Lead Developer / ROS2 Integration | Doosan Robotics Bootcamp |
| \[Contributor 2] | Computer Vision / OCR             | \[Institution or Role]   |
| \[Contributor 3] | System Architecture               | \[Institution or Role]   |

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

---

## 💬 Contact

Feel free to reach out via [GitHub Issues](https://github.com/your-username/turtlebot4_illegal_parking/issues) for feedback or questions.
