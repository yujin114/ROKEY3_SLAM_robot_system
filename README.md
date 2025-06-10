<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/TurtleBot4-robot-brightgreen?logo=raspberrypi" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/C++-language-00599C?logo=c%2B%2B" />
  <img src="https://img.shields.io/badge/YOLOv8-CV-red?logo=opencv" />
  <img src="https://img.shields.io/badge/EasyOCR-OCR-orange?logo=openai" />
  <img src="https://img.shields.io/badge/MySQL-database-blue?logo=mysql" />
  <img src="https://img.shields.io/badge/FastAPI-backend-green?logo=fastapi" />
  <img src="https://img.shields.io/badge/Docker-container-2496ED?logo=docker" />
  <img src="https://img.shields.io/badge/Nav2-navigation-critical?logo=mapbox" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>

# 🚓 TurtleBot4 Multi-Robot Park Patrol

**Autonomous illegal parking detection system using TurtleBot4 robots**, developed as part of the **Doosan Rokey Bootcamp 3 in 2025**.

---

## 📌 Overview

This project demonstrates a **multi-robot surveillance system** using two TurtleBot4 robots. The robots autonomously patrol an area, detect parked vehicles, recognize license plates, and verify them against a database. If a violation is detected, the robot triggers a warning signal (🔊 *beep-boop alert*).

---

## 🎯 Use Case Scenario

1. A vehicle enters a **blind zone** and is detected using **YOLO-based vision**.
2. The nearest TurtleBot4 robot executes **Nav2 navigation** to approach the vehicle.
3. The robot performs **local patrol** and identifies any **parked vehicle**.
4. The license plate number and current position are sent to a **central system controller**.
5. The controller queries a **local database** to verify the license plate.
6. If the vehicle is unauthorized → 🚨 **illegal parking alert** is triggered.
7. The alert is visualized on a **GUI dashboard** and stored in a **monitoring database**, displaying the **vehicle number and zone information**.
8. The system operates in **two zones**, each covered by a **dedicated TurtleBot4 robot** (multi-robot architecture).

---

## 🎥 Demo Video

<p align="center">
  <a href="https://youtu.be/ikRZk5629sc">
    <img src="https://img.youtube.com/vi/ikRZk5629sc/0.jpg" alt="Watch the video"/>
  </a>
</p>


---


## 🔁 System Architecture
<p align="center">
  <img src="https://github.com/user-attachments/assets/87b2cd6b-dec9-4c9d-9b13-1d4321aad650" alt="image" width="60%"/>
</p>

* `robot0`: Patrols Zone 1, checks vehicle presence, detects license plate, and raises alert.
* `robot1`: Operates in Zone 2, performs same tasks independently.
* **Shared Services**:

  * License plate OCR module
  * Vehicle number database service

---

## 🔧 Features

### 🚘 AI Computer Vision (YOLOv8 + OCR + Depth)
- Detects vehicles using **YOLOv8** object detection.
- Recognizes license plates using **Easy OCR** with **depth-guided localization**.
- Measures distance to license plate using a **depth camera** to enhance recognition accuracy.
<p align="center">
  <img src="https://github.com/user-attachments/assets/ce79af0a-64f4-4c7b-b4be-d8aa1e960cdf" width="30%" alt="obt1"/>
  <img src="https://github.com/user-attachments/assets/ecc03aea-abe6-4bbc-99b6-1a25151a2344" width="38%" alt="obt2"/>
</p>

### 🗺️ Custom Navigation Graph with BFS
- Implements a **custom waypoint-based navigation graph**.
- Employs a **Breadth-First Search (BFS)** algorithm for optimal path planning across patrol zones.
- Supports **automatic saving and loading** of waypoint configurations in YAML format, ensuring persistence and reusability.

  
<p align="center">
  <img src="https://github.com/user-attachments/assets/391fbd0a-d164-4aee-8ad7-f4cfebed90b0" width="55%" alt="normalvsBFS"/>
</p>


### 📊 Real-Time Dashboard
- Monitors robot patrol status and OCR results in real time.
- Displays **unauthorized vehicle detections** by zone.
- Visual feedback for system alerts and mission states.
<p align="center">
  <img src="https://github.com/user-attachments/assets/721c0d23-95d1-4b73-9345-4e0688aaf56f" width="55%" alt="dashboard"/>
</p>

### 🗃️ Local Database Integration
- Maintains a **whitelist** of authorized license plates.
- Matches OCR results against the database to determine **illegal parking**.
- Sends alert messages if an unauthorized vehicle is detected.
<p align="center">
  <img src="https://github.com/user-attachments/assets/26c331e9-5fde-4b31-92a6-7484fcd3b6f5" width="45%" alt="image1"/>
</p>

---

## 📄 Documentation

For a detailed explanation of this project, please refer to the following document:

👉 [docs/F-1&2_지능1_이재호_배재성_전유진_정은영_김태영_김도엽_한준모.pdf](docs/F-1&2_지능1_이재호_배재성_전유진_정은영_김태영_김도엽_한준모.pdf)


---
## 👥 Contributors

Thanks to these wonderful people who have contributed to this project:

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/weedmo">
        <img src="https://github.com/weedmo.png" width="100px;" alt="weedmo"/><br />
        <sub><b>weedmo</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/yujin114">
        <img src="https://github.com/yujin114.png" width="100px;" alt="yujin114"/><br />
        <sub><b>yujin114</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/ethica-alt">
        <img src="https://github.com/ethica-alt.png" width="100px;" alt="ethica-alt"/><br />
        <sub><b>ethica-alt</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/jsbae-RL">
        <img src="https://github.com/weedmo.png" width="100px;" alt="jsbae-RL"/><br />
        <sub><b>jsbae-RL</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/SmiteLims">
        <img src="https://github.com/SmiteLims.png" width="100px;" alt="SmiteLims"/><br />
        <sub><b>SmiteLims</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/taeyoung730">
        <img src="https://github.com/taeyoung730.png" width="100px;" alt="taeyoung730"/><br />
        <sub><b>taeyoung730</b></sub>
      </a>
    </td>
    <td align="center">
      <a href="https://github.com/kimdoyub">
        <img src="https://github.com/kimdoyub.png" width="100px;" alt="kimdoyub"/><br />
        <sub><b>kimdoyub</b></sub>
      </a>
    </td>
  </tr>
</table>


---

